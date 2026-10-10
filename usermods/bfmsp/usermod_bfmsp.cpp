// usermod_bfmsp.cpp
// ---------------------------------------------------------------------------
// BfMspUsermod - Betaflight MSP -> WLED bridge usermod（方案 B）
//
// MSP 帧格式（Betaflight）：
//   请求：$ M <  len(1)  cmd(1)  [payload]  crc(1)     crc = XOR(len..payload)
//   应答：$ M >  len(1)  cmd(1)  [payload]  crc(1)
//
// 数据来源（已在 Betaflight 源码 msp.c 中核对字段布局）：
//   MSP_STATUS_EX (150)：cycleTime(2) i2cErr(2) sensors(2) flightModeFlags(4)
//                         load(2) pidProfileCount(1) rateProfileIndex(1)
//     -> flightModeFlags 偏移 6..9（u32 LE）：bit0=ARM, bit1=ANGLE, bit2=HORIZON
//   MSP_ANALOG    (106)：vbat(1, 0.1V) mAh(2) rssi(2) amperage(2, 0.01A) fullVbat(2)
//     -> vbat 偏移 0
//   MSP_RC        (107)：channel[18] 各 u16 LE，通道 3 = 油门（偏移 6..7）
//
// 目标：WLED 16（v16.0.1）。v16 使用自注册机制：
//   - 本文件末尾 REGISTER_USERMOD(bf_msp) 自注册
//   - 配套 usermods/bfmsp/library.json（libArchive:false）
//   - 在 platformio_override.ini 里 custom_usermods = bfmsp 启用
// ---------------------------------------------------------------------------
#include "usermod_bfmsp.h"

#if !defined(ESP32)
  #error "BfMspUsermod requires an ESP32 (incl. ESP32-C3/S3). ESP8266 has no free hardware UART for this."
#endif

// Betaflight MSP 命令号
#define MSP_STATUS_EX 150
#define MSP_ANALOG    106
#define MSP_RC        107

// 简易调试打印（编译时带 WLED_DEBUG 才输出到 Serial）
#ifdef WLED_DEBUG
  #define BFDBG(...) Serial.printf(__VA_ARGS__)
#else
  #define BFDBG(...)
#endif

// ---------------------------------------------------------------------------
// setup：初始化 UART
// 编译宏覆盖示例（platformio_override.ini）：
//   -D BF_MSP_RX_PIN=5   -D BF_MSP_TX_PIN=4   -D BF_MSP_BAUD=115200
// ---------------------------------------------------------------------------
void BfMspUsermod::setup() {
#ifdef BF_MSP_RX_PIN
  rxPin = BF_MSP_RX_PIN;
#endif
#ifdef BF_MSP_TX_PIN
  txPin = BF_MSP_TX_PIN;
#endif
#ifdef BF_MSP_BAUD
  baud = BF_MSP_BAUD;
#endif

  // ESP32-C3 上 Serial1 = UART1，引脚通过 begin() 显式指定
  Serial1.begin(baud, SERIAL_8N1, rxPin, txPin);
  lastPollMs = millis();
  BFDBG("BfMsp: UART1 started rx=%u tx=%u baud=%lu\n", rxPin, txPin, (unsigned long)baud);

  // BOOT 按钮（GPIO9）：strapping 脚只能作输入，开内部上拉（板载通常已有上拉）
#ifdef BF_MSP_BOOT_PIN
  bootPin = BF_MSP_BOOT_PIN;
#endif
  pinMode(bootPin, INPUT_PULLUP);
  BFDBG("BfMsp: BOOT button on GPIO%u\n", bootPin);

  // 灯带开关模式（0=禁用 1=解锁亮 2=反相 3=链路丢失熄灯；也可运行时在 JSON 改）
#ifdef BF_MSP_SWITCH_MODE
  switchMode = BF_MSP_SWITCH_MODE;
#endif
}

// ---------------------------------------------------------------------------
// loop：收帧 -> 定时轮询 -> 链路超时检测 -> 应用状态
// ---------------------------------------------------------------------------
void BfMspUsermod::loop() {
  if (!enabled) return;

  parseSerial();

  uint32_t now = millis();
  if (now - lastPollMs >= pollMs) {
    lastPollMs = now;
    pollFc();
  }

  // 1s 内没收到任何有效应答 => 判定链路丢失
  if (fcConnected && (now - lastFcMs > 1000)) {
    fcConnected = false;
    appliedState = 0xFF; // 强制下次重算
    BFDBG("BfMsp: FC link lost\n");
  }

  checkButton();
  applyState();
}

// ---------------------------------------------------------------------------
// MSP 帧解析：状态机逐字节处理，收到完整帧后校验 CRC 并分发
// ---------------------------------------------------------------------------
void BfMspUsermod::parseSerial() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    if (rxLen == 0) {                    // 等待帧头 '$'
      if (b == '$') rxBuf[rxLen++] = b;
      continue;
    }
    if (rxLen == 1) {                    // 等待 'M'
      rxBuf[rxLen++] = b;
      if (rxBuf[1] != 'M') rxLen = 0;
      continue;
    }
    if (rxLen == 2) {                    // 等待方向 '<'（应答帧）
      rxBuf[rxLen++] = b;
      if (rxBuf[2] != '<') rxLen = 0;
      continue;
    }

    // 其余为帧体：rxBuf[3]=len, rxBuf[4]=cmd, rxBuf[5..]=payload, 末字节=CRC
    rxBuf[rxLen++] = b;

    if (rxLen == 4) {
      if (rxBuf[3] > sizeof(rxBuf) - 6) { // 长度非法，重新同步
        rxLen = 0;
        continue;
      }
    }

    if (rxLen >= 5 && rxLen == 6u + rxBuf[3]) {
      uint8_t crc = 0;
      for (uint8_t i = 3; i < rxLen - 1; i++) crc ^= rxBuf[i];
      if (crc == rxBuf[rxLen - 1]) {
        onMspFrame(rxBuf[4], &rxBuf[5], rxBuf[3]);
      } else {
        BFDBG("BfMsp: CRC fail cmd=%u\n", rxBuf[4]);
      }
      rxLen = 0;
    } else if (rxLen >= sizeof(rxBuf)) { // 溢出保护
      rxLen = 0;
    }
  }
}

// ---------------------------------------------------------------------------
// 定时向飞控发送 MSP 请求（无 payload，len=0）
// ---------------------------------------------------------------------------
void BfMspUsermod::pollFc() {
  sendRequest(MSP_STATUS_EX);
  sendRequest(MSP_ANALOG);
#ifdef BF_MSP_USE_THROTTLE
  sendRequest(MSP_RC);
#endif
}

void BfMspUsermod::sendRequest(uint8_t cmd) {
  uint8_t header[4] = {'$', 'M', '<', 0x00}; // '$','M','<',len=0
  Serial1.write(header, 4);
  Serial1.write(cmd);
  Serial1.write(0x00 ^ cmd);                 // crc = 0 ^ cmd
}

// ---------------------------------------------------------------------------
// BOOT 按钮（GPIO9）：按下沿触发一次 -> 循环切换手动预设
// 注意：GPIO9 是 strapping 脚，只能输入；按住开机 = 进入下载模式
// ---------------------------------------------------------------------------
void BfMspUsermod::checkButton() {
  if (!bootEnabled) return;

  bool level = digitalRead(bootPin); // 上拉：松开 HIGH，按下 LOW
  uint32_t now = millis();

  if (level != lastBtnLevel) {       // 电平变化 -> 重新开始去抖计时
    lastBtnLevel = level;
    lastBtnMs = now;
    return;
  }
  if (level == LOW && !btnTriggered && (now - lastBtnMs >= debounceMs)) {
    btnTriggered = true;             // 稳定按下且超过去抖时间，触发一次
    advanceManualPreset();
  }
  if (level == HIGH) btnTriggered = false; // 松开复位，允许下一次触发
}

void BfMspUsermod::advanceManualPreset() {
  if (presetCycleMax < presetCycleMin) presetCycleMax = presetCycleMin;
  manualPreset++;
  if (manualPreset > presetCycleMax) manualPreset = presetCycleMin;
  applyPreset(manualPreset);
  BFDBG("BfMsp: BOOT pressed -> manual preset %u\n", manualPreset);
}

// ---------------------------------------------------------------------------
// 应答帧分发与状态提取（字段布局依据 Betaflight master msp.c）
// ---------------------------------------------------------------------------
void BfMspUsermod::onMspFrame(uint8_t cmd, uint8_t* p, uint8_t len) {
  switch (cmd) {
    case MSP_STATUS_EX: {
      if (len < 10) break; // 至少要有 flightModeFlags
      uint32_t flags = p[6] | (p[7] << 8) | (p[8] << 16) | ((uint32_t)p[9] << 24);
      armed       = flags & 0x01; // boxId 0 = ARM
      angleMode   = flags & 0x02; // boxId 1 = ANGLE
      horizonMode = flags & 0x04; // boxId 2 = HORIZON
      fcConnected = true;
      lastFcMs    = millis();
      break;
    }
    case MSP_ANALOG:
      if (len >= 1) vbat = p[0]; // 0.1V 单位
      break;
    case MSP_RC:
      if (len >= 8) throttle = p[6] | (p[7] << 8); // 通道 3（油门），u16 LE
      break;
  }
}

// ---------------------------------------------------------------------------
// 灯带开关（受 BF 控制）：独立于预设切换，只控制整灯亮/灭。
// 直接改全局 bri 即可——WLED 主循环每帧会把 bri 同步到灯带总线。
// ---------------------------------------------------------------------------
void BfMspUsermod::applySwitch() {
  bool wantOn = true;
  switch (switchMode) {
    case 1: wantOn = armed; break;        // 解锁亮 / 锁定灭
    case 2: wantOn = !armed; break;       // 锁定亮 / 解锁灭（反相）
    case 3: wantOn = fcConnected; break;  // 链路丢失熄灯
    default: return;                      // 0 = 禁用
  }
  if (wantOn == switchOn) return;         // 开关状态未翻转
  switchOn = wantOn;

  if (wantOn) {
    // 恢复亮度：优先用关灯前记忆的亮度，否则用 WLED 默认亮度 briS(128)；
    // 若用户已手动调亮（bri>0），保留手动值。
    if (bri == 0) bri = savedBri ? savedBri : briS;
  } else {
    savedBri = bri;
    bri = 0;
  }
  BFDBG("BfMsp: switch %s (bri=%u)\n", wantOn ? "ON" : "OFF", bri);
}

// ---------------------------------------------------------------------------
// 状态机：低电压 > 已解锁 > 角度模式 > 未解锁 > 链路丢失，变化时才切换预设
// ---------------------------------------------------------------------------
void BfMspUsermod::applyState() {
  applySwitch();  // 每次循环都评估开关（内部有去重，开销极小）
  uint8_t st;
  if (!fcConnected)                            st = ST_LINKLOST;
  else if (vbat && vbat < lowBattVolt)         st = ST_LOWBATT;
  else if (armed)                              st = ST_ARMED;
  else if (angleMode || horizonMode)           st = ST_ANGLE;
  else                                         st = ST_DISARMED;

  // 若用户用 BOOT 手动选了预设（manualPreset != 0），只要飞控状态不变，
  // appliedState 与当前状态相同，这里不会覆盖手动选择；状态一变自动恢复。
  if (st == appliedState) return;
  appliedState = st;

  switch (st) {
    case ST_DISARMED: applyPreset(presetDisarmed); break;
    case ST_ARMED:    applyPreset(presetArmed);    break;
    case ST_LOWBATT:  applyPreset(presetLowBatt);  break;
    case ST_ANGLE:    applyPreset(presetAngle);    break;
    case ST_LINKLOST: applyPreset(presetLinkLost); break;
  }
  BFDBG("BfMsp: state=%u (armed=%u angle=%u vbat=%u)\n", st, armed, angleMode, vbat);
}

void BfMspUsermod::applyPreset(uint16_t presetId) {
  if (presetId == 0) return; // 0 = 保持当前效果不动
  // WLED 16：presets 对象已移除，applyPreset 改为全局函数（wled.h 声明）；
  // CALL_MODE_FX_ONLY 已删除，改用 CALL_MODE_NO_NOTIFY（改状态但不发 UDP 通知）。
  applyPreset(presetId, CALL_MODE_NO_NOTIFY);
}

// ---------------------------------------------------------------------------
// 可选：将状态暴露到 WLED 的 /json/state（浏览器可实时查看）
// ---------------------------------------------------------------------------
void BfMspUsermod::addToJsonState(JsonObject& root) {
  JsonObject top = root["bfmsp"];
  if (top.isNull()) top = root.createNestedObject("bfmsp");
  top["enabled"]        = enabled;
  top["lowBattVolt"]    = lowBattVolt;
  top["presetDisarmed"] = presetDisarmed;
  top["presetArmed"]    = presetArmed;
  top["presetLowBatt"]  = presetLowBatt;
  top["presetAngle"]    = presetAngle;
  top["presetLinkLost"] = presetLinkLost;
  top["bootEnabled"]    = bootEnabled;
  top["presetCycleMin"] = presetCycleMin;
  top["presetCycleMax"] = presetCycleMax;
  top["manualPreset"]   = manualPreset;
  top["switchMode"]     = switchMode;
  top["switchOn"]       = switchOn;
  top["fcConnected"]    = fcConnected;
  top["armed"]          = armed;
  top["vbat"]           = vbat; // 0.1V 单位
}

void BfMspUsermod::readFromJsonState(JsonObject& root) {
  JsonObject top = root["bfmsp"];
  if (top.isNull()) return;
  enabled        = top["enabled"]        | enabled;
  lowBattVolt    = top["lowBattVolt"]    | lowBattVolt;
  presetDisarmed = top["presetDisarmed"] | presetDisarmed;
  presetArmed    = top["presetArmed"]    | presetArmed;
  presetLowBatt  = top["presetLowBatt"]  | presetLowBatt;
  presetAngle    = top["presetAngle"]    | presetAngle;
  presetLinkLost = top["presetLinkLost"] | presetLinkLost;
  bootEnabled    = top["bootEnabled"]    | bootEnabled;
  presetCycleMin = top["presetCycleMin"] | presetCycleMin;
  presetCycleMax = top["presetCycleMax"] | presetCycleMax;
  switchMode = top["switchMode"] | switchMode;
  if (switchMode > 3) switchMode = 0; // 越界回退到禁用
}

// ---------------------------------------------------------------------------
// v16 自注册：WLED 构建系统通过 custom_usermods = bfmsp 自动装载本 usermod，
// 无需修改 usermods_list.cpp。
// ---------------------------------------------------------------------------
static BfMspUsermod bf_msp;
REGISTER_USERMOD(bf_msp);
