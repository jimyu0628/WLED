// usermod_bfmsp.h
// ---------------------------------------------------------------------------
// BfMspUsermod - Betaflight MSP -> WLED bridge usermod (方案 B)
//
// 功能：ESP32 通过 UART 与飞控双向通信，定时向飞控请求 MSP_STATUS_EX /
//       MSP_ANALOG / MSP_RC，解析出【解锁状态 / 飞行模式 / 电池电压 / 油门】，
//       再调用 WLED 预设自动切换灯光效果；并可让飞控状态直接控制灯带整体
//       开关（解锁亮/锁定灭等，见 switchMode）。
//
// 目标：WLED 16（v16.0.1），ESP32 / ESP32-C3 / ESP32-S3（本工程针对 C3）
// 说明：v16 自注册式 usermod（REGISTER_USERMOD + library.json），
//       不需要修改 usermods_list.cpp；接入方式见 README.md / GITHUB_BUILD_GUIDE.md。
// ---------------------------------------------------------------------------
#pragma once

#include "wled.h"

// 自定义 usermod ID（避免与内置 usermod 冲突，选一个不常用的大数即可）
#define USERMOD_ID_BFMSP 9001

class BfMspUsermod : public Usermod {
  public:
    void setup() override;
    void loop() override;
    uint16_t getId() override { return USERMOD_ID_BFMSP; }   // v16：getId()

    // 可选：把运行状态/配置暴露到 WLED 的 /json/state（便于调试）
    void addToJsonState(JsonObject& root) override;   // v16：引用参数
    void readFromJsonState(JsonObject& root) override;

  private:
    // 灯光状态优先级（由高到低）
    enum BfState : uint8_t {
      ST_DISARMED = 0,  // 未解锁：预设 1
      ST_ARMED    = 1,  // 已解锁：预设 2
      ST_LOWBATT  = 2,  // 低电压：预设 3（优先级高于解锁）
      ST_ANGLE    = 3,  // 角度/自稳模式：预设 4
      ST_LINKLOST = 4,  // 超过 1s 未收到飞控数据：预设 5
    };

    void parseSerial();
    void sendRequest(uint8_t cmd);
    void onMspFrame(uint8_t cmd, uint8_t* payload, uint8_t len);
    void pollFc();
    void applyState();
    void applySwitch();
    void applyPreset(uint16_t presetId);
    void checkButton();
    void advanceManualPreset();

    // -------------------- 配置（可用编译宏或 JSON 覆盖） --------------------
    bool     enabled        = true;   // 总开关
    uint8_t  rxPin          = 5;      // ESP32 RX  <- 飞控 TX（示例 GPIO5）
    uint8_t  txPin          = 4;      // ESP32 TX  -> 飞控 RX（示例 GPIO4）
    uint32_t baud           = 115200; // 必须与飞控 MSP 端口波特率一致
    uint16_t pollMs         = 250;    // 每 250ms 轮询一次飞控
    uint16_t lowBattVolt    = 105;    // 低电压阈值，0.1V 单位 => 10.5V（3S）
    uint16_t presetDisarmed = 1;      // 各状态对应的 WLED 预设 ID（0 = 保持当前效果不动）
    uint16_t presetArmed    = 2;
    uint16_t presetLowBatt  = 3;
    uint16_t presetAngle    = 4;
    uint16_t presetLinkLost = 5;

    // -------------------- BOOT 按钮（GPIO9）手动切预设 ---------------------
    bool     bootEnabled    = true;   // 按钮总开关
    uint8_t  bootPin        = 9;      // ESP32-C3 BOOT 键所在 GPIO（strapping 脚，仅输入）
    uint16_t presetCycleMin = 1;      // 手动循环预设的 ID 范围（含端点）
    uint16_t presetCycleMax = 5;
    uint16_t debounceMs     = 40;     // 按键去抖时间（ms）

    // -------------------- 灯带开关（受 BF 控制，独立于预设切换） -------------
    uint8_t  switchMode = 1;    // 0=禁用 1=解锁亮/锁定灭 2=锁定亮/解锁灭 3=链路丢失熄灯
    bool     switchOn   = true; // 当前开关输出（true=亮）
    uint8_t  savedBri   = 0;    // 关灯前记住的亮度，恢复用

    // ---------------------------- 运行时状态 -------------------------------
    uint8_t  rxBuf[128];          // MSP 帧接收缓冲（最大 payload 122 字节）
    uint8_t  rxLen        = 0;
    uint32_t lastPollMs   = 0;
    uint32_t lastFcMs     = 0;
    uint8_t  vbat         = 0;    // 电池电压，0.1V 单位（如 105 = 10.5V）
    uint16_t throttle     = 0;    // 油门通道值 1000..2000（默认不解析，见编译宏）
    bool     armed        = false;
    bool     angleMode    = false;
    bool     horizonMode  = false;
    bool     fcConnected  = false;
    uint8_t  appliedState = 0xFF; // 已应用的状态，用于去重

    // BOOT 按钮运行时状态
    bool     lastBtnLevel = true;   // 松开 = HIGH（上拉）
    bool     btnTriggered = false;  // 已触发标志，防按住连发
    uint32_t lastBtnMs    = 0;
    uint16_t manualPreset = 0;      // 0 = 未手动选择预设
};
