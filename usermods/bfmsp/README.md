# PAVO20 Pro × WLED(ESP32-C3) — 方案 B：MSP 桥接

用飞控（Betaflight）通过 UART/MSP 控制 WLED 灯带效果的 usermod 工程。

## 目录

- [原理](#原理)
- [文件说明](#文件说明)
- [硬件接线（重点修正）](#硬件接线重点修正)
- [Betaflight 设置](#betaflight-设置)
- [WLED 编译集成](#wled-编译集成)
- [WLED 预设准备](#wled-预设准备)
- [状态机与优先级](#状态机与优先级)
- [故障排查](#故障排查)
- [已知限制](#已知限制)

## 原理

Betaflight 的 MSP 是**请求-应答**协议：ESP32 定时（默认 250ms）向飞控发送
`$M<` 请求帧，飞控回 `$M>` 应答帧。本 usermod 解析三类应答：

| 命令 | 内容 | 用途 |
|---|---|---|
| MSP_STATUS_EX (150) | flightModeFlags（偏移 6..9，u32） | bit0=ARM（解锁）、bit1=ANGLE、bit2=HORIZON |
| MSP_ANALOG (106) | vbat（偏移 0，0.1V 单位） | 低电压告警 |
| MSP_RC (107，可选) | 通道 3 = 油门 | 后续可做油门联动亮度 |

字段布局已对照 Betaflight master 源码 `src/main/msp/msp.c` 与 `msp_box.c` 核对。

## 文件说明

| 文件 | 作用 |
|---|---|
| `usermod_bfmsp.h` | usermod 类定义、配置项与运行时状态 |
| `usermod_bfmsp.cpp` | MSP 帧解析、轮询、状态机、预设切换、v16 自注册 |
| `library.json` | PlatformIO 库清单（`libArchive:false`，v16 必需） |
| `platformio_override.ini` | 自定义编译环境（`env:esp32c3dev_bfmsp`，上传到 WLED 仓库根目录） |
| `wled-custom-build.yaml` | GitHub Actions 在线编译脚本（上传到 `.github/workflows/`） |
| `GITHUB_BUILD_GUIDE.md` | 免本机安装的 GitHub 在线编译逐步操作指南 |
| `README.md` | 本说明 |

## 硬件接线（重点修正）

> 上一轮方案介绍中"FC TX → ESP32 RX 单向"的说法**不完整**：MSP 是请求-应答协议，
> ESP32 必须向飞控**回发请求**，因此需要**双向两线**，外加 5V 与共地。

```
飞控 F4 2-3S 20A AIO V1             ESP32-C3
─────────────────────               ─────────
UART 空闲口 TX  ─────────────────▶  GPIO5 (RX)   应答数据流入
UART 空闲口 RX  ◀─────────────────  GPIO4 (TX)   请求帧流出
5V pad (5V@3A) ─────────────────▶  VIN/5V       （严禁接 VBAT！）
GND             ─────────────────▶  GND          全程共地
```

- **供电**：ESP32 和 WS2812 灯带的 VCC 都接飞控 **5V pad（5V@3A BEC）**。
  VBAT 是 3S（9.9–12.6V），**严禁**直接接灯带或 ESP32。
- **UART 选哪个口**：打开 Betaflight Configurator 的 **Ports 页**确认——
  板载 Serial ELRS 通常占 UART1，DJI O3/O4 的 MSP DisplayPort 可能占 UART3（或 UART4）。
  选一个 Peripheral 显示 "None" 的空闲口（例如 TX4/RX4 或 TX6/RX6），把它配成 MSP。
- **灯带数据线**：ESP32 的灯带数据 GPIO（你 WLED 里配置的那个）→ 灯带 DIN。
  3.3V 逻辑推 5V 灯带绝大多数灯珠批次可用；若偶发乱色/闪烁，加一片 74HCT125 升压缓冲。
- **引脚注意**：示例用 GPIO5/GPIO4，均非 C3 的 strapping 脚（2/8/9），
  且不与常见 C3 板 WLED 默认灯带脚（GPIO8）冲突；具体以你的板子为准。

## Betaflight 设置

1. 连接飞控 USB，打开 Betaflight Configurator；
2. **Ports 页**：找到空闲串口（见上），Peripherals 选 **MSP**，波特率选 **115200**；
3. 确认两端波特率一致（固件里默认 `115200`，可在编译宏 `BF_MSP_BAUD` 覆盖）；
4. 验证：在 Configurator 的 CLI 里 `serial` 能看到该口带 `MSP` 功能即可。

> 若不想用 Configurator，CLI 的 `serial` 命令等价（口序号以当前固件 `serial` 列表为准，
> 直接抄 Configurator Ports 页生成的那行最稳妥）。

## WLED 编译集成（v16 自注册）

主站 WLED 官方固件**不含** Betaflight MSP 功能，必须编译。本 usermod 已按
**WLED 16（v16.0.1）** 的新机制写好：`REGISTER_USERMOD()` 自注册 +
`library.json`，**不需要**再改 `usermods/usermods_list.cpp`。

**推荐路线：GitHub 在线编译（本机零安装）**——详见 `GITHUB_BUILD_GUIDE.md`，
全程浏览器操作，服务器编译后下载固件。核心就三件事：

1. Fork `wled/WLED` → 切到 **v16.0.1** 分支；
2. 上传 `usermods/bfmsp/`（usermod_bfmsp.h / usermod_bfmsp.cpp / library.json）、
   根目录 `platformio_override.ini`、`.github/workflows/wled-custom-build.yaml`；
3. Actions 页 Run workflow，等 15~30 分钟下载 `.bin`，用 WLED 网页 OTA 刷入
   （先刷官方固件打底，见指南第 9 步）。

本机有工具的话也可以本地编译（VSCode + PlatformIO）：

```bash
git clone --depth 1 --branch v16.0.1 https://github.com/wled/WLED.git
# 复制本工程三个文件到 usermods/bfmsp/（h/cpp/library.json）
# 复制 platformio_override.ini 到仓库根目录
pio run -e esp32c3dev_bfmsp
```

> 注意：WLED 对 ESP32-C3 是"实验性支持"，新版本已明显改善；刷入后若 AP 不出现，
> 多半是 C3 板型的引脚/Flash 配置问题，先用官方 Web 安装器（install.wled.me）
> 验证板子能跑标准固件，再刷自编译固件。

## WLED 预设准备

在 WLED 网页 UI 中把想要的效果存成预设（Presets）：

| 预设 ID | 建议内容 | 触发状态 |
|---|---|---|
| 1 | 柔和呼吸/单色常亮 | 未解锁 |
| 2 | 彩虹流水 | 已解锁 |
| 3 | 红色闪烁 | 低电压（<10.5V，3S） |
| 4 | 蓝色跑马 | 角度/自稳模式 |
| 5 | 白色频闪 | 飞控链路丢失 |

预设 ID 在 `/json/state` 的 `bfmsp` 节点可改（或直接改头文件默认值）。
任一项设为 `0` = 该状态保持当前效果不动。

## 状态机与优先级

```
链路丢失(>1s无应答)  >  低电压(vbat<阈值)  >  已解锁  >  角度/自稳模式  >  未解锁
```

状态**变化时**才切换预设（内部去重，避免每 250ms 反复触发）。
飞行中解锁/解锁切换、电压跌破阈值都会实时反映到灯带。

## BOOT 按钮切换预设（GPIO9）

ESP32-C3 开发板上的 **BOOT 键 = GPIO9**（板载按键，无需额外接线）。固件已内置
按键处理：**每按一次，手动循环切换一个预设**（默认在预设 1..5 之间轮换）。

要点：

- **GPIO9 是 strapping 脚**：开机瞬间按住 BOOT 会进入下载模式（刷固件时用）；
  运行时只能作**输入**（代码用 `INPUT_PULLUP`，板载通常还有外部上拉），
  绝不要把它配置成输出或拉低。
- **手动选择与飞控自动逻辑的关系**：按 BOOT 选中的预设会一直保持，
  直到飞控状态（解锁/低电压/角度/链路丢失）**发生变化**，自动预设才重新接管。
  即"手动优先，状态变化后自动恢复"。
- **循环范围**：默认 1..5（就是状态预设那 5 个），可在 `/json/state` 的 `bfmsp`
  节点改 `presetCycleMin` / `presetCycleMax`，或直接改头文件默认值。
- **去抖**：40ms 软件去抖；快速连按每次都会触发一次切换。

如需改引脚（一般不需要）：`platformio_override.ini` 加 `-D BF_MSP_BOOT_PIN=9`。

## 故障排查

| 现象 | 排查 |
|---|---|
| 灯带无反应、`fcConnected=false` | 查接线（TX/RX 是否交叉接反）、波特率、Ports 页 MSP 是否配在空闲口 |
| 帧 CRC 频繁失败 | 波特率不匹配或数据线过长/受干扰；先降到 57600 试 |
| 偶尔乱色/闪烁 | 电机 EMI 或 3.3V 逻辑余量不足：数据线远离电机线、加 74HCT125、5V 端并 100–470µF 电容 |
| 解锁不切效果 | 确认 `armed` 解析：查 Configurator 的 Modes 页是否启用 ARM（bit0 恒为解锁位） |
| 上电反复重启 | 5V BEC 过载：降低灯珠数量/亮度上限，检查 ESP32 峰值电流 |

## 已知限制

- 本 usermod 的 v16 接口（`getId()`、`addToJsonState(JsonObject&)`、
  `REGISTER_USERMOD()`、`library.json` 格式）已对照 **WLED v16.0.1** 官方源码
  （usermods/multi_relay）核对，但**尚未在真实构建中编译验证**——以 GitHub
  Actions 编译结果为准，若报错按 GITHUB_BUILD_GUIDE.md 常见问题处理。
- 灯珠数量建议 ≤30、亮度上限 ~40%，以保护 5V@3A BEC 余量。
- 若你的飞控固件较老（如 BF 4.2 及更早），MSP_STATUS_EX 尾部字段布局可能不同，
  但 flightModeFlags 固定在偏移 6..9，解析不受影响。
