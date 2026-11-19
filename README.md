# PEO Rover — ESP32-S3 WiFi 桥 / WiFi Bridge

> **一句话定位 / One-line role**：S3 是上位机和主控之间的「路由器」——把上位机的 JSON 命令翻译成主控能懂的帧、把主控的传感器数据翻译成 JSON，同时转发相机视频。它不处理业务逻辑，只做透传和安全兜底。
>
> The S3 is the "router" between the upper computer and the main controller — it translates upper-computer JSON commands into frames the main controller understands, translates the main controller's sensor data into JSON, and relays camera video. It runs no business logic, only pass-through plus safety guardrails.

---

## 工程意义 / Why This Project Exists

**中文**：上位机（PC）和主控（F407）之间需要一条无线链路，但 F407 没有 WiFi 能力。S3 就是这条无线链路的「中间人」：连上位机热点（WiFi STA），把上位机发的 JSON 命令（TCP 8888）翻译成 esp_link 帧透传给 F407（USART2），再把 F407 的传感器数据翻译成 JSON 上报，同时把 OV2640 相机视频转发给上位机（TCP 8890）。它刻意「不聪明」——不处理任何业务逻辑，只做透传和几个安全兜底（STOP 锁、generation 代次、心跳停车、PID 存 NVS）。

**English**: The upper computer (PC) and the main controller (F407) need a wireless link, but the F407 has no WiFi. The S3 is that link's "middleman": it joins the upper computer's hotspot (WiFi STA), translates the upper computer's JSON commands (TCP 8888) into esp_link frames forwarded to the F407 (USART2), translates the F407's sensor data back into JSON, and relays the OV2640 camera video to the upper computer (TCP 8890). It is deliberately "dumb" — it runs no business logic, only pass-through plus a few safety guardrails (STOP lock, generation counter, heartbeat stop, PID stored in NVS).

## 功能与作用 / What It Does

| 功能 Function | 说明 Description |
|---|---|
| 命令透传 | JSON（TCP 8888）→ esp_link 帧 → F407 |
| 遥测上报 | F407 sensor 帧 → JSON（50 Hz）→ 上位机 |
| 视频转发 | OV2640 JPEG（TCP 8890）→ 上位机 |
| 安全兜底 | STOP 锁 + generation 代次 + 心跳停车 |
| PID 持久化 | PID 存 NVS，上电/连上位机后自动下发 F407 |
| 状态灯 | WS2812 蓝/绿指示连接状态 |

## 与其它三端的联系 / Connections to the Other Three Nodes

```
        ┌──────────────────────────────────────┐
        │      上位机 Upper Computer (PC)       │
        └──────────────────┬───────────────────┘
                WiFi STA   │ TCP 8888 命令/遥测
                            │ TCP 8890 视频
        ┌──────────────────▼───────────────────┐
        │     ESP32-S3（WiFi 桥 Bridge）     ★  │
        └──────────────────┬───────────────────┘
                            │ USART2 (921600)
        ┌──────────────────▼───────────────────┐
        │  STM32F407（主控 Main Controller）    │
        └──────────────────┬───────────────────┘
                            │ USART3
        ┌──────────────────▼───────────────────┐
        │   ESP32-C3（手柄 BLE 桥 Gamepad）     │
        └──────────────────────────────────────┘
```

- **↔ 上位机**：WiFi STA 连上位机热点；TCP 8888 收命令/发遥测，TCP 8890 发视频。
- **↔ STM32F407**：USART2 是唯一通道，S3 透传命令帧、接收 sensor 帧。
- **↔ ESP32-C3**：无直接电气连接（S3 管 WiFi，C3 管 BLE，物理隔离）。

---

## 技术细节 / Technical Details

本文对应 2026-11-16 的当前源码。项目使用 ESP-IDF v6.1、Arduino-ESP32 4.0.0-rc1 组件和本地修改的 `components/esp32-camera`。保留 `dependencies.lock`，避免依赖版本漂移。

S3 负责 **TCP JSON 命令转发到 STM32F407、传感器遥测、OV5640 UDP 视频和 WS2812**。C3 手柄桥、S3 BLE/HID 和本地宏引擎已移除；自动化逻辑由上位机脚本运行。

## 硬件与配置

- ESP32-S3 N8R8：8 MiB Quad Flash（DIO、80 MHz）与 8 MiB Octal PSRAM（80 MHz）。不要启用 Octal Flash。
- PSRAM 开机初始化和内存测试开启；配置允许未检测到 PSRAM 时继续启动，相机此时不可用。
- UART1：TX GPIO11 → F407 PD6；RX GPIO12 ← F407 PD5；921600 baud、8-N-1、共地。
- 引脚统一位于 `components/peo_pinmap/PEO_PINMAP.h`；GPIO35–37 留给 Octal PSRAM。
- OV5640 模组自带 XCLK，`pin_xclk=GPIO_NUM_NC`。`CAM_XCLK_HZ=24000000` 用于传感器时序计算，必须与实物晶振相符。
- 单颗 WS2812 使用 GPIO48，RMT 按 GRB 字节顺序、字节内 MSB 优先发送。

## 任务结构

`app_main` 先创建队列并锁存启动 STOP，再创建任务；Arduino 自动 loopTask 未启用。

| 任务 | 核 / 优先级 | 职责 |
| --- | --- | --- |
| `esp_rx` | 0 / 6 | 初始化 UART，解析 CRC 校验后的 112 B 传感器快照 |
| `esp_tx` | 0 / 5 | 优先发送锁存 STOP，消费带本地代次的命令队列 |
| `esp_net` | 0 / 5 | AP、TCP 单客户端、心跳、ACK/遥测、UDP 视频及还帧 |
| `telemetry` | 1 / 4 | 在线时每轮延时 40 ms，读取最新快照并生成 JSON |
| `cmd_parser` | 1 / 4 | JSON 参数与代次检查，派发本地或 F407 命令 |
| `ws2812` | 1 / 2 | 消费最新颜色请求并发送 RMT 波形 |
| `camera` | 1 / 3 | 相机初始化、分辨率切换、抓帧和视频入队 |

前三个任务创建失败会永久禁止恢复普通 UART 命令。其余任务按当前代码作为可选任务，创建失败记录日志并继续启动。相机驱动另有自己的采集任务，配置为 core 1、8192 B 栈。

## 网络与命令

- AP：`PEO_Rover`，密码 `12345678`，默认地址 `192.168.4.1`；启动时扫描并在信道 1–11 中选择干扰评分最低者。
- TCP：8888，仅允许一个客户端。每条 JSON 以换行结束；行长上限 16384 B，已入队完整行总预算 65536 B。
- UDP：8889，目标地址来自 TCP 客户端 IP，目标端口固定为 8889，无独立 UDP 握手。
- ACK 和遥测共用 TCP 字节流，ACK 队列优先处理。上位机必须按行解析，并区分 `ack` 与 `type:"sensor"`。

| 命令 | 参数 / 行为 |
| --- | --- |
| `LED_TOGGLE`、`SENSOR_RESET`、`STOP_ALL`、`ODOMETER_RESET` | 无参数；转发到 F407，其中 STOP 使用独立锁存通路 |
| `SYS_RESET` | 向 F407 发送复位命令，不重启 S3 |
| `MOTOR` | `motor` 为整数 0–3，`speed` 为 int16 范围整数 |
| `MOTOR_CLOSED` | `motor` 为整数 0–3，`speed_d100` 为 int32 范围整数 |
| `PID` | `kp`、`ki` 必须是可表示为有限 float 的数值 |
| `PING` | 回显数值 `t`，缺失/非数值时使用 0；更新客户端活动，不解除 STOP |
| `CAM_RES` | `res` 为 `QQVGA`、`QVGA`、`VGA` 或 `SVGA`；请求队列保留最新值 |
| `VIDEO_ON` / `VIDEO_OFF` | 开关视频服务；开机和最后一个客户端断开后默认关闭 |
| `WS2812` | `r/g/b` 必填，均为整数 0–255；`brightness` 缺失时默认 100，提供时必须为整数 0–100 |

以下每行末尾都需发送换行：

```json
{"cmd":"PING","t":12345}
{"cmd":"WS2812","r":128,"g":0,"b":0,"brightness":50}
{"cmd":"CAM_RES","res":"QQVGA"}
{"cmd":"VIDEO_ON"}
{"cmd":"STOP_ALL"}
```

普通响应为 `{"ack":"命令名","status":0}`。状态 0 表示桥接层接受请求，**不表示 F407 已执行，也不表示相机已完成切换或灯珠已更新**；状态 1 表示参数错误、未知命令或当前无法接受；显式 `gen` 过期时状态为 2。非法 JSON 或没有可识别命令的对象可能被静默忽略。

可选 `gen` 必须是 uint32 范围整数，值取自最新遥测。普通 F407 命令只有代次仍有效且 STOP 已发送完成后才可恢复提交；无 `gen` 的旧本地代次消息可能被静默丢弃。`STOP_ALL` 不受 `gen` 格式/数值限制，`PING` 也在代次校验之前处理。本地命令使用入口代次检查，但不会解除 UART STOP。

## 心跳、遥测与停车边界

成功提交 `MOTOR`、`MOTOR_CLOSED` 或 `PID` 后，该连接开始受 2000 ms 心跳超时监督；上述成功命令或有效 `PING` 可续期，其他命令不续期。超时会锁存 STOP 并断开连接。TCP 断开、接收超限或发送积压/停滞错误也会请求 STOP。

STOP 不占用普通命令队列容量，并使旧代次工作失效。已通过 TX 提交点的一帧仍可能先于 STOP 发出；UART 完成不代表 F407 已执行。这不是硬件急停。S3 掉电、崩溃或 UART 断线时无法保证送达 STOP，F407 必须独立实现过期命令停车，车辆还需物理断电手段。

遥测顶层字段为 `type`、`tick`、`gen`、`dist`、`imu`、`wheels`、`power`、`cmd_ack`、`sys`；不再包含 gamepad/macro。无有效快照或接收年龄超过 500 ms 时增加 `link_stale:true`，仍可上报 S3 的 CPU/RAM 信息。`link_stale` 是状态提示，当前 S3 不会仅因该标记自动停车。

## 视频行为

- 默认 QVGA，目标节拍 24 fps，实际帧率受传感器、初始化和网络影响。
- QQVGA/QVGA 的 JPEG quality 为 25，VGA 为 40，SVGA 为 50（数值越大压缩越强）。
- 三个 PSRAM 帧缓冲，申请容量按 `max(width*height/4, 65536)` 计算，驱动再增加 DMA/缓存对齐余量。
- 启动先尝试初始化诊断，随后反初始化待命；客户端在线且视频开启时恢复初始化。进入空闲时归还帧并反初始化。每次成功恢复/切换后预热 48 次抓帧，不发送这些帧。
- 分辨率切换通过 flush、deinit、硬复位和重新 init 完成；失败会有限重试并尝试回退。`CAM_RES` 的 ACK 只表示请求成功入队。
- 活动状态下的非空帧不再按 5000 B 下限筛掉。驱动执行 SOI 检查、缓存同步和 EOI 截尾，上位机仍需完整解码并丢弃损坏帧。
- 视频队列深度为 1，覆盖时归还旧帧。网络发送失败后冷却 200 ms；未成功入队的帧由 camera 归还，成功入队后由 esp_net 归还。
- UDP 每包最多 1400 B JPEG 数据，前置 8 B 大端头：`0xCAFE`、16 位帧号、从 0 开始的包号、总包数。丢包帧由上位机丢弃。

## 构建与本地测试

本机 PowerShell：

```powershell
Set-Location 'C:/Users/Danny/Documents/Espressif Files/peo_rover_esp32s3'
./build.ps1
python tests/review_regressions_host.py
python tests/camera_dma_alignment_host.py
& 'C:/Espressif/tools/python/v6.1/venv/Scripts/python.exe' tests/verify_firmware.py
```

`build.ps1` 激活 EIM v6.1 环境并清除可能干扰构建的 MSYS 变量。检查构建退出码、最终成功日志，以及 ELF/BIN 是否更新。

- `review_regressions_host.py`：用实际解析器、灯珠初始化/缩放函数及视频交接代码，加主机适配器，检查参数边界、位序和小帧所有权。需要 C++ 编译器（`CXX` 或 PATH 上的 g++）与工程托管的 cJSON。GCC 使用 C++17，MSVC 使用 C++20 支持 RMT 配置的指定初始化。
- `camera_dma_alignment_host.py`：编译真实分配/缓存函数，检查 18 组对齐组合、同步方向/错误返回以及旧布局拒绝。需要 C11 编译器（`CC` 或 PATH 上的 gcc）。
- 两个测试支持 MSVC：在 x64 开发人员命令行中设置 `CC=cl`、`CXX=cl` 后运行。GCC 可分别使用 `CFLAGS`、`CXXFLAGS` 添加参数。
- `verify_firmware.py`：检查 sdkconfig、已生成配置并打印 ELF SHA256；它不执行构建，也不验证 ELF 与最新源码是否一致。
- 主机测试不模拟 ESP32 DMA、无线电、相机、RMT 实际波形或 F407 电机执行。当前仓库没有旧文档中的 `run_link_tests.py` 和 `net_run_host_tests.py`，不能据此声称 UART/网络已获完整回归覆盖。

最新修复和验证结果见 [REPAIR_STATUS.md](REPAIR_STATUS.md)。

## 烧录与硬件验收

先断开电机动力或架空车轮，关闭占用串口的其他监视器。按实际串口替换 COM11：

```powershell
& 'C:/Espressif/tools/Microsoft.v6.1.PowerShell_profile.ps1'
idf.py -p COM11 flash monitor
```

使用完整 bootloader、分区表及应用偏移烧录；本次修复无需改 eFuse 或全片擦除。监视器用 Ctrl+] 退出。

验收包括 PSRAM 自检、启动 STOP、TCP 断连/心跳停车与重连恢复、四种分辨率及视频开关、QQVGA/纯色/暗场小 JPEG、灯珠 1/2/128 和不同亮度，以及同时运行遥测和视频。独立核实 F407 看门狗、轮序/方向及模组晶振。

## 历史记录

`BOOT_LOOP_FIX_REPORT.md`、`codex_review_report.md`、`codex_review_report_round2.md` 保留当时的证据和结论，已标明历史状态。它们涉及已删除组件或旧测试，不能当作现版本功能说明或最新验证结果。当前说明以本 README、源码及 REPAIR_STATUS 为准。
