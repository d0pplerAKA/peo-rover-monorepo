# PEO Rover — ESP32-S3 WiFi 桥固件

> **一句话定位**：S3 是上位机与主控之间的「路由器」——把上位机的 JSON 命令翻译成主控能懂的帧、把主控的传感器数据翻译成 JSON、同时转发相机视频。它刻意「不聪明」：不处理任何业务逻辑，只做透传和几个安全兜底。

---

## 1. 工程总览

S3（ESP32-S3 N8R8，ESP-IDF v6.1 + arduino-esp32 组件）负责整车的无线连接与数据搬运：

- **WiFi STA**：连上位机热点（SSID/密码硬编码在 `esp_net.h`，部署前需填实际值）；
- **TCP 8888**：上位机 JSON 命令 → esp_link 命令帧 → F407（USART2 921600）；反向把 F407 的 116B 传感器帧解析成快照，由 telemetry 组 JSON 下发；
- **TCP 8890**：OV2640 相机 JPEG 视频单向转发给上位机；
- **安全兜底**：STOP 锁 + generation 代次（断电重启后旧命令失效）、心跳超时停车、PID 存 NVS（车上持久化）。

固件 = 7 个组件 + 7 个 FreeRTOS 任务，组件间只用队列通讯。

## 2. 工程意义

1. **F407 没有 WiFi**：主控只有串口，S3 是它与上位机之间的唯一无线通道。
2. **BLE 与 WiFi 不能共存于一颗芯片**（项目早期实测踩坑）：手柄 BLE 桥因此拆到了 C3，S3 专职 WiFi。
3. **安全屏障**：S3 承担「通信层安全」——上位机断线、心跳超时、视频连接异常时自动锁 STOP，并用电代次（generation）让断电重启后的旧命令全部失效。
4. **车机 PID 持久化**：PID 存在 S3 的 NVS（磨损均衡、百万次寿命），掉电不丢；上电/连上位机后自动下发回 F407——车机离线也能用调好的 PID。

## 3. 功能与作用

| 功能 | 实现 |
|---|---|
| 命令透传 | TCP 8888 JSON → cmd_parser 解析校验 → esp_link 帧 → F407 |
| 遥测上报 | F407 50Hz sensor 帧 → telemetry 组 JSON（20Hz，手操态降 2Hz）→ 上位机 |
| 视频转发 | OV2640 JPEG 24fps → TCP 8890（8B 头 + JPEG 体） |
| STOP 锁 + 代次 | 上电即锁 STOP；每次 STOP 代次 +1；恢复需携带新代次的刻意输入 |
| 心跳监督 | 上位机 `{"type":"heartbeat"}` 2s 超时停车（只监督发过控制命令的客户端） |
| deadman 心跳 | S3→F407 200ms 空帧心跳（F407 侧 >500ms 无帧自动停车） |
| PID 持久化 | NVS（namespace=esp_link、key=pid、8B blob），首帧 sensor/客户端连接自动恢复下发 |
| 状态灯 | WS2812（GPIO48）：蓝呼吸=未连热点，绿常亮=已连，绿快闪×3=连接沿 |
| 本地命令 | CAM_RES / VIDEO_ON/OFF / WS2812 灯珠（不转发 F407） |

## 4. 与其它三端的联系

```
        ┌──────────────────────────────────┐
        │       上位机（PC，Python）        │
        └───────────────┬──────────────────┘
              WiFi STA  │ TCP 8888 命令/遥测
                         │ TCP 8890 视频
        ┌───────────────▼──────────────────┐
        │  ESP32-S3（WiFi 桥，本工程） ★    │
        └───────────────┬──────────────────┘
                         │ USART2 921600（esp_link 帧）
        ┌───────────────▼──────────────────┐
        │  STM32F407（主控执行器）           │
        └───────────────┬──────────────────┘
                         │ USART3
        ┌───────────────▼──────────────────┐
        │  ESP32-C3（手柄 BLE 桥）           │
        └──────────────────────────────────┘
```

- **↔ 上位机**：唯一出口。S3 以 STA 连上位机热点，8888 收命令/发遥测、8890 发视频。
- **↔ STM32F407**：USART2 是唯一通道，S3 透传命令帧、接收 sensor 帧，并向 F407 发 200ms 空帧心跳（喂 F407 的 deadman）。
- **↔ ESP32-C3**：无直接电气连接（S3 管 WiFi、C3 管 BLE，物理隔离）。

## 5. 主要实现

### 5.1 任务结构（app_main 装配，7 任务）

| 任务 | 优先级 | 栈 | 核 | 职责 |
|---|---|---|---|---|
| esp_rx | 6 | 4096 | 0 | UART1 初始化 + sensor 帧接收解析（必需） |
| esp_tx | 5 | 3072 | 0 | 命令队列消费 + 组帧发送 + 空帧心跳（必需） |
| esp_net | 5 | 10240 | 0 | WiFi STA + 8888/8890 select 循环（必需） |
| telemetry | 4 | 6144 | 1 | 遥测 JSON 组装 |
| cmd_parser | 4 | 6144 | 1 | JSON 命令解析 |
| camera | 3 | 8192 | 1 | OV2640 抓帧 |
| ws2812 | 2 | 3072 | 1 | 状态灯维护 |

必需任务创建失败 → `esp_link_fatal_fault()` 永久死循环（宁停不跑）。队列：sensor 快照 1 深覆盖、命令 8 深、net rx 32 深、ack 8 深、camera 请求 1 深覆盖。

### 5.2 帧协议（与 F407 同构）

`SOF(0xAA 0x55) + LEN(1B) + PAYLOAD + CRC16(2B)`，CRC16-CCITT（0x1021/0xFFFF）。方向1 = F407 50Hz sensor 帧（PAYLOAD 固定 116B，`static_assert(sizeof==116)`；RX 滑窗状态机 CRC 失败保留后缀重同步）；方向2 = 命令帧（每帧一条，`cmd_id+param≤8B`，发送后 2 tick 停顿触发 F407 IDLE 组帧）。

### 5.3 命令表（cmd_parser）

转发 F407（JSON → esp_link 帧）：

| JSON | 说明 |
|---|---|
| `{"cmd":"LED_TOGGLE"/"LED_ON"/"LED_OFF"}` | 板载 LED |
| `{"cmd":"MOTOR","motor":0-3,"speed":i16}` | 开环 PWM 千分比 |
| `{"cmd":"MOTOR_CLOSED","motor":0-3,"speed_d100":i32}` | 闭环速度（0.01cm/s） |
| `{"cmd":"PID","kp":f32,"ki":f32}` | PID 增益，走 send_pid_and_save（同时写 NVS） |
| `{"cmd":"STOP_ALL"}` | 无条件豁免 gen 校验，直走 request_stop |
| `{"cmd":"ODOMETER_RESET"/"SYS_RESET"}` | 里程清零 / F407 软复位 |

本地命令（不转发）：`CAM_RES {"res":"QQVGA|QVGA|VGA|SVGA"}`、`VIDEO_ON/OFF`、`WS2812 {"r","g","b","brightness"}`。ACK 格式 `{"ack":名,"status":N}`（0 成功/1 参数坏或忙/2 代次过期）。**注意：没有 PING 命令**，心跳是单向 `{"type":"heartbeat"}`。

### 5.4 STOP 锁 + generation（安全核心）

- `esp_link_request_stop()`：`++generation; inhibited=true; stop_pending=true`；STOP 帧优先发（失败 10ms 重试）。
- 恢复：`esp_link_resume_commands(observed_gen)` 要求 `observed==generation && !stop_pending && !fatal`。
- 提交点：TX 锁内校验 `!inhibited && cmd.generation==generation`，至多一条已提交旧帧可先于 STOP 上线。
- 语义 =「安全纪元」：每次 STOP 换代，旧工作/旧客户端命令全部作废，必须携带新代次的**刻意新输入**才能恢复。
- 触发 STOP 的路径：上电、心跳超时、WiFi 断连、客户端断连、视频连接异常、发送硬错误、慢客户端/超长行。

### 5.5 PID 存 NVS

namespace=`esp_link`、key=`pid`、8B blob（kp f32 LE + ki f32 LE）。写时机 = PID 命令成功入队后（`esp_link_send_pid_and_save`）；`s_pid_mutex` 串行化手动保存与恢复读取（防旧值覆盖）。自动恢复触发点 = RX 首帧 sensor / TCP 8888 客户端连接 → TX 任务 resume+下发，失败每 50ms 重试。语义：**一切以车上 PID 为准**。

### 5.6 视频（camera_service + esp_net）

OV2640，模组自带 20MHz XCLK（`CAM_XCLK_HZ=20000000`，ESP32 不输出时钟）；分辨率 QQVGA/QVGA/VGA/SVGA（初始 QVGA）；全分辨率 24fps 绝对节拍；JPEG 质量 QQVGA/QVGA=25、VGA=30、SVGA=35；缓冲 16/32/64/96KB；fb_count=3 + GRAB_LATEST。发送：8B 大端头 `0xCAFE+fid+len` + JPEG 体，EAGAIN→50ms 冷却（在飞帧必须发完），发送硬错误→STOP+关连接。空闲（无客户端/视频 OFF）完全 deinit（归还 PSRAM），恢复前硬复位传感器。

## 6. 功能的影响与制约

- **上电即锁 STOP**：断电重启后旧电机目标全部失效，必须等携带新代次的输入才能运动——安全优先于便利。
- **心跳只监督「发过命令」的客户端**（has_commanded）：只发过 PING 类无害命令的连接超时不触发停车；发过 MOTOR/MOTOR_CLOSED/PID 的客户端 2s 无心跳即锁 STOP 并踢掉。
- **单客户端强制**：`PEO_NET_MAX_CLIENTS=1`，第二个 TCP 连接被拒——上位机端用 ControlLock（8892）配套互斥。
- **视频与命令两条 TCP 独立**：视频连接断开不影响命令通道；但视频发送硬错误会触发 STOP（保守安全）。
- **遥测节拍受锁驱动**：手操态（lock=0）降为 2Hz，自主态 20Hz——手操时把带宽让给视频。
- **PID 权威在车上**：上位机 set_pid 后 S3 立即落 NVS；上电/连上位机都会用车上值覆盖 F407。
- **STA 热点配置是硬编码宏**（`PEO_NET_STA_SSID/PASS`）：换热点必须改固件重烧，没有运行时配置入口。
- **S3 死机发不出 STOP**：因此 F407 侧有独立 deadman（靠 S3 的 200ms 空帧心跳喂狗），车上有物理断电手段是最后一道防线。

## 7. 技术栈与构建

- 芯片：ESP32-S3 N8R8（8MB Quad Flash + 8MB Octal PSRAM）；ESP-IDF v6.1 + arduino-esp32 组件。
- 构建：`./build.ps1`（激活 EIM v6.1 环境 + `idf.py build`）。
- 烧录：`idf.py -p COMx flash monitor`。

## 8. 版权

Copyright (c) 2026 Danny — MIT License（见 `LICENSE`）。

第三方组件保留各自许可——见 `THIRD_PARTY_NOTICES.md`：ESP-IDF
（Apache-2.0）、esp32-camera（Apache-2.0）、cJSON（MIT）、托管组件按
各自许可。

LGPL 说明：本固件静态链接 arduino-esp32（LGPL-2.1）。用本公开源码
重新编译固件即可满足 LGPL 的重新链接要求（见上文构建说明）。
