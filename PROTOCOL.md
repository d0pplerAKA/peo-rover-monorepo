# PEO Rover 通讯协议定义（PROTOCOL v1.5）

> 本文件是 **F407 ↔ ESP32-S3 ↔ ESP32-C3 ↔ 上位机** 四端开发的单一事实来源（single source of truth）。
> 各端（ESP32-S3 固件 / ESP32-C3 固件 / Python 上位机）实现时以本文件为准；有改动先改这里。
> 来源基准：`peo_rover/COMPONENTS/ESP_LINK/include/esp_link.h`（v2.26）+ `SENSOR_HUB/sensor_hub.h`。

---

## 0. 系统架构

```
 [Xbox 手柄] --BLE--> [ESP32-C3] --UART 115200--> [ESP32-S3] <---WiFi AP---> [上位机 Python (PySide6)]
                      (BLE 桥，50Hz 状态帧 §1.4)     |  ▲
                                                    |  └── UDP 8889：视频流（JPEG 直推，不经 F407）
                                                    |
                                                  USART2 @ 921600（PD5=TX / PD6=RX）
                                                  esp_link 帧协议（见 §1）
                                                    |
                                                  | [F407 主控]（v2.26：传感器 hub / 电机 / 命令队列）
```

> **2026-11-14 架构变更**：BLE 与 WiFi AP 共存于同一颗 ESP32-S3 导致链路不稳定（用户实测结论），
> Xbox 手柄 BLE 拆至独立 ESP32-C3（SuperMini）。C3 只做 BLE 桥（不映射），
> 原始手柄状态经 UART 发 S3，映射逻辑留在 S3 的 c3_link 组件。上位机 JSON 字段不变，无感迁移。

| 链路 | 物理层 | 协议 | 方向 |
|---|---|---|---|
| 手柄 ↔ ESP32-C3 | BLE | Xbox Series X HID | 双向 |
| ESP32-C3 ↔ ESP32-S3 | UART 115200（引脚占位，见 §1.4）| 手柄状态帧（§1.4）| 单向（C3→S3）|
| ESP32-S3 ↔ F407 | USART2 921600 | esp_link 帧（§1）| 双向 |
| ESP32-S3 ↔ 上位机 | WiFi（ESP32 开 AP）| TCP 8888 JSON 命令（§2）+ UDP 8889 视频（§3）| 双向 / 单向 |

### 0.1 组员资源清单（上位机可获得的全部资源）

上位机代码（GUI 插件 / 独立程序）可访问的资源如下，数据通路见对应章节：

| # | 资源 | 实时数据流 | 控制/操作 |
|---|---|---|---|
| 1 | MPU6050（姿态+加速度） | 50Hz JSON `imu.quat[4]` / `imu.accel_g[3]`（§2.1） | 无（只读） |
| 2 | INA3221（四路电流+总线电压） | 50Hz JSON `power.current_mA[4]` / `power.bus_voltage_mV[4]`（§2.1） | 无（只读） |
| 3 | 编码器（速度+里程） | 50Hz JSON `wheels.speed_d100[4]`（0.01 cm/s）/ `wheels.odometer[4]`（§2.1） | **清零里程**：`{"cmd": "ODOMETER_RESET"}`（§1.3 0x07） |
| 4 | 四电机 PWM | `wheels.speed_d100[4]` 回显 | 开环：`MOTOR`；闭环速度：`MOTOR_CLOSED`；急停：`STOP_ALL`（§2.1） |
| 5 | 电机 PI 参数 | 无 | `{"cmd": "PID", "kp": .., "ki": ..}`（四轮共享增益，§1.3 0x04） |
| 6 | 摄像头 | UDP 8889 视频帧 JPEG（§3）→ GUI `on_frame` / `RoverClient.on_frame` | 无（只读；帧率/分辨率由 ESP32 端配置） |
| 7 | Xbox 手柄 | 50Hz JSON `gamepad` 字段（§2.1，摇杆/扳机/按键/电量/连接态） | 无（只读；手柄→命令映射在 ESP32-S3 c3_link 组件） |

Python 侧对应：`SensorData` 字段（§4）、`rover_comm.client.RoverClient`（网络层）、
`algorithms.NavAlgorithmBase`（GUI 插件接口）。

---

## 1. 链路1：esp_link 帧协议（USART2，F407 ↔ ESP32-S3）

### 1.1 帧格式（所有多字节字段小端）

```
┌────────┬─────┬──────────────┬─────────┐
│ SOF    │ LEN │ PAYLOAD      │ CRC16   │
│ 0xAA 0x55 │ 1B  │ LEN 字节      │ 2B（低字节在前）│
└────────┴─────┴──────────────┴─────────┘
```

- **SOF**：`0xAA 0x55`（高字节在前）
- **LEN**：1 字节，payload 长度
- **CRC16-CCITT**：多项式 `0x1021`，初始 `0xFFFF`，覆盖 **SOF ~ PAYLOAD 全部字节**（不含 CRC 本身），帧尾**低字节在前**。

### 1.2 方向1（F407 → ESP32-S3，~50Hz 持续上报）：PAYLOAD = SensorData_t（112B）

F407 端序列化 = **整个 struct 内存拷贝（原生 C 布局，含 padding）**。
帧总长 = 5 + 112 = 117 字节。

**SensorData_t 字节布局（Cortex-M4 ARM EABI，4 字节对齐，逐字节偏移表）：**

| 偏移 | 大小 | 字段 | 类型 | 说明 |
|---:|---:|---|---|---|
| 0 | 2 | dist_front_mm | uint16 LE | 前测距 mm |
| 2 | 2 | dist_left_mm | uint16 LE | 左测距 mm |
| 4 | 2 | dist_right_mm | uint16 LE | 右测距 mm |
| 6 | 1 | dist_front_status | uint8 | 0=有效 |
| 7 | 1 | dist_left_status | uint8 | 0=有效 |
| 8 | 1 | dist_right_status | uint8 | 0=有效 |
| 9 | 3 | **padding** | — | 对齐（跳过）|
| 12 | 4 | dist_timestamp_ms | uint32 LE | SysTick ms |
| 16 | 4 | quat_w | float32 LE | MPU 四元数 |
| 20 | 4 | quat_x | float32 LE | |
| 24 | 4 | quat_y | float32 LE | |
| 28 | 4 | quat_z | float32 LE | |
| 32 | 4 | accel_x | float32 LE | 加速度 g |
| 36 | 4 | accel_y | float32 LE | |
| 40 | 4 | accel_z | float32 LE | |
| 44 | 4 | mpu_timestamp_ms | uint32 LE | |
| 48 | 16 | speed_d100[4] | int32 LE ×4 | 四轮速度 0.01 cm/s，正=前进 |
| 64 | 16 | odometer[4] | int32 LE ×4 | 四轮里程计数 |
| 80 | 4 | ctrl_timestamp_ms | uint32 LE | |
| 84 | 8 | current_mA[4] | int16 LE ×4 | 四轮电流 mA（有符号）|
| 92 | 8 | bus_voltage_mV[4] | uint16 LE ×4 | 四路总线电压 mV |
| 100 | 4 | ina_timestamp_ms | uint32 LE | |
| 104 | 4 | sys_tick_ms | uint32 LE | 整帧快照时间戳 |
| 108 | 1 | last_cmd_id | uint8 | 最近命令 ID（回显）|
| 109 | 1 | last_cmd_status | uint8 | 0=OK，非0=错误码 |
| 110 | 2 | **padding** | — | 尾部对齐（跳过）|

**总计 112 字节。**

### 1.3 方向2（ESP32-S3 → F407）：PAYLOAD = 命令帧（cmd_id + param，每帧一条）

**打包约定（ESP32-S3 发送方必须遵守）：**
1. 每帧**恰好一条命令**：`cmd_id(1B) + param(变长)`；param 长度由 cmd_id 约定（见命令集表）。
2. **手柄数据不再下发 F407**：Xbox 手柄由 ESP32-C3 接收（BLE HID 报告 → §1.4 状态帧），
   ESP32-S3 c3_link 组件映射成 MOTOR / MOTOR_CLOSED / STOP_ALL 等命令帧后下发；F407 只负责执行命令。
3. 全帧 payload 后算 CRC（覆盖 SOF~PAYLOAD），低字节在前。
4. 发完一帧后**停顿 ≥1 字节时间**，触发 F407 的 USART2 IDLE 中断组帧。

**命令集（cmd_id + param）：**

| cmd_id | 名称 | param 布局 | 说明 |
|---:|---|---|---|
| 0x01 | LED_TOGGLE | 无 | |
| 0x02 | SENSOR_RESET | 无 | |
| 0x03 | MOTOR | motor(1B) + speed(2B, int16 LE) | 开环直驱 |
| 0x04 | PID | kp(4B, float32 LE) + ki(4B, float32 LE) | 共享增益 |
| 0x05 | MOTOR_CLOSED | motor(1B) + speed_d100(4B, int32 LE) | 闭环目标速度 |
| 0x06 | STOP_ALL | 无 | 全停急刹（宏中止/断连保护/安全刹车）|
| 0x07 | ODOMETER_RESET | 无 | 清零四轮里程 |

**命令结果码（F407 回显在 SensorData_t.last_cmd_status）：**

| 码 | 含义 |
|---:|---|
| 0x00 | 成功 |
| 0x01 | 未知命令 ID |
| 0x02 | 参数长度错误 |
| 0x03 | 电机号越界 |

### 1.4 链路0：ESP32-C3 ↔ ESP32-S3 手柄状态链路（50Hz，单向 C3→S3）

- **动机**：BLE 与 WiFi AP 共存于同一颗 ESP32-S3 导致链路不稳定（用户实测结论），
  手柄 BLE 拆至独立 ESP32-C3（SuperMini，4MB flash）。**C3 只做 BLE 桥，不做映射**；
  映射逻辑留在 ESP32-S3 c3_link 组件（原 xbox_map 安全逻辑原样复用）。
- **物理层**：UART 115200 8-N-1，无流控。
  **引脚占位（待用户确认接线后改两端 PEO_PINMAP.h）**：
  C3 GPIO4(TX)/GPIO5(RX) ↔ S3 GPIO18(RX)/GPIO17(TX)（S3 用 UART_NUM_2）。
  （C3 GPIO20/21 已被板载 USB 转串口占用，不可作链路引脚。）
- **帧格式**：与 §1.1 esp_link 同构——`SOF(0xAA 0x55) + LEN(1) + PAYLOAD + CRC16-CCITT(2)`，
  小端，CRC 覆盖 SOF~PAYLOAD，帧尾低字节在前。
- **PAYLOAD = type(1B) + body**。当前唯一类型：

```
type = 0x10  GAMEPAD_STATE（50Hz 周期，无论手柄是否连接都发，兼作 C3 存活心跳）
```

| 偏移 | 大小 | 字段 | 类型 | 说明 |
|---:|---:|---|---|---|
| 0 | 1 | type | uint8 | 固定 0x10 |
| 1 | 4 | seq | uint32 LE | 有效 HID 通知序号（C3 端 notificationSequence 透传，跨重连递增）|
| 5 | 4 | disc_gen | uint32 LE | 断连代次（C3 端 disconnectGeneration 透传）|
| 9 | 1 | connected | uint8 | 0=未连接 1=已连接且有数据 |
| 10 | 1 | battery | uint8 | 电量 0~100 |
| 11 | 1 | input_fresh | uint8 | 0/1：**C3 本地判定**最近 5s 内有有效 HID 通知（C3 millis − receivedNotificationAt ≤ 5s；未连接恒 0）|
| 12 | 2 | joy_lx | uint16 LE | 左摇杆水平（0~65535，中值 32768）|
| 14 | 2 | joy_ly | uint16 LE | 左摇杆垂直 |
| 16 | 2 | joy_rx | uint16 LE | 右摇杆水平 |
| 18 | 2 | joy_ry | uint16 LE | 右摇杆垂直 |
| 20 | 2 | trigger_lt | uint16 LE | 左扳机 0~1023 |
| 22 | 2 | trigger_rt | uint16 LE | 右扳机 0~1023 |
| 24 | 2 | buttons | uint16 LE | 按键位图（位序见下）|

body = 25B，payload = 26B，帧总长 = 31B。

**buttons 位图位序**（与 S3 c3_link 组件 XB_BTN_* 宏一致）：
`bit0=A 1=B 2=X 3=Y 4=LB 5=RB 6=MENU(start) 7=VIEW(select) 8=SHARE 9=LS 10=RS
11=DPAD_UP 12=DPAD_DOWN 13=DPAD_LEFT 14=DPAD_RIGHT 15=XBOX`

**可靠性 / 安全语义：**
- S3 c3_link 逐字节状态机解析（与 §1.2 RX 同模式：SOF 同步、LEN 边界、CRC 校验、丢字节保留后缀重同步）。
- **C3 失联判定**：S3 端连续 **500ms** 未收到任何状态帧 → 视为 C3 掉线 →
  `esp_link_request_stop()`（锁 STOP）+ 手柄快照置 `connected=0`；收到帧自动解除失联锁存。
- **手柄断连与输入新鲜度均在帧内直接体现**（用户 2026-11-14 定）：断连 = `connected=0`；
  输入停摆 = `input_fresh=0`（C3 本地按 5s 判定，S3 直接消费，不再由 seq 时间戳推断）。
  S3 映射任务对二者统一走 STOP 锁存 + 新鲜回中解锁（原 xbox_map 逻辑原样复用）。
  seq 仍随帧传输，供「STOP 后出现新通知」的判断（post_stop 条件）；
  seq 仅在有效 HID 通知时递增（电池/短报告不递增）。

---

## 2. 链路2a：TCP JSON 命令通道（上位机 ↔ ESP32-S3，端口 8888）

- 上位机连 ESP32 的 AP 后，连 TCP `8888`。
- 每行一个 JSON 对象（`\n` 结尾），ESP32 解析后转成 esp_link 二进制命令发 F407。
- 命令执行结果由 ESP32 回 JSON ACK；F407 的最终执行状态再随 §1.2 上报的 `last_cmd_status` 回显。

### 2.1 命令、ACK 与传感器流

**上位机 → ESP32（命令）：**

```json
{"cmd": "LED_TOGGLE"}
{"cmd": "SENSOR_RESET"}
{"cmd": "MOTOR", "motor": 0, "speed": 500}
{"cmd": "PID", "kp": 0.25, "ki": 0.005}
{"cmd": "MOTOR_CLOSED", "motor": 1, "speed_d100": 4000}
{"cmd": "STOP_ALL"}
{"cmd": "ODOMETER_RESET"}
{"cmd": "CAM_RES", "res": "QVGA"}
{"cmd": "VIDEO_ON"}
{"cmd": "VIDEO_OFF"}
```

> `CAM_RES` 是 ESP32-S3 **本地**命令（不转发 F407）：`res` ∈ `QQVGA`(160x120) / `QVGA`(320x240) / `VGA`(640x480) / `SVGA`(800x600)，相机任务热切换（deinit + 重新 init，约半秒黑屏，期间不推流）。分辨率越大链路带宽需求越高（VGA ≈ QVGA 的 4 倍数据量），链路不稳时优先 QVGA/QQVGA；切换失败自动回退原分辨率。帧率统一固定 24fps（用户 2026-11-15 定案：可变帧率功能已全部取消，CAM_FPS 命令已删除）。

**视频按需传输：**

- `VIDEO_ON` / `VIDEO_OFF` 与 `CAM_RES` 一样是 S3 本地命令，不转发 F407，分别开始 / 停止发送 UDP `8889` 视频流。
- S3 上电 / 复位后默认 video OFF；任一 TCP 客户端发送命令均全局生效。
- 成功 ACK 使用命令原名：`{"ack":"VIDEO_ON","status":0}` / `{"ack":"VIDEO_OFF","status":0}`。
- 所有 TCP 客户端断开后自动回到 OFF，下次连接需重新发送 `VIDEO_ON`。
- `gen` 校验与 `CAM_RES` 相同：带 `gen` 时校验当前代次，不带时走兼容路径；非法 / 过期值按下文 ACK 规则处理。
- 相机任务继续抓帧；OFF 时归还帧缓冲，不做 SOI / EOI / SOS 校验、不上发、不计入 ok / bad，计入 cam stats 新增的 `skipped`。`real_fps` 按 `(ok + bad + skipped) / 统计时长` 计算实际抓帧率。

**命令可选字段 `gen`（会话代次，v1.5 起，问题3 修复）：**

- STOP 锁存时 ESP32 端递增内部代次（generation），并随 50Hz 传感器 JSON 流广播 `"gen"` 字段。
- 上位机应记录最近一次收到的 `gen`，并在发运动命令（MOTOR / MOTOR_CLOSED / PID / ODOMETER_RESET / macro 等）时附上 `"gen": <最近值>`。
- ESP32 校验：带 `gen` 的消息，`gen` 必须等于当前代次，否则**拒绝**（ACK `status=2`，不执行、不恢复运动）。目的：淘汰 STOP 前已进入 TCP 缓冲、STOP 后才被读取的旧命令（它们携带旧代次）——防止旧命令解除 STOP。
- 不带 `gen` 的消息维持兼容行为（按 ESP32 读取时刻的代次处理）；**新版上位机必须携带 `gen`**。
- `STOP_ALL` 命令**无条件接受**（不校验 gen，任何状态下必须可达）；`PING` 同样豁免并无条件回包。
- 心跳（§2.4）不携带 gen。

**ESP32 → 上位机（ACK）：**

```json
{"ack": "MOTOR", "status": 0}
```

**ACK status 码：**

| status | 含义 |
|---:|---|
| 0 | 成功 |
| 1 | 未知命令 / 参数缺失 / 发送失败 |
| 2 | stale generation：`gen` 与当前代次不符，命令被拒（不执行、不恢复运动）|

**ESP32 → 上位机（传感器上报，50Hz，由 §1.2 帧转 JSON）：**

```json
{"type": "sensor", "tick": 12345, "gen": 3,
 "dist": {"front": {"mm": 123, "ok": true}, "left": {"mm": 45, "ok": true}, "right": {"mm": 67, "ok": false}},
 "imu": {"quat": [0.99, 0.0, 0.0, 0.1], "accel_g": [0.0, 0.0, 1.0]},
 "wheels": {"speed_d100": [0, 0, 0, 0], "odometer": [0, 0, 0, 0]},
 "power": {"current_mA": [100, 110, 95, 105], "bus_voltage_mV": [11800, 11800, 11800, 11800]},
 "cmd_ack": {"id": 3, "status": 0},
 "sys": {"cpu0": 12, "cpu1": 34, "ram_free": 123456, "ram_total": 200000},
 "gamepad": {"connected": true, "battery": 92,
             "joy_l": [32768, 32768], "joy_r": [32768, 32768],
             "triggers": [0, 0], "buttons": ["A"]}}
```

**系统性能统计 `sys`（可选对象，S3 本地采集，不改变 SensorData）：**

| 键 | 类型 | 说明 |
|---|---|---|
| `cpu0` / `cpu1` | int | 双核最近约 1 秒窗口利用率，0~100%；首轮无历史为 0，窗口不足 1 秒沿用上轮值 |
| `ram_free` | int | 内部 SRAM 堆当前空闲字节，`heap_caps_get_free_size(MALLOC_CAP_INTERNAL)` |
| `ram_total` | int | 内部 SRAM 堆总量字节，`heap_caps_get_total_size(MALLOC_CAP_INTERNAL)` |

启用 FreeRTOS run time stats，以 ESP Timer 1 MHz 时钟统计，每核 `CPU% = 100 - idle_delta * 100 / window_us`，限制到 0~100。配置开启会触发全量重编并略增开销。GUI 每秒恒显传感器 Hz、视频 fps、CPU0/CPU1、RAM 已用百分比及空闲 KB、PING RTT；无数据为 `-`（频率为 0），不再显示坏帧 / 丢帧。

**诊断命令 `PING`：**

- 请求 `{"cmd":"PING","t":1794760000000}`，`t` 为整数毫秒时间戳；缺失或非 number 时视为 0。
- S3 本地命令，不转发 F407，无控制副作用。无条件回包 `{"ack":"PING","status":0,"t":1794760000000}`，`t` 数值通过 double 原值返回。
- 任何时候可发，不受 STOP、消息本地代次或 `gen`（含非法 / 过期值）限制；ACK 沿用 TCP 广播队列。
- 上位机每秒发送一次，以回包时刻的本机毫秒时间戳减去发送时间戳测 RTT。

**gamepad 字段（可选，ESP32-S3 c3_link 收到 C3 手柄状态帧（§1.4）后附带，50Hz 随 sensor 流推送，只读展示用）：**

| 键 | 类型 | 说明 |
|---|---|---|
| `connected` | bool | BLE 连接状态 |
| `battery` | int | 电量 0~100（%），无数据时 0 |
| `joy_l` / `joy_r` | int[2] | 左右摇杆 (x, y)，0~65535，中值 32768 |
| `triggers` | int[2] | [LT, RT]，0~1023 |
| `buttons` | str[] | **按下的**按钮名数组（松开的不出现） |

**按钮名集合**（与旧 GAMEPAD_BTN_* 位图对应，v2.26 起只在上位机↔ESP32 这条链路上用）：

`A` `B` `X` `Y` `LB` `RB` `MENU` `VIEW` `SHARE` `LS` `RS` `DPAD_UP` `DPAD_DOWN` `DPAD_LEFT` `DPAD_RIGHT`

**ESP32 端序列化对照**（asukiaaa xboxNotif 字段 → JSON）：

| JSON | asukiaaa 字段 |
|---|---|
| `joy_l[0]` / `joy_l[1]` | `joyLHori` / `joyLVert` |
| `joy_r[0]` / `joy_r[1]` | `joyRHori` / `joyRVert` |
| `triggers[0]` / `triggers[1]` | `triggerLT` / `triggerRT` |
| `buttons[]` | `btnA` `btnB` `btnX` `btnY` `btnLB` `btnRB` `btnShare` `btnLS` `btnRS` `btnSelect` `btnStart` `btnDPadUp/Down/Left/Right` |
| `battery` | `battery` |

### 2.2 宏脚本（macro，ESP32 执行器解析后逐条拆发 F407）

**下发（一条 JSON 消息）：**

```json
{"type": "macro", "id": 1, "steps": [
  {"cmd": "MOTOR_CLOSED", "motor": 0, "speed_d100": 4000, "duration_ms": 0},
  {"wait": {"field": "dist_front_mm", "op": "<", "value": 200, "timeout_ms": 5000}},
  {"cmd": "MOTOR_CLOSED", "motor": 0, "speed_d100": 0, "duration_ms": 500},
  {"goto": 0}
]}
```

**步骤三种类型（单宏最多 64 步——`steps` 数组条目上限，ESP32 端资源护栏；更长的任务拆多个宏串联，等 `state:"done"` 后再发下一个）：**

| 类型 | 格式 | 语义 |
|---|---|---|
| 动作 | `{"cmd":"...","motor":N,"...":"...","duration_ms":T}` | 编码成 esp_link 命令发 F407；`duration_ms=0` = 命令持续生效立即下一步（配合条件步骤），`>0` = 发命令后等 T 毫秒再下一步 |
| 条件等待 | `{"wait":{"field":F,"op":O,"value":V,"timeout_ms":T}}` | 轮询最近 SensorData 的字段 F 与 V 比较（O ∈ `< > <= >= == !=`），满足即下一步；超时 T 也下一步 |
| 跳转 | `{"goto": N}` | 跳到 0-based 步骤索引 N（可形成循环）；越界 = 宏结束 |

**field 可选值**（取自 §1.2 SensorData 上报字段）：
`dist_front_mm` / `dist_left_mm` / `dist_right_mm` / `dist_*_status` / `quat_w` / `quat_x` / `quat_y` / `quat_z` / `accel_x` / `accel_y` / `accel_z` / `speed_d100_0..3` / `odometer_0..3` / `current_mA_0..3` / `bus_voltage_mV_0..3` / `sys_tick_ms`

**进度上报**（附加在 §2.1 传感器 JSON 流里）：
```json
"macro": {"id": 1, "step": 2, "total": 5, "state": "running"}
```
state ∈ `running / done / aborted`。

**中止**：`{"type": "macro_abort", "id": 1}`

**断连保护（硬规则）**：执行宏的 TCP 连接断开 → ESP32 立即发 STOP_ALL 并中止宏。

### 2.3 多客户端订阅

- TCP 8888 支持 **≤4 个客户端同时连接**，每个连接都收到 50Hz 传感器 JSON 流（§2.1）+ 宏进度。
- 命令（含宏）任何客户端可发；宏同一时刻只允许一个（新宏覆盖旧宏并中止旧的）。
- 组员可直接连 ESP32（任何语言）收数据/发命令，无需依赖上位机项目。

### 2.4 闭环算法（上位机实时控制）与离线宏的分工

视频流的处理依赖上位机 Python 库（cv2/OpenCV 等），**ESP32-S3 不做图像处理**
（无 OpenCV，算力/内存受限）。因此依赖视频的导航算法是**上位机闭环**，不是 ESP32 端宏。

两条执行路径：

| | A. 离线宏（§2.2） | B. 闭环算法（本节） |
|---|---|---|
| 执行位置 | ESP32 宏引擎自持 | 上位机算法进程 |
| 适用 | 不依赖视频的预设动作/条件机动（超声波避障、定点停车） | 视觉巡线、目标检测追踪、复杂导航 |
| 输入 | ESP32 可取的传感器字段（§2.2 field 列表） | 50Hz 传感器 JSON + UDP 视频帧（cv2 处理） |
| 命令节奏 | 一次下发，ESP32 逐步执行 | 实时逐条下发（典型 10~20Hz） |
| 断连行为 | ESP32 自动 STOP_ALL | 上位机停止发命令 |

**B 路径数据流：**

```text
ESP32 推流(UDP 8889) → 上位机 cv2 处理 → 决策 → 标准命令(JSON, TCP 8888)
                                                    │
      ┌─────────────────────────────────────────────┘
      ▼
ESP32 解析 JSON（透传，<1ms）→ esp_link 帧 → USART2 → STM32 while loop 执行
```

- 闭环算法下发的命令就是 §2.1 的标准命令（与宏 act 同格式），**ESP32 不需理解算法逻辑**，
  只是"路由器"：JSON → esp_link 帧 → UART。
- 上位机每发一条，STM32 执行一条（MOTOR_CLOSED 是设定值命令，无队列堆积）。

**心跳保护（上位机保活，安全硬规则）：**

```json
{"type": "heartbeat"}
```

- 上位机连接后**每 500ms 发一条**（无参数，只作保活信号）。
- ESP32 端记录最近一次心跳时间；**连续 2s 未收到心跳（且该连接发过命令）→ 自动发 STOP_ALL**。
- 覆盖场景：上位机进程崩溃/卡死/网络瞬断导致视觉闭环失去控制源。
- 与 §2.2 的宏断连保护独立并存（宏保护绑执行宏的连接，心跳保护绑控制源连接）。
- 心跳只用于检测"上位机还活着"；ESP32 无需回包。

**事件 → 短宏（两种模式的组合）：**
视觉算法检测到事件（如"前方发现障碍"）时，可不逐条直控，而是构建 3~5 步短宏
一次下发（如 后退→转向→前进），让 ESP32 自持执行，兼顾视觉决策与断连安全。

**延迟预算（局域网，视觉闭环典型值）：**

| 环节 | 延迟 |
|---|---|
| 视频帧到齐（30fps，UDP 分片） | ~33ms（帧龄主导） |
| cv2 处理（巡线级） | <10ms |
| TCP JSON 上行 | 1~5ms |
| ESP32 解析 + UART 921600 10B 帧 | ~1ms |
| STM32 执行 | 下一个 while loop 迭代 |
| **合计** | **~50ms → 视觉闭环 10~20Hz 控制率可行** |

帧龄是主导项：后续按导航需要权衡分辨率/帧率（如 320×240@30fps 换成 240×180@15fps，
帧龄可压到 ~20ms 内，视算法需求而定）。

---

## 3. 链路2b：UDP 视频流（ESP32-S3 → 上位机，端口 8889）

自定应用层协议（借 RTP 思路简化），每包：

```
┌────────┬──────────┬──────────┬──────────┬─────────────┐
│ magic  │ frame_id │ packet_id│ total_pkt│ JPEG payload│
│ 2B     │ 2B       │ 2B       │ 2B       │ ≤1400B      │
└────────┴──────────┴──────────┴──────────┴─────────────┘
```

- 全部 **大端**；magic = `0xCAFE`。
- `frame_id` 帧序号递增；`packet_id` 帧内包序号（0 起）；`total_pkt` 本帧总包数。
- 接收端按 `frame_id` 分组、`packet_id` 排序拼帧；**丢包/乱序 = 整帧丢弃跳下一帧**（视频不重传）。
- 单包 JPEG payload ≤1400B（避开 MTU 大包丢包）。
- 上位机接收端**不设包大小上限**（recvfrom 64KB），若未来 ESP32 端调整分片大小，
  上位机无需同步修改。

---

## 4. Python 参考实现（上位机用，与 C 端逐位对齐）

```python
import struct

# SensorData_t（112B，含 padding，与 §1.2 偏移表逐位对应）
SENSOR_DATA = struct.Struct('<HHHBBBxxxIfffffffI8iI4h4HIIBB2x')

# UDP 视频帧头（8B，大端）
VIDEO_HDR = struct.Struct('>HHHH')

def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT：多项式 0x1021，初始 0xFFFF（与 ESP_LINK_CRC16 一致）。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def pack_esp_frame(payload: bytes) -> bytes:
    """方向2 组帧：SOF 0xAA55 + LEN + PAYLOAD + CRC（低字节在前）。"""
    frame = b'\xAA\x55' + bytes([len(payload)]) + payload
    crc = crc16_ccitt(frame)
    return frame + struct.pack('<H', crc)
```

---

## 5. 网络参数

| 项 | 值 |
|---|---|
| ESP32 AP SSID | `PEO_Rover` |
| ESP32 AP 密码 | `12345678`（≥8 位，WPA2）|
| ESP32 AP IP | `192.168.4.1`（默认）|
| TCP 命令端口 | `8888` |
| UDP 视频端口 | `8889` |
| 传感器上报频率 | 50Hz（与 F407 §1.2 同频）|
| 视频分辨率（初始）| QVGA(320×240) → 稳定后 VGA(640×480) |
