# PEO Rover 通讯协议定义（PROTOCOL v2.0）

> 实现对齐修订：2026-11-16。TCP/UDP 部分已与当前 S3 源码和上位机实现核对。
> UART 布局保留原约定并与本地 S3 头文件核对；本轮未运行 F407 固件或硬件验收。
> 来源基准：`peo_rover/COMPONENTS/ESP_LINK/include/esp_link.h`（v2.26）+ `SENSOR_HUB/sensor_hub.h`。
> **v2.0（2026-11-15）**：C3 手柄桥、宏引擎、GUI 算法插件全部废弃（§2.2 标注）；
> 组员自动化唯一路径 = §2.4 组员编程模型（上位机原生 Python 脚本）。

---

## 0. 系统架构

```
                                                   WiFi AP
 [上位机 Python (PySide6 GUI / peo SDK 脚本)] <──────────────> [ESP32-S3]
         ▲                                                      │  ▲
         └────────── UDP 8889：视频流（JPEG 直推）──────────────┘  │
                                                                  │
                                                          USART2 @ 921600（PD5=TX / PD6=RX）
                                                          esp_link 帧协议（见 §1）
                                                                  │
                                                              [F407 主控]（传感器 hub / 电机 / 命令队列）
```

> **2026-11-15 系统收敛**（用户拍板）：① Xbox 手柄 BLE 桥（ESP32-C3 + c3_link）废弃；
> ② ESP32 宏引擎（macro_engine）废弃；③ GUI 算法插件（algorithms/）废弃。
> 组员自动化 = §2.4 组员编程模型（上位机原生 Python 脚本，ESP32 负责桥接与停车屏障）。

| 链路 | 物理层 | 协议 | 方向 |
|---|---|---|---|
| ESP32-S3 ↔ F407 | USART2 921600 | esp_link 帧（§1）| 双向 |
| ESP32-S3 ↔ 上位机 | WiFi（ESP32 开 AP）| TCP 8888 JSON 命令（§2）+ UDP 8889 视频（§3）| 双向 / 单向 |

### 0.1 组员资源清单（上位机可获得的全部资源）

上位机代码（`peo` SDK 脚本 / GUI）可访问的资源如下，数据通路见对应章节：

| # | 资源 | 实时数据流 | 控制/操作 |
|---|---|---|---|
| 1 | MPU6050（姿态+加速度） | 约 25Hz JSON `imu.quat[4]` / `imu.accel_g[3]`（§2.1）| 无（只读）|
| 2 | INA3221（四路电流+总线电压） | 约 25Hz JSON `power.current_mA[4]` / `power.bus_voltage_mV[4]`（§2.1）| 无（只读）|
| 3 | 编码器（速度+里程） | 约 25Hz JSON `wheels.speed_d100[4]`（0.01 cm/s）/ `wheels.odometer[4]`（§2.1）| **清零里程**：`{"cmd": "ODOMETER_RESET"}`（§1.3 0x07）|
| 4 | 四电机 PWM | `wheels.speed_d100[4]` 回显 | 开环：`MOTOR`；闭环速度：`MOTOR_CLOSED`；急停：`STOP_ALL`（§2.1）|
| 5 | 电机 PI 参数 | 无 | `{"cmd": "PID", "kp": .., "ki": ..}`（四轮共享增益，§1.3 0x04）|
| 6 | 摄像头 | UDP 8889 视频帧 JPEG（§3）→ `peo.cam.read()` / `RoverClient.on_frame` | 无（只读；分辨率由上位机 `CAM_RES` 热切换）|

Python 侧对应：`SensorData` 字段（§4）、`rover_comm.client.RoverClient`（网络层）、
`peo` SDK（§2.4 组员编程模型，组员唯一接口）。

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
- **CRC16-CCITT-FALSE**：多项式 `0x1021`，初始 `0xFFFF`，覆盖 **SOF ~ PAYLOAD 全部字节**（不含 CRC 本身），帧尾**低字节在前**。

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
2. 全帧 payload 后算 CRC（覆盖 SOF~PAYLOAD），低字节在前。
3. 发完一帧后**停顿 ≥1 字节时间**，触发 F407 的 USART2 IDLE 中断组帧。

**命令集（cmd_id + param）：**

| cmd_id | 名称 | param 布局 | 说明 |
|---:|---|---|---|
| 0x01 | LED_TOGGLE | 无 | |
| 0x02 | SENSOR_RESET | 无 | |
| 0x03 | MOTOR | motor(1B) + speed(2B, int16 LE) | 开环直驱 |
| 0x04 | PID | kp(4B, float32 LE) + ki(4B, float32 LE) | 共享增益 |
| 0x05 | MOTOR_CLOSED | motor(1B) + speed_d100(4B, int32 LE) | 闭环目标速度 |
| 0x06 | STOP_ALL | 无 | 全停急刹（断连保护/安全刹车）|
| 0x07 | ODOMETER_RESET | 无 | 清零四轮里程 |
| 0x08 | SYS_RESET | 无 | F407 软复位（S3 不重启）|

**命令结果码（F407 回显在 SensorData_t.last_cmd_status）：**

| 码 | 含义 |
|---:|---|
| 0x00 | 成功 |
| 0x01 | 未知命令 ID |
| 0x02 | 参数长度错误 |
| 0x03 | 电机号越界 |

---

## 2. TCP JSON（端口 8888）

每行一个 JSON 对象，以换行结束。S3 仅接受一个客户端；视频、遥测和命令共享该控制会话。最大命令行 16384 B（不含换行），S3 完整行队列总字节预算 65536 B。Python 接收端也对单行设置 16384 B 上限。

### 2.1 命令、ACK、gen 和遥测

| cmd | 参数 |
| --- | --- |
| `LED_TOGGLE` / `SENSOR_RESET` / `STOP_ALL` / `ODOMETER_RESET` / `SYS_RESET` | 无参数；SYS_RESET 只请求 F407 复位 |
| `MOTOR` | `motor` 整数 0–3，`speed` 为 int16 范围整数 |
| `MOTOR_CLOSED` | `motor` 整数 0–3，`speed_d100` 为 int32 范围整数（0.01 cm/s） |
| `PID` | 有限 float32 可表示的 `kp`、`ki` |
| `CAM_RES` | `res`: QQVGA / QVGA / VGA / SVGA |
| `VIDEO_ON` / `VIDEO_OFF` | S3 本地视频开关 |
| `WS2812` | `r/g/b` 必填整数 0–255；`brightness` 可省略，默认 100，提供时须为整数 0–100 |
| `PING` | 数值 `t` 原样回显；省略/非 number 使用 0 |

```json
{"cmd":"MOTOR_CLOSED","motor":1,"speed_d100":4000,"gen":3}
{"cmd":"WS2812","r":128,"g":0,"b":0,"brightness":50,"gen":3}
{"cmd":"STOP_ALL"}
{"cmd":"PING","t":123456}
```

普通 ACK：`{"ack":"命令名","status":0}`。0 表示桥接层接受，1 表示参数错误/未知命令/当前无法接受，2 表示显式 gen 过期。**ACK 不是 F407 执行确认，也不是相机/灯珠硬件完成确认。** 未识别对象、非法 JSON 和某些无 gen 的旧消息可能静默丢弃。

S3 每次锁存 STOP 递增 uint32 `gen` 并在遥测发送。带 gen 时须为当前 uint32 整数；无 gen 保留固件兼容路径。当前 Python 客户端除 STOP/PING 外，必须获得可用遥测代次后才允许普通命令入队，并自动附加 gen。STOP 清空本机尚未发送命令，待不同 gen 的遥测到达后再允许新工作。调用方仍可显式提供 gen，过期值由 S3 拒绝。

STOP/PING 不受 gen 限制。SDK 四轮目标作为一个本机发送批次（最多四条 JSON）排队，F407 仍逐条执行。STOP 前已提交的一批可能先发出，不能保证物理瞬时停车。普通动作返回只表示本机入队成功；`peo.stop()` 单独等待新收到的成功 STOP ACK，300 ms 超时后重发一次，仍未确认抛异常。协议没有请求 ID，无法提供严格逐请求 ACK 关联。

S3 当前每轮延时 40 ms 读取快照并发 JSON，名义约 **25 Hz**，不是原文的 50 Hz。UART 传感器源更新频率与 TCP 发送频率不同。

```json
{"type":"sensor","tick":12345,"gen":3,
 "dist":{"front":{"mm":123,"ok":true},"left":{"mm":45,"ok":true},"right":{"mm":67,"ok":false}},
 "imu":{"quat":[1,0,0,0],"accel_g":[0,0,1]},
 "wheels":{"speed_d100":[0,0,0,0],"odometer":[0,0,0,0]},
 "power":{"current_mA":[100,110,-95,105],"bus_voltage_mV":[11800,11800,11800,11800]},
 "cmd_ack":{"id":3,"status":0},
 "sys":{"cpu0":12,"cpu1":34,"ram_free":123456,"ram_total":200000}}
```

无 UART 快照或快照接收年龄超过 500 ms 时，S3 添加 `link_stale:true`，仍可发送自身资源数据。上位机将此标记保存在 `SensorData.link_stale`（不改变 112 B UART 布局），SDK fresh 还要求已连接且最近 250 ms 收到遥测。缺失测距字段按无效处理。历史 sensor `cmd_ack` 不能确认当前 STOP。

`sys.cpu0/cpu1` 为 S3 每核利用率，`ram_free/ram_total` 为内部堆字节。PING 用本机单调毫秒时间戳，只有匹配当前待确认 t 的 ACK 才更新 RTT。

### 2.2 已移除功能

C3 手柄桥、S3 本地宏引擎和上位机宏面板均已移除。`{"type":"macro",...}` 不再受理，不提供宏步骤/变量/跳转协议。

### 2.3 连接、资源与视频状态

- AP `PEO_Rover` / `12345678`，默认 IP `192.168.4.1`；S3 `PEO_NET_MAX_CLIENTS=1`。
- Monitor 和 SDK 共用本机 loopback TCP 8892 控制租约；这不是跨设备的锁。
- TCP 断开后 S3 请求 STOP、关闭视频并清理队列；相机退出活动循环后 flush/deinit。
- VIDEO_ON 后相机重新初始化；成功恢复或切换后的前 48 次抓帧用于预热。首帧时延取决于初始化/采集，不保证 0.5 s。
- VIDEO_OFF 时相机退出抓帧循环并反初始化，不再持续采集旧文档所谓 skipped 流。
- 分辨率请求 latest-wins；切换失败有限重试并尝试回退。ACK 只代表请求入队。

### 2.4 Python 编程与心跳

脚本运行在 PC，通过 SDK 读属性、发动作、等待；S3/F407 不执行用户 Python。

**实际心跳请求：**

```json
{"cmd":"PING","t":123456}
```

每 500 ms 一次，S3 回复 `{"ack":"PING","status":0,"t":123456}`。旧 `{"type":"heartbeat"}` 不是当前固件支持的命令。

S3 在连接成功提交 MOTOR/MOTOR_CLOSED/PID 后监督活动；这些成功命令或 PING 续期，超过 2000 ms 则锁存 STOP 并断开。其他本地命令不续期。后台 PING 不能证明算法本身在推进，循环需自行检查传感器和图像新鲜度。断线/软件崩溃时硬件侧仍需独立看门狗。

SDK `drive` 不再按未确认的旧目标去重，也不声称实现固定 5 ms 节流。调用方应显式 `delay` 控制频率。GUI Stop 立即请求优先 STOP，随后由 SDK 检查协作取消；不能强杀不合作的 Python 线程。

## 3. UDP 视频（端口 8889）

每包为 `magic(2) + frame_id(2) + packet_id(2) + total_pkt(2) + JPEG 分片`，头部全部大端，magic 为 `0xCAFE`。数据分片最多 1400 B；packet_id 从 0 开始，frame_id 为 16 位并可回绕，S3 全断连清理时会复位帧号。

接收端只接受 TCP 对端 IP、UDP 源端口 8889。最多保留一帧、512 KiB / 375 个分片。包 0 建立帧上下文，之后允许本帧包乱序；包 0 之前的片不缓存。相同帧的 total 必须一致，冲突重复包会放弃该帧。迟到旧帧首包不会顶掉较新帧；2 s 间隔后允许重新建立序号上下文。重连创建新组装器。

帧完整后检查 SOI、SOF/SOS 顺序、段长、扫描边界及末尾 EOI，再交解码器。JPEG 结构守卫不验证熵数据完整性。SDK 只返回最近 1 s 内的完整帧；UDP 不重传。接收上限是实现约束，未来改变固件分片大小时必须同步接收端。

## 4. Python 二进制参考

```python
SENSOR_DATA = struct.Struct('<HHHBBBxxxIfffffffI8iI4h4HIIBB2x')  # 112 B
VIDEO_HDR = struct.Struct('>HHHH')                             # 8 B
```

实现位于 `rover_comm/protocol.py`。CRC16-CCITT-FALSE 多项式 0x1021、初值 0xFFFF，无反射，测试向量 `123456789` 得 0x29B1；不是 KERMIT。

## 5. 当前参数及验证边界

| 项目 | 值 |
| --- | --- |
| TCP / UDP | 8888 / 8889 |
| 本机控制锁 | 127.0.0.1:8892 |
| TCP 遥测 | 约 25 Hz |
| S3 视频目标节拍 | 24 fps，实际受硬件/网络影响 |
| 默认分辨率 | QVGA 320×240 |
| 支持分辨率 | QQVGA 160×120、QVGA 320×240、VGA 640×480、SVGA 800×600 |
| JSON 行上限 | 16384 B |
| 普通本机发送队列 | 64 个批次，每批最多四条 JSON |

本轮离线测试不提供端到端时延预算或硬件执行保证。最新测试与修复记录见 [REVIEW_AND_REPAIR.md](REVIEW_AND_REPAIR.md)。
