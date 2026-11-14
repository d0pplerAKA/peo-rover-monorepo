# PEO Rover 通讯协议定义（PROTOCOL v1.0）

> 本文件是 **F407 ↔ ESP32-S3 ↔ 上位机** 三端开发的单一事实来源（single source of truth）。
> 两端（ESP32-S3 固件 / Python 上位机）实现时以本文件为准；有改动先改这里。
> 来源基准：`peo_rover/COMPONENTS/ESP_LINK/include/esp_link.h`（v2.25）+ `SENSOR_HUB/sensor_hub.h`。

---

## 0. 系统架构

```
 [Xbox 手柄] --BLE--> [ESP32-S3] <---WiFi AP---> [上位机 Python (PySide6)]
                        |  ▲
                        |  └── UDP 8889：视频流（JPEG 直推，不经 F407）
                        |
                      USART2 @ 4.2Mbps（PD5=TX / PD6=RX）
                      esp_link 帧协议（见 §1）
                        |
                      [F407 主控]（v2.25：传感器 hub / 电机 / 命令队列）
```

| 链路 | 物理层 | 协议 | 方向 |
|---|---|---|---|
| 手柄 ↔ ESP32-S3 | BLE | Xbox Series X HID | 双向 |
| ESP32-S3 ↔ F407 | USART2 4.2Mbps | esp_link 帧（§1）| 双向 |
| ESP32-S3 ↔ 上位机 | WiFi（ESP32 开 AP）| TCP 8888 JSON 命令（§2）+ UDP 8889 视频（§3）| 双向 / 单向 |

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

### 1.3 方向2（ESP32-S3 → F407）：PAYLOAD = GamepadData_t(20B) + 命令(可选变长)

**打包约定（ESP32-S3 发送方必须遵守）：**
1. 每帧固定带 20B 手柄数据；手柄未连接也整包发（`connected=0`，其余 0/默认）。
2. 手柄后追加 0~多个命令：`cmd_id(1B) + param(变长)`。
3. 全帧 payload 后算 CRC（覆盖 SOF~PAYLOAD），低字节在前。
4. 发完一帧后**停顿 ≥1 字节时间**，触发 F407 的 USART2 IDLE 中断组帧。

**GamepadData_t 字节布局（20B，无 padding）：**

| 偏移 | 大小 | 字段 | 类型 |
|---:|---:|---|---|
| 0 | 2 | joy_lx | uint16 LE（0~65535）|
| 2 | 2 | joy_ly | uint16 LE |
| 4 | 2 | joy_rx | uint16 LE |
| 6 | 2 | joy_ry | uint16 LE |
| 8 | 2 | trigger_lt | uint16 LE |
| 10 | 2 | trigger_rt | uint16 LE |
| 12 | 2 | buttons | uint16 LE 位图（见下）|
| 14 | 1 | connected | 0=未连接 1=已连接 |
| 15 | 1 | battery | 电量 0~100 % |
| 16 | 1 | rumble_left | 振动回显 |
| 17 | 1 | rumble_right | 振动回显 |
| 18 | 1 | rumble_lt | 振动回显 |
| 19 | 1 | rumble_rt | 振动回显 |

**按键位图（buttons 字段 bit 位）：**

| bit | 按键 | bit | 按键 |
|---|---|---|---|
| 0 | A | 8 | Share |
| 1 | B | 9 | LS（左摇杆按下）|
| 2 | X | 10 | RS（右摇杆按下）|
| 3 | Y | 11 | D-Pad Up |
| 4 | LB | 12 | D-Pad Down |
| 5 | RB | 13 | D-Pad Left |
| 6 | Start | 14 | D-Pad Right |
| 7 | Select | | |

**命令集（cmd_id + param）：**

| cmd_id | 名称 | param 布局 | 说明 |
|---:|---|---|---|
| 0x01 | LED_TOGGLE | 无 | |
| 0x02 | SENSOR_RESET | 无 | |
| 0x03 | MOTOR | motor(1B) + speed(2B, int16 LE) | 开环直驱 |
| 0x04 | PID | kp(4B, float32 LE) + ki(4B, float32 LE) | 共享增益 |
| 0x05 | MOTOR_CLOSED | motor(1B) + speed_d100(4B, int32 LE) | 闭环目标速度 |

**命令结果码（F407 回显在 SensorData_t.last_cmd_status）：**

| 码 | 含义 |
|---:|---|
| 0x00 | 成功 |
| 0x01 | 未知命令 ID |
| 0x02 | 参数长度错误 |
| 0x03 | 电机号越界 |

---

## 2. 链路2a：TCP JSON 命令通道（上位机 ↔ ESP32-S3，端口 8888）

- 上位机连 ESP32 的 AP 后，连 TCP `8888`。
- 每行一个 JSON 对象（`\n` 结尾），ESP32 解析后转成 esp_link 二进制命令发 F407。
- 命令执行结果由 ESP32 回 JSON ACK；F407 的最终执行状态再随 §1.2 上报的 `last_cmd_status` 回显。

**上位机 → ESP32（命令）：**

```json
{"cmd": "LED_TOGGLE"}
{"cmd": "SENSOR_RESET"}
{"cmd": "MOTOR", "motor": 0, "speed": 500}
{"cmd": "PID", "kp": 0.25, "ki": 0.005}
{"cmd": "MOTOR_CLOSED", "motor": 1, "speed_d100": 4000}
```

**ESP32 → 上位机（ACK）：**

```json
{"ack": "MOTOR", "status": 0}
```

**ESP32 → 上位机（传感器上报，50Hz，由 §1.2 帧转 JSON）：**

```json
{"type": "sensor", "tick": 12345,
 "dist": {"front": {"mm": 123, "ok": true}, "left": {"mm": 45, "ok": true}, "right": {"mm": 67, "ok": false}},
 "imu": {"quat": [0.99, 0.0, 0.0, 0.1], "accel_g": [0.0, 0.0, 1.0]},
 "wheels": {"speed_d100": [0, 0, 0, 0], "odometer": [0, 0, 0, 0]},
 "power": {"current_mA": [100, 110, 95, 105], "bus_voltage_mV": [11800, 11800, 11800, 11800]},
 "cmd_ack": {"id": 3, "status": 0}}
```

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

---

## 4. Python 参考实现（上位机用，与 C 端逐位对齐）

```python
import struct

# SensorData_t（112B，含 padding，与 §1.2 偏移表逐位对应）
SENSOR_DATA = struct.Struct('<HHHBBBxxxIfffffffI8iI4h4HIIBB2x')

# GamepadData_t（20B）
GAMEPAD = struct.Struct('<HHHHHHHBBBBBB')

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
