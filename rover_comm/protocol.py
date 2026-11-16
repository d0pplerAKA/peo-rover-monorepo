"""
rover_comm.protocol — PEO Rover 三端协议核心（Python 参考实现）

单一事实来源：PROTOCOL.md（本仓库根目录）
C 端权威源：peo_rover/COMPONENTS/ESP_LINK/include/esp_link.h + SENSOR_HUB/sensor_hub.h

约定：
- 所有多字节字段小端（esp_link 链路）；UDP 视频头大端。
- 帧尾 CRC16 低字节在前。
- 组员写算法时直接 `from rover_comm.protocol import SensorData, encode_cmd, ...`。
"""
import struct
from dataclasses import dataclass, field

# =============================================================================
# 常量
# =============================================================================
SOF0, SOF1 = 0xAA, 0x55
FRAME_OVH = 5                      # SOF(2) + LEN(1) + CRC(2)

# 命令 ID（esp_link 方向2，与 esp_link.h 一致）
CMD_LED_TOGGLE    = 0x01
CMD_SENSOR_RESET  = 0x02
CMD_MOTOR         = 0x03
CMD_PID           = 0x04
CMD_MOTOR_CLOSED  = 0x05
CMD_STOP_ALL      = 0x06
CMD_ODOMETER_RESET = 0x07
CMD_SYS_RESET     = 0x08

# 命令结果码（回显在 SensorData.last_cmd_status）
ST_OK          = 0x00
ST_ERR_INVALID = 0x01
ST_ERR_BADLEN  = 0x02
ST_ERR_BADMOTOR = 0x03

# 网络参数（PROTOCOL.md §5）
TCP_CMD_PORT   = 8888
UDP_VIDEO_PORT = 8889
VIDEO_MAGIC    = 0xCAFE
AP_SSID        = "PEO_Rover"
AP_PASSWORD    = "12345678"

# =============================================================================
# 结构体（与 C 布局逐位对齐，勿动格式串；PROTOCOL.md §1.2/§1.3/§3 有偏移表）
# =============================================================================
SENSOR_DATA = struct.Struct('<HHHBBBxxxIfffffffI8iI4h4HIIBB2x')   # 112B
VIDEO_HDR   = struct.Struct('>HHHH')                             # 8B 大端

# =============================================================================
# CRC16-CCITT-FALSE（poly 0x1021，init 0xFFFF，与 F407 ESP_LINK_CRC16 一致）
# 自检：crc16_ccitt(b"123456789") == 0x29B1
# =============================================================================
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


# =============================================================================
# esp_link 帧（方向2 组帧 / 方向1 解析）
# =============================================================================
def pack_esp_frame(payload: bytes) -> bytes:
    """组帧：SOF 0xAA55 + LEN(1) + PAYLOAD + CRC16（低字节在前）。"""
    frame = bytes((SOF0, SOF1, len(payload))) + payload
    return frame + struct.pack('<H', crc16_ccitt(frame))


def unpack_esp_frame(frame: bytes) -> bytes | None:
    """解帧：校验 SOF + LEN + CRC，成功返回 payload，失败返回 None。"""
    if len(frame) < FRAME_OVH:
        return None
    if frame[0] != SOF0 or frame[1] != SOF1:
        return None
    length = frame[2]
    if len(frame) != FRAME_OVH + length:
        return None
    body = frame[:3 + length]
    crc = frame[3 + length] | (frame[4 + length] << 8)     # 低字节在前
    if crc16_ccitt(body) != crc:
        return None
    return bytes(body[3:])


# =============================================================================
# 命令编码（方向2：cmd_id(1) + param）
# =============================================================================
def encode_cmd(cmd_id: int, *params: bytes) -> bytes:
    return bytes((cmd_id,)) + b''.join(params)


def cmd_motor(motor: int, speed: int) -> bytes:
    """开环直驱：motor(1) + speed(int16)。"""
    return encode_cmd(CMD_MOTOR, bytes((motor,)), struct.pack('<h', speed))


def cmd_pid(kp: float, ki: float) -> bytes:
    """共享 PI 增益：kp(4,float) + ki(4,float)。"""
    return encode_cmd(CMD_PID, struct.pack('<f', kp), struct.pack('<f', ki))


def cmd_motor_closed(motor: int, speed_d100: int) -> bytes:
    """闭环目标速度（0.01 cm/s）：motor(1) + speed_d100(int32)。"""
    return encode_cmd(CMD_MOTOR_CLOSED, bytes((motor,)), struct.pack('<i', speed_d100))


def cmd_stop_all() -> bytes:
    """全停急刹（无参）。"""
    return encode_cmd(CMD_STOP_ALL)


def cmd_odometer_reset() -> bytes:
    """清零四轮里程（无参）。"""
    return encode_cmd(CMD_ODOMETER_RESET)


def cmd_sys_reset() -> bytes:
    """全车软复位（无参，F407 NVIC_SystemReset）。"""
    return encode_cmd(CMD_SYS_RESET)


# =============================================================================
# 数据结构（解析自方向1 上报帧）
# =============================================================================
@dataclass
class SensorData:
    """F407 上报快照（50Hz），字段名与 PROTOCOL.md §1.2 偏移表一致。"""
    dist_front_mm: int = 0
    dist_left_mm: int = 0
    dist_right_mm: int = 0
    dist_front_status: int = 0
    dist_left_status: int = 0
    dist_right_status: int = 0
    dist_timestamp_ms: int = 0
    quat_w: float = 0.0
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    mpu_timestamp_ms: int = 0
    speed_d100: tuple = (0, 0, 0, 0)          # 0.01 cm/s，正=前进
    odometer: tuple = (0, 0, 0, 0)
    ctrl_timestamp_ms: int = 0
    current_mA: tuple = (0, 0, 0, 0)          # 有符号
    bus_voltage_mV: tuple = (0, 0, 0, 0)
    ina_timestamp_ms: int = 0
    sys_tick_ms: int = 0
    last_cmd_id: int = 0
    last_cmd_status: int = 0
    link_stale: bool = False       # S3 JSON metadata; not part of the 112-byte UART payload

    @classmethod
    def parse(cls, payload: bytes) -> 'SensorData':
        """从方向1 帧 payload（112B）解析。"""
        v = SENSOR_DATA.unpack(payload)
        return cls(
            dist_front_mm=v[0], dist_left_mm=v[1], dist_right_mm=v[2],
            dist_front_status=v[3], dist_left_status=v[4], dist_right_status=v[5],
            dist_timestamp_ms=v[6],
            quat_w=v[7], quat_x=v[8], quat_y=v[9], quat_z=v[10],
            accel_x=v[11], accel_y=v[12], accel_z=v[13],
            mpu_timestamp_ms=v[14],
            speed_d100=tuple(v[15:19]), odometer=tuple(v[19:23]),
            ctrl_timestamp_ms=v[23],
            current_mA=tuple(v[24:28]), bus_voltage_mV=tuple(v[28:32]),
            ina_timestamp_ms=v[32],
            sys_tick_ms=v[33], last_cmd_id=v[34], last_cmd_status=v[35],
        )


@dataclass
class VideoHeader:
    """UDP 视频帧头（大端，PROTOCOL.md §3）。"""
    magic: int = 0
    frame_id: int = 0
    packet_id: int = 0
    total_pkt: int = 0

    @classmethod
    def parse(cls, data: bytes) -> 'VideoHeader':
        return cls(*VIDEO_HDR.unpack(data))
