"""
rover_comm — PEO Rover 通讯核心库（组员可直接 import）

三端架构（PROTOCOL.md）：
    F407 <--USART2 esp_link--> ESP32-S3 <--WiFi(TCP JSON/UDP 视频)--> 上位机

子模块：
    protocol : 帧/结构体/CRC/命令编码（纯数据层，无网络）
    client   : TCP 命令 + 传感器流 + UDP 视频接收（网络层）
"""
from .protocol import (
    # 常量
    CMD_LED_TOGGLE, CMD_SENSOR_RESET, CMD_MOTOR, CMD_PID,
    CMD_MOTOR_CLOSED, CMD_STOP_ALL, CMD_ODOMETER_RESET, CMD_SYS_RESET,
    ST_OK, ST_ERR_INVALID, ST_ERR_BADLEN, ST_ERR_BADMOTOR,
    TCP_CMD_PORT, VIDEO_TCP_PORT,
    # 函数
    crc16_ccitt, pack_esp_frame, unpack_esp_frame,
    encode_cmd, cmd_motor, cmd_pid, cmd_motor_closed, cmd_stop_all,
    cmd_odometer_reset, cmd_sys_reset,
    # 结构
    SensorData,
    SENSOR_DATA, VIDEO_REC,
)
from .client import RoverClient

__all__ = [
    "CMD_LED_TOGGLE", "CMD_SENSOR_RESET", "CMD_MOTOR", "CMD_PID",
    "CMD_MOTOR_CLOSED", "CMD_STOP_ALL", "CMD_ODOMETER_RESET", "CMD_SYS_RESET",
    "ST_OK", "ST_ERR_INVALID", "ST_ERR_BADLEN", "ST_ERR_BADMOTOR",
    "TCP_CMD_PORT", "VIDEO_TCP_PORT",
    "crc16_ccitt", "pack_esp_frame", "unpack_esp_frame",
    "encode_cmd", "cmd_motor", "cmd_pid", "cmd_motor_closed", "cmd_stop_all",
    "cmd_odometer_reset", "cmd_sys_reset",
    "SensorData",
    "SENSOR_DATA", "VIDEO_REC",
    "RoverClient",
]
