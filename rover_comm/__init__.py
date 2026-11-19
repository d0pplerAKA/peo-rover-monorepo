"""
rover_comm - PEO Rover communication core library (team members can import it
directly).

Three-end architecture (PROTOCOL.md):
    F407 <--USART2 esp_link--> ESP32-S3 <--WiFi (TCP JSON / TCP video)--> PC

Submodules:
    protocol : frames / structs / CRC / command encoding (pure data layer, no
        networking)
    client   : TCP command + sensor stream + TCP video receive (network layer)
    control  : local control-lease lock shared by the GUI and the SDK

Copyright (c) 2026 Danny
SPDX-License-Identifier: MIT
"""
from .protocol import (
    # constants
    CMD_LED_TOGGLE, CMD_MOTOR, CMD_PID,
    CMD_MOTOR_CLOSED, CMD_STOP_ALL, CMD_ODOMETER_RESET, CMD_SYS_RESET,
    CMD_LED_ON, CMD_LED_OFF,
    ST_OK, ST_ERR_INVALID, ST_ERR_BADLEN, ST_ERR_BADMOTOR,
    TCP_CMD_PORT, VIDEO_TCP_PORT,
    # functions
    crc16_ccitt, pack_esp_frame, unpack_esp_frame,
    encode_cmd, cmd_motor, cmd_pid, cmd_motor_closed, cmd_stop_all,
    cmd_odometer_reset, cmd_sys_reset,
    cmd_led_toggle, cmd_led_on, cmd_led_off,
    # structs
    SensorData,
    SENSOR_DATA, VIDEO_REC,
)
from .client import RoverClient

__all__ = [
    "CMD_LED_TOGGLE", "CMD_MOTOR", "CMD_PID",
    "CMD_MOTOR_CLOSED", "CMD_STOP_ALL", "CMD_ODOMETER_RESET", "CMD_SYS_RESET",
    "CMD_LED_ON", "CMD_LED_OFF",
    "ST_OK", "ST_ERR_INVALID", "ST_ERR_BADLEN", "ST_ERR_BADMOTOR",
    "TCP_CMD_PORT", "VIDEO_TCP_PORT",
    "crc16_ccitt", "pack_esp_frame", "unpack_esp_frame",
    "encode_cmd", "cmd_motor", "cmd_pid", "cmd_motor_closed", "cmd_stop_all",
    "cmd_odometer_reset", "cmd_sys_reset",
    "cmd_led_toggle", "cmd_led_on", "cmd_led_off",
    "SensorData",
    "SENSOR_DATA", "VIDEO_REC",
    "RoverClient",
]
