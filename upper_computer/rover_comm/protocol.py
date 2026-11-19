"""
rover_comm.protocol - PEO Rover three-end protocol core (Python reference
implementation).

Single source of truth: PROTOCOL.md (repo root).
Authoritative C side: peo_rover/COMPONENTS/ESP_LINK/include/esp_link.h +
SENSOR_HUB/sensor_hub.h.

Conventions:
- All multi-byte fields are little-endian (esp_link link); the video header is
  big-endian.
- The trailing CRC16 has its low byte first.
- Team members writing algorithms import directly:
      from rover_comm.protocol import SensorData, encode_cmd, ...

Copyright (c) 2026 Danny
SPDX-License-Identifier: MIT
"""
import struct
from dataclasses import dataclass, field

# =============================================================================
# Constants
# =============================================================================
SOF0, SOF1 = 0xAA, 0x55        # start-of-frame magic bytes (esp_link)
FRAME_OVH = 5                  # SOF(2) + LEN(1) + CRC(2) overhead bytes

# Command IDs (esp_link direction 2; must match esp_link.h)
CMD_LED_TOGGLE    = 0x01
CMD_MOTOR         = 0x03
CMD_PID           = 0x04
CMD_MOTOR_CLOSED  = 0x05
CMD_STOP_ALL      = 0x06
CMD_ODOMETER_RESET = 0x07
CMD_SYS_RESET     = 0x08
CMD_LED_ON        = 0x09
CMD_LED_OFF       = 0x0A

# Command result codes (echoed in SensorData.last_cmd_status)
ST_OK          = 0x00
ST_ERR_INVALID = 0x01
ST_ERR_BADLEN  = 0x02
ST_ERR_BADMOTOR = 0x03

# Network parameters (PROTOCOL.md §5)
TCP_CMD_PORT   = 8888
VIDEO_TCP_PORT = 8890     # video stream uses TCP (decision 2026-11-16; UDP 8889 retired)
VIDEO_MAGIC    = 0xCAFE

# =============================================================================
# Structs (byte-for-byte aligned with the C layout — do not edit the format
# strings; PROTOCOL.md §1.2/§1.3/§3 has the offset tables)
# =============================================================================
SENSOR_DATA = struct.Struct('<HHHBBBxxxIfffffffI8iI4h4HIIBBBBBxxx')   # 116 bytes
VIDEO_REC   = struct.Struct('>HHI')   # 8 bytes, big-endian: magic+fid+len
                                       # (TCP provides integrity; no app-layer CRC)

# =============================================================================
# CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF; matches F407 ESP_LINK_CRC16)
# Self-check: crc16_ccitt(b"123456789") == 0x29B1
# =============================================================================
def crc16_ccitt(data: bytes) -> int:
    """Compute the CRC-16/CCITT-FALSE checksum of `data`.

    Args:
        data: raw bytes to checksum.

    Returns:
        The 16-bit CRC value.
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


# =============================================================================
# esp_link frames (direction 2 framing / direction 1 parsing)
# =============================================================================
def pack_esp_frame(payload: bytes) -> bytes:
    """Frame a payload: SOF 0xAA55 + LEN(1) + PAYLOAD + CRC16 (low byte first).

    Args:
        payload: command or data bytes to send.

    Returns:
        The complete framed bytes ready for the wire.
    """
    frame = bytes((SOF0, SOF1, len(payload))) + payload
    return frame + struct.pack('<H', crc16_ccitt(frame))


def unpack_esp_frame(frame: bytes) -> bytes | None:
    """Unframe: verify SOF + LEN + CRC, return the payload (None on failure).

    Args:
        frame: the received frame bytes.

    Returns:
        The payload bytes if the frame is valid, otherwise None.
    """
    if len(frame) < FRAME_OVH:
        return None
    if frame[0] != SOF0 or frame[1] != SOF1:
        return None
    length = frame[2]
    if len(frame) != FRAME_OVH + length:
        return None
    body = frame[:3 + length]
    crc = frame[3 + length] | (frame[4 + length] << 8)     # low byte first
    if crc16_ccitt(body) != crc:
        return None
    return bytes(body[3:])


# =============================================================================
# Command encoding (direction 2: cmd_id(1) + param)
# =============================================================================
def encode_cmd(cmd_id: int, *params: bytes) -> bytes:
    """Concatenate a command id and its parameter bytes into one payload.

    Args:
        cmd_id: the one-byte command id.
        *params: zero or more pre-packed parameter byte strings.

    Returns:
        The assembled command payload.
    """
    return bytes((cmd_id,)) + b''.join(params)


def cmd_motor(motor: int, speed: int) -> bytes:
    """Open-loop direct drive: motor(1) + speed(int16, per-mille)."""
    return encode_cmd(CMD_MOTOR, bytes((motor,)), struct.pack('<h', speed))


def cmd_pid(kp: float, ki: float) -> bytes:
    """Shared PI gains: kp(4, float) + ki(4, float)."""
    return encode_cmd(CMD_PID, struct.pack('<f', kp), struct.pack('<f', ki))


def cmd_motor_closed(motor: int, speed_d100: int) -> bytes:
    """Closed-loop target speed (0.01 cm/s): motor(1) + speed_d100(int32)."""
    return encode_cmd(CMD_MOTOR_CLOSED, bytes((motor,)), struct.pack('<i', speed_d100))


def cmd_stop_all() -> bytes:
    """Emergency stop / full halt (no parameters)."""
    return encode_cmd(CMD_STOP_ALL)


def cmd_odometer_reset() -> bytes:
    """Reset all four wheel odometers to zero (no parameters)."""
    return encode_cmd(CMD_ODOMETER_RESET)


def cmd_sys_reset() -> bytes:
    """Soft-reset the whole vehicle (no parameters; F407 NVIC_SystemReset)."""
    return encode_cmd(CMD_SYS_RESET)


def cmd_led_toggle() -> bytes:
    """Toggle the onboard LED (no parameters)."""
    return encode_cmd(CMD_LED_TOGGLE)


def cmd_led_on() -> bytes:
    """Turn the onboard LED on (no parameters; absolute state)."""
    return encode_cmd(CMD_LED_ON)


def cmd_led_off() -> bytes:
    """Turn the onboard LED off (no parameters; absolute state)."""
    return encode_cmd(CMD_LED_OFF)


# =============================================================================
# Data structures (parsed from direction-1 uplink frames)
# =============================================================================
@dataclass
class SensorData:
    """F407 uplink snapshot (50 Hz); field names match PROTOCOL.md §1.2."""
    dist_front_mm: int = 0          # front distance in mm
    dist_left_mm: int = 0           # left distance in mm
    dist_right_mm: int = 0          # right distance in mm
    dist_front_status: int = 0      # 0 = valid, non-zero = invalid
    dist_left_status: int = 0
    dist_right_status: int = 0
    dist_timestamp_ms: int = 0      # VL53 distance sample time in ms
    quat_w: float = 0.0             # IMU orientation quaternion (w, x, y, z)
    quat_x: float = 0.0
    quat_y: float = 0.0
    quat_z: float = 0.0
    accel_x: float = 0.0            # acceleration in g (x, y, z)
    accel_y: float = 0.0
    accel_z: float = 0.0
    mpu_timestamp_ms: int = 0       # IMU sample time in ms
    speed_d100: tuple = (0, 0, 0, 0)   # wheel speed in 0.01 cm/s; positive = forward
    odometer: tuple = (0, 0, 0, 0)     # wheel odometer (encoder counts)
    ctrl_timestamp_ms: int = 0         # control sample time in ms
    current_mA: tuple = (0, 0, 0, 0)   # motor current in mA (signed)
    bus_voltage_mV: tuple = (0, 0, 0, 0)  # power bus voltage in mV
    ina_timestamp_ms: int = 0          # power sample time in ms
    sys_tick_ms: int = 0               # F407 system tick in ms (uint32)
    last_cmd_id: int = 0               # id of the last command the F407 executed
    last_cmd_status: int = 0           # result code of that command
    control_mode: int = 0              # 0 = closed loop, 1 = open loop
    fw_version: int = 0                # F407 firmware version (116-byte payload field)
    s3_fw: int = 0                     # S3 firmware version (JSON metadata only)
    lock: int = 0                      # control lock: 0 = manual, 1 = autonomous
    link_stale: bool = False           # S3 JSON metadata; not part of the 116-byte
                                       # UART payload
