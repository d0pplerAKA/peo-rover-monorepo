"""
peo - PEO Rover team SDK (the single entry point for team members;
PROTOCOL.md §2.4 team programming model).

Three primitives: read data attributes + call action functions + delay for
pacing. Decisions and loops are the team member's own native Python code; the
SDK provides no logic wrappers.

Usage:
    import peo
    peo.connect("peo_rover.local")

    peo.drive_all(30)
    while peo.fresh and peo.vl53.front_ok and peo.vl53.front_mm > 200:
        peo.delay(20)
    peo.stop()

    peo.cam.on()
    img, seq = peo.cam.read()
    peo.cam.off()

Copyright (c) 2026 Danny
SPDX-License-Identifier: MIT
"""
import time as _time
import math as _math

from ._bridge import _bridge, ScriptStopped, NotConnected, LinkLost, CamOff, \
    _stop_requested
from ._i18n import tr
from .cam import cam

# Sensor data namespaces (read-only; refreshed in the background at ~25 Hz).
# When no data is available / the link is down, values are None — use
# peo.fresh to tell whether the link is alive.

class _MPU:
    """Read-only IMU namespace (orientation quaternion + acceleration)."""

    @property
    def quat(self):
        """Orientation quaternion (w, x, y, z), or None without data."""
        s = _bridge._snapshot()
        return None if s is None else (s.quat_w, s.quat_x, s.quat_y, s.quat_z)

    @property
    def accel(self):
        """Acceleration in g (x, y, z), or None without data."""
        s = _bridge._snapshot()
        return None if s is None else (s.accel_x, s.accel_y, s.accel_z)


class _VL53:
    """Read-only namespace for the three VL53 distance sensors."""

    @property
    def front_mm(self):
        """Front distance in mm, or None without data."""
        s = _bridge._snapshot()
        return None if s is None else s.dist_front_mm

    @property
    def left_mm(self):
        """Left distance in mm, or None without data."""
        s = _bridge._snapshot()
        return None if s is None else s.dist_left_mm

    @property
    def right_mm(self):
        """Right distance in mm, or None without data."""
        s = _bridge._snapshot()
        return None if s is None else s.dist_right_mm

    @property
    def front_ok(self):
        """True when the front sensor reports a valid reading, or None."""
        s = _bridge._snapshot()
        return None if s is None else (s.dist_front_status == 0)

    @property
    def left_ok(self):
        """True when the left sensor reports a valid reading, or None."""
        s = _bridge._snapshot()
        return None if s is None else (s.dist_left_status == 0)

    @property
    def right_ok(self):
        """True when the right sensor reports a valid reading, or None."""
        s = _bridge._snapshot()
        return None if s is None else (s.dist_right_status == 0)


class _Wheels:
    """Read-only namespace for wheel speed, odometer, and motor current."""

    @property
    def speed_cm_s(self):
        """Four wheel speeds in cm/s (protocol 0.01 cm/s fixed point -> float)."""
        s = _bridge._snapshot()
        return None if s is None else [v / 100.0 for v in s.speed_d100]

    @property
    def odometer(self):
        """Four wheel odometer values, or None without data."""
        s = _bridge._snapshot()
        return None if s is None else list(s.odometer)

    @property
    def current_mA(self):
        """Four motor currents in mA (signed), or None without data."""
        s = _bridge._snapshot()
        return None if s is None else list(s.current_mA)

    @property
    def bus_voltage_mV(self):
        """Power bus voltages in mV, or None without data."""
        s = _bridge._snapshot()
        return None if s is None else list(s.bus_voltage_mV)


class _Sys:
    """Read-only namespace for ESP32 system stats (CPU % and RAM %)."""

    def __init__(self):
        self._cpu0 = None
        self._cpu1 = None
        self._ram = None

    def _update(self, info):
        """Refresh cached stats from the latest sys_info dict (or None)."""
        self._cpu0 = info.get("cpu0") if info else None
        self._cpu1 = info.get("cpu1") if info else None
        free = info.get("ram_free") if info else None
        total = info.get("ram_total") if info else None
        self._ram = None if (free is None or not total) else \
            round((total - free) * 100 / total)

    @property
    def cpu0(self):
        """Core 0 utilization percent (pulled on each read), or None."""
        self._update(_bridge.client.sys_info)
        return self._cpu0

    @property
    def cpu1(self):
        """Core 1 utilization percent (pulled on each read), or None."""
        self._update(_bridge.client.sys_info)
        return self._cpu1

    @property
    def ram(self):
        """RAM usage percent (pulled on each read), or None."""
        self._update(_bridge.client.sys_info)
        return self._ram


mpu = _MPU()
vl53 = _VL53()
wheels = _Wheels()
sys = _Sys()


# ---------------- Lifecycle ----------------

def connect(host="peo_rover.local") -> None:
    """Connect to the ESP32 (TCP 8888 command/telemetry + TCP 8890 video +
    heartbeat + single-instance lease).

    The cached PID and resolution are sent automatically after connecting and
    re-sent after an F407 soft reset.
    """
    _bridge.connect(host)


def disconnect() -> None:
    """Close the connection and release the single-instance lease."""
    _bridge.disconnect()


def delay(ms: float) -> None:
    """Block for ms milliseconds (Event.wait; precision depends on the OS
    scheduler). If the IDE stop button fires during the wait, raises
    ScriptStopped to end the script thread."""
    if _stop_requested.is_set():
        raise ScriptStopped(tr("script stopped", "脚本已停止"))
    if not _math.isfinite(ms) or ms < 0:
        raise ValueError("delay must be finite and non-negative")
    _stop_requested.wait(ms / 1000.0)
    if _stop_requested.is_set():
        raise ScriptStopped(tr("script stopped", "脚本已停止"))


def _clear_stop_flag() -> None:
    """Clear the cooperative stop flag (used by the IDE before running)."""
    _stop_requested.clear()


def _request_stop() -> None:
    """Set the stop flag and queue a priority STOP; never wait for ACK on the
    GUI thread."""
    _stop_requested.set()
    _bridge.client.stop_all()   # priority queue; never wait for ACK on the GUI thread


# ---------------- Actions ----------------

def led(on: bool | None = None) -> None:
    """Control the onboard LED.

    on=True  -> LED_ON  (turn on, absolute state)
    on=False -> LED_OFF (turn off, absolute state)
    on=None  -> LED_TOGGLE (toggle, the default)
    """
    if on is not None and type(on) is not bool:
        raise ValueError("led() argument must be bool or None")
    if on is True:
        _bridge.send({"cmd": "LED_ON"})
    elif on is False:
        _bridge.send({"cmd": "LED_OFF"})
    else:
        _bridge.send({"cmd": "LED_TOGGLE"})


def drive_all(speed_cm_s: float) -> None:
    """Drive all four wheels at the same closed-loop speed (cm/s). Equivalent
    to peo.drive(0,v, 1,v, 2,v, 3,v)."""
    _drive_four([speed_cm_s] * 4)


def drive(*pairs) -> None:
    """Variadic motor control: peo.drive(motor, speed_cm_s, ...) takes
    (motor, speed) pairs and sends ONLY the motors you name (data update);
    unnamed motors keep their current target on the F407.

    Examples:
        peo.drive(0, 100, 1, 100, 2, 100, 3, 100)    all four forward
        peo.drive(0, 100, 1, 100, 2, -100, 3, -100)  spin in place
        peo.drive(1, 30)                             only M1 (others unchanged)

    motor is 0..3 (repeating the same motor is an error). Speed is cm/s and may
    be negative (reverse). To halt everything use peo.stop() (sends STOP_ALL).
    """
    if len(pairs) % 2 != 0:
        raise ValueError(tr(
            "drive() arguments must come in (motor, speed) pairs",
            "drive() 参数必须成对：peo.drive(motor, speed, ...)"))
    if not pairs:
        raise ValueError(tr(
            "drive() requires at least one (motor, speed) pair; use stop() to halt",
            "drive() 至少需要一对 (motor, speed)；全停请用 stop()"))
    seen = set()
    cmds = []
    for i in range(0, len(pairs), 2):
        motor, spd = pairs[i], pairs[i + 1]
        if not isinstance(motor, int) or isinstance(motor, bool) \
                or not 0 <= motor <= 3:
            raise ValueError(tr(
                f"invalid motor index {motor!r} (must be int 0-3)",
                f"电机编号 {motor!r} 非法（须为 0~3 的整数）"))
        if motor in seen:
            raise ValueError(tr(
                f"motor {motor} specified twice in one call",
                f"电机 {motor} 在一次调用中重复指定"))
        seen.add(motor)
        if isinstance(spd, bool):
            raise ValueError(tr(
                "speed must be a finite number",
                "速度必须是有限数值"))
        value = float(spd)
        if not _math.isfinite(value) or not -2147483648 <= value * 100 <= 2147483647:
            raise ValueError(tr(
                "speed outside finite int32 d100 range",
                "速度超出 int32 d100 范围"))
        cmds.append({"cmd": "MOTOR_CLOSED", "motor": motor,
                     "speed_d100": int(round(value * 100))})
    _bridge.send_batch(cmds)


def motor(*pairs) -> None:
    """Open-loop PWM direct drive (per-mille), bypassing the speed loop — the
    fallback when the closed-loop PID is misbehaving. Sends ONLY the motors you
    name (data update); unnamed motors keep their current PWM on the F407.

    speed unit = per-mille: -1000 full reverse .. +1000 full forward, 0 = coast.
    Positive = forward, negative = reverse. NOT cm/s (closed-loop drive() is
    cm/s).

    Examples:
        peo.motor(0, 500, 1, 500, 2, 500, 3, 500)     four wheels half forward
        peo.motor(0, -800, 1, -800, 2, -800, 3, -800) four wheels reverse
        peo.motor(0, 500, 1, 500, 2, -500, 3, -500)   spin in place
        peo.motor(1, 800)                             only M1 (others unchanged)

    motor is 0..3 (repeating the same motor is an error). speed must be an int
    in -1000..1000 (not a bool). Sending MOTOR switches the F407 to open loop;
    sending MOTOR_CLOSED (drive/drive_all) switches back to closed loop. Use
    peo.stop() to halt.
    """
    if len(pairs) % 2 != 0:
        raise ValueError(tr(
            "motor() arguments must come in (motor, speed) pairs",
            "motor() 参数必须成对：peo.motor(motor, speed, ...)"))
    if not pairs:
        raise ValueError(tr(
            "motor() requires at least one (motor, speed) pair; use stop() to halt",
            "motor() 至少需要一对 (motor, speed)；全停请用 stop()"))
    seen = set()
    cmds = []
    for i in range(0, len(pairs), 2):
        m, spd = pairs[i], pairs[i + 1]
        if not isinstance(m, int) or isinstance(m, bool) or not 0 <= m <= 3:
            raise ValueError(tr(
                f"invalid motor index {m!r} (must be int 0-3)",
                f"电机编号 {m!r} 非法（须为 0~3 的整数）"))
        if m in seen:
            raise ValueError(tr(
                f"motor {m} specified twice in one call",
                f"电机 {m} 在一次调用中重复指定"))
        if isinstance(spd, bool) or not isinstance(spd, int) or not -1000 <= spd <= 1000:
            raise ValueError(tr(
                f"invalid motor speed {spd!r} (must be int -1000..1000 per-mille)",
                f"电机速度 {spd!r} 非法（须为 -1000~1000 的整数千分比）"))
        seen.add(m)
        cmds.append({"cmd": "MOTOR", "motor": m, "speed": spd})
    _bridge.send_batch(cmds)


def _drive_four(speeds: list) -> None:
    """Validate all wheels before atomically publishing one local queue batch.

    The four UART commands are still independent on the vehicle. Repeated
    targets are resent: queue acceptance is not execution, and STOP/reset
    invalidates them.
    """
    d100 = []
    for value in speeds:
        if isinstance(value, bool):
            raise ValueError("speed must be a finite number")
        value = float(value)
        if not _math.isfinite(value) or not -2147483648 <= value * 100 <= 2147483647:
            raise ValueError("speed outside finite int32 d100 range")
        d100.append(int(round(value * 100)))
    _bridge.send_batch([{"cmd": "MOTOR_CLOSED", "motor": i, "speed_d100": speed}
                        for i, speed in enumerate(d100)])


def set_pid(*args, kp=None, ki=None) -> None:
    """Queue a shared four-wheel PI-gain command and write the config cache;
    returning does not mean the F407 has applied it.

    Valid forms:
        peo.set_pid(kp=0.3)         set only kp (ki keeps its current value)
        peo.set_pid(ki=0.01)        set only ki (kp keeps its current value)
        peo.set_pid(0.3, 0.01)      set both (two positional args)
    Invalid:
        peo.set_pid(0.25)           a single positional arg is ambiguous (kp or
                                    ki?) -> TypeError
    """
    if len(args) == 1:
        raise TypeError(tr(
            "set_pid(0.25) is ambiguous — use set_pid(kp=0.25) or set_pid(ki=0.25); "
            "to set both pass two arguments: set_pid(kp, ki)",
            "set_pid(0.25) 有歧义（无法区分 kp/ki）——请写 set_pid(kp=0.25) 或 "
            "set_pid(ki=0.25)；同时设置请传两个参数：set_pid(kp, ki)"))
    if len(args) == 2:
        if kp is not None or ki is not None:
            raise TypeError("Do not mix positional and keyword PID values")
        kp, ki = args
    elif len(args) > 2:
        raise TypeError(tr(
            "set_pid() takes at most 2 arguments",
            "set_pid() 最多接受两个参数"))
    if kp is None and ki is None:
        raise ValueError(tr(
            "set_pid() needs at least one of kp / ki",
            "set_pid() 至少需要 kp 或 ki 之一"))
    cfg = dict(_bridge._config.get("pid") or {})
    if kp is not None:
        cfg["kp"] = float(kp)
    if ki is not None:
        cfg["ki"] = float(ki)
    kp_v, ki_v = float(cfg.get("kp", 0.25)), float(cfg.get("ki", 0.005))
    if not all(_math.isfinite(v) and abs(v) <= 3.402823466e38 for v in (kp_v, ki_v)):
        raise ValueError("PID values must fit finite float32")
    _bridge.send({"cmd": "PID", "kp": kp_v, "ki": ki_v})
    _bridge._config["pid"] = {"kp": kp_v, "ki": ki_v}
    _bridge._save_config()


def stop() -> None:
    """Priority STOP; require a new successful S3 ACK, retry once on timeout.

    Raises TimeoutError if unconfirmed. This is S3 acceptance, not F407
    execution. Remains available after IDE cancellation.
    """
    _bridge.emergency_stop()


def reset_odometer() -> None:
    """Reset all four wheel odometers to zero."""
    _bridge.send({"cmd": "ODOMETER_RESET"})


def ws2812(r: int, g: int, b: int, brightness: int = 100) -> None:
    """Set the WS2812 status LED color + brightness (an S3-local command driven
    directly over RMT).

    r/g/b must be integers 0..255; brightness must be an integer 0..100.
    """
    for name, value, upper in (("r", r, 255), ("g", g, 255), ("b", b, 255),
                               ("brightness", brightness, 100)):
        if type(value) is not int or not 0 <= value <= upper:
            raise ValueError(f"{name} must be an integer in 0..{upper}")
    _bridge.send({"cmd": "WS2812", "r": r, "g": g, "b": b, "brightness": brightness})


def sys_reset() -> None:
    """Request an F407 soft reset (NVIC_SystemReset), leaving the S3 running.
    When the link recovers, the SDK detects the fresh edge and re-sends the
    cached PID/resolution automatically."""
    _bridge.send({"cmd": "SYS_RESET"})


# ---------------- Top-level read-only attributes ----------------

def __getattr__(name):
    """Resolve top-level read-only attributes (fresh, control_mode, versions,
    lock) on demand from the current snapshot."""
    if name == "fresh":
        return _bridge.fresh
    if name == "control_mode":
        s = _bridge._snapshot()
        return None if s is None else s.control_mode
    if name == "fw_version":
        s = _bridge._snapshot()
        return None if s is None else s.fw_version
    if name == "s3_fw":
        s = _bridge._snapshot()
        return None if s is None else s.s3_fw
    if name == "lock":
        s = _bridge._snapshot()
        return None if s is None else s.lock
    raise AttributeError(name)


__all__ = [
    "connect", "disconnect", "delay",
    "mpu", "vl53", "wheels", "sys", "cam",
    "led", "drive_all", "drive", "motor", "set_pid", "stop",
    "reset_odometer", "sys_reset", "ws2812",
    "control_mode", "fw_version", "s3_fw", "lock",
    "ScriptStopped", "NotConnected", "LinkLost", "CamOff",
    "_request_stop", "_clear_stop_flag", "_bridge",
]
