"""
peo — PEO Rover 组员 SDK（组员唯一入口，PROTOCOL.md §2.4 组员编程模型）

三原语：数据属性（读）+ 动作函数（写）+ 延时（节奏）。判断/循环是组员自己的
原生 Python 代码，SDK 不提供任何逻辑封装。

用法：
    import peo
    peo.connect("peo_rover.local")

    peo.drive_all(30)
    while peo.fresh and peo.vl53.front_ok and peo.vl53.front_mm > 200:
        peo.delay(20)
    peo.stop()

    peo.cam.on()
    img, seq = peo.cam.read()
    peo.cam.off()
"""
import time as _time
import math as _math

from ._bridge import _bridge, ScriptStopped, NotConnected, LinkLost, CamOff, \
    _stop_requested
from ._i18n import tr
from .cam import cam

# 传感器数据命名空间（只读，约 25Hz 后台刷新；无数据/链路断时值为 None，
# 用 peo.fresh 判断链路是否活着）

class _MPU:
    @property
    def quat(self):
        s = _bridge._snapshot()
        return None if s is None else (s.quat_w, s.quat_x, s.quat_y, s.quat_z)

    @property
    def accel(self):
        s = _bridge._snapshot()
        return None if s is None else (s.accel_x, s.accel_y, s.accel_z)


class _VL53:
    @property
    def front_mm(self):
        s = _bridge._snapshot()
        return None if s is None else s.dist_front_mm

    @property
    def left_mm(self):
        s = _bridge._snapshot()
        return None if s is None else s.dist_left_mm

    @property
    def right_mm(self):
        s = _bridge._snapshot()
        return None if s is None else s.dist_right_mm

    @property
    def front_ok(self):
        s = _bridge._snapshot()
        return None if s is None else (s.dist_front_status == 0)

    @property
    def left_ok(self):
        s = _bridge._snapshot()
        return None if s is None else (s.dist_left_status == 0)

    @property
    def right_ok(self):
        s = _bridge._snapshot()
        return None if s is None else (s.dist_right_status == 0)


class _Wheels:
    @property
    def speed_cm_s(self):
        """四轮速度 cm/s（协议 0.01cm/s 定点 → 浮点）。"""
        s = _bridge._snapshot()
        return None if s is None else [v / 100.0 for v in s.speed_d100]

    @property
    def odometer(self):
        s = _bridge._snapshot()
        return None if s is None else list(s.odometer)

    @property
    def current_mA(self):
        s = _bridge._snapshot()
        return None if s is None else list(s.current_mA)

    @property
    def bus_voltage_mV(self):
        s = _bridge._snapshot()
        return None if s is None else list(s.bus_voltage_mV)


class _Sys:
    def __init__(self):
        self._cpu0 = None
        self._cpu1 = None
        self._ram = None

    def _update(self, info):
        self._cpu0 = info.get("cpu0") if info else None
        self._cpu1 = info.get("cpu1") if info else None
        free = info.get("ram_free") if info else None
        total = info.get("ram_total") if info else None
        self._ram = None if (free is None or not total) else \
            round((total - free) * 100 / total)

    @property
    def cpu0(self):
        self._update(_bridge.client.sys_info)
        return self._cpu0

    @property
    def cpu1(self):
        self._update(_bridge.client.sys_info)
        return self._cpu1

    @property
    def ram(self):
        self._update(_bridge.client.sys_info)
        return self._ram


mpu = _MPU()
vl53 = _VL53()
wheels = _Wheels()
sys = _Sys()


# ---------------- 生命周期 ----------------

def connect(host="peo_rover.local") -> None:
    """连接 ESP32（TCP 8888 命令/遥测 + TCP 8890 视频 + 心跳 + 单实例锁）。

    连接成功后自动下发缓存的 PID 与分辨率配置；F407 软复位后自动重发。
    """
    _bridge.connect(host)


def disconnect() -> None:
    _bridge.disconnect()


def delay(ms: float) -> None:
    """阻塞延时（上位机 Event.wait，精度受系统调度影响）。若期间被 IDE 停止按钮打断，
    抛 ScriptStopped 终止脚本线程。"""
    if _stop_requested.is_set():
        raise ScriptStopped(tr("script stopped", "脚本已停止"))
    if not _math.isfinite(ms) or ms < 0:
        raise ValueError("delay must be finite and non-negative")
    _stop_requested.wait(ms / 1000.0)
    if _stop_requested.is_set():
        raise ScriptStopped(tr("script stopped", "脚本已停止"))


def _clear_stop_flag() -> None:
    _stop_requested.clear()


def _request_stop() -> None:
    _stop_requested.set()
    _bridge.client.stop_all()   # priority queue; never wait for ACK on the GUI thread


# ---------------- 动作 ----------------

def led(on: bool | None = None) -> None:
    """控制板载 LED。

    on=True  -> LED_ON（点亮，绝对状态）
    on=False -> LED_OFF（熄灭，绝对状态）
    on=None  -> LED_TOGGLE（翻转，默认）
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
    """四轮同速（闭环 cm/s）。等价 peo.drive(0,v, 1,v, 2,v, 3,v)。"""
    _drive_four([speed_cm_s] * 4)


def drive(*pairs) -> None:
    """变参马达控制：peo.drive(motor, speed_cm_s, ...) 成对传入，
    **只发指定的电机（数据更新）**，未指定的电机在 F407 端保持当前转速。

    例：
        peo.drive(0, 100, 1, 100, 2, 100, 3, 100)   四轮同速前进
        peo.drive(0, 100, 1, 100, 2, -100, 3, -100) 原地转向
        peo.drive(1, 30)                            只改 M1（其余保持）

    motor ∈ 0~3（重复指定同一轮报错）。速度单位 cm/s，可为负（反转）。
    全停用 peo.stop()（发 STOP_ALL）。
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
    """开环 PWM 直驱（千分比），绕过速度闭环——闭环 PID 参数异常时的保底手段。
    **只发指定的电机（数据更新）**，未指定的电机在 F407 端保持当前 PWM。

    speed 单位 = 千分比（per-mille）：-1000 全速反转 ~ +1000 全速正转，
    0 = coast（滑行）。正=前进，负=反转。**不是 cm/s**（闭环 drive() 才是 cm/s）。

    例：
        peo.motor(0, 500, 1, 500, 2, 500, 3, 500)     四轮半速前进（开环）
        peo.motor(0, -800, 1, -800, 2, -800, 3, -800) 四轮后退
        peo.motor(0, 500, 1, 500, 2, -500, 3, -500)   原地转向（开环）
        peo.motor(1, 800)                             只改 M1（其余保持）

    motor ∈ 0~3（重复指定同一轮报错）。speed 须为 -1000~1000 的整数（非 bool）。
    发 MOTOR 会让 F407 切到开环模式；发 MOTOR_CLOSED（drive/drive_all）切回闭环。
    全停用 peo.stop()。
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

    The four UART commands are still independent on the vehicle. Repeated targets
    are resent: queue acceptance is not execution, and STOP/reset invalidates them.
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
    """排入四轮共享 PI 增益命令并写配置缓存；返回不代表 F407 已执行。

    合法形式：
        peo.set_pid(kp=0.3)         只设 kp（ki 保持当前值）
        peo.set_pid(ki=0.01)        只设 ki（kp 保持当前值）
        peo.set_pid(0.3, 0.01)      同时设（位置传两参）
    非法形式：
        peo.set_pid(0.25)           位置单参有歧义（不知道是 kp 还是 ki）→ 报错
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

    Raises TimeoutError if unconfirmed. This is S3 acceptance, not F407 execution.
    Remains available after IDE cancellation.
    """
    _bridge.emergency_stop()


def reset_odometer() -> None:
    """清零四轮里程。"""
    _bridge.send({"cmd": "ODOMETER_RESET"})


def ws2812(r: int, g: int, b: int, brightness: int = 100) -> None:
    """WS2812 灯珠颜色 + 亮度（S3 本地命令，RMT 直驱）。

    r/g/b must be integers 0..255; brightness must be an integer 0..100.
    """
    for name, value, upper in (("r", r, 255), ("g", g, 255), ("b", b, 255),
                               ("brightness", brightness, 100)):
        if type(value) is not int or not 0 <= value <= upper:
            raise ValueError(f"{name} must be an integer in 0..{upper}")
    _bridge.send({"cmd": "WS2812", "r": r, "g": g, "b": b, "brightness": brightness})


def sys_reset() -> None:
    """请求 F407 软复位（NVIC_SystemReset），不重启 S3。链路恢复时，
    SDK 检测 fresh 恢复沿自动重发 PID/分辨率。"""
    _bridge.send({"cmd": "SYS_RESET"})


# ---------------- 顶层只读属性 ----------------

def __getattr__(name):
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
