"""
rover_comm.script — 宏脚本构建器（组员友好 API）

宏由 ESP32-S3 执行器解析后逐条拆成 esp_link 命令发 F407（PROTOCOL.md §2.2）。
用法：
    from rover_comm.script import Macro
    m = Macro(id=1)
    m.act("MOTOR_CLOSED", motor=0, speed_d100=4000, duration_ms=0)
    m.wait("dist_front_mm", "<", 200, timeout_ms=5000)
    m.act("MOTOR_CLOSED", motor=0, speed_d100=0, duration_ms=500)
    m.goto(0)
    client.send(m.to_json())
"""
import json

# 动作命令参数表：cmd -> 必需参数名
_CMD_PARAMS = {
    "LED_TOGGLE":    (),
    "SENSOR_RESET":  (),
    "MOTOR":         ("motor", "speed"),
    "PID":           ("kp", "ki"),
    "MOTOR_CLOSED":  ("motor", "speed_d100"),
    "STOP_ALL":      (),
}

# 条件字段合法集（PROTOCOL.md §2.2）
_VALID_FIELDS = {
    "dist_front_mm", "dist_left_mm", "dist_right_mm",
    "dist_front_status", "dist_left_status", "dist_right_status",
    "quat_w", "quat_x", "quat_y", "quat_z",
    "accel_x", "accel_y", "accel_z",
    "speed_d100_0", "speed_d100_1", "speed_d100_2", "speed_d100_3",
    "odometer_0", "odometer_1", "odometer_2", "odometer_3",
    "current_mA_0", "current_mA_1", "current_mA_2", "current_mA_3",
    "bus_voltage_mV_0", "bus_voltage_mV_1", "bus_voltage_mV_2", "bus_voltage_mV_3",
    "sys_tick_ms",
}
_VALID_OPS = {"<", ">", "<=", ">=", "==", "!="}


class Macro:
    """宏脚本：动作序列 + 传感器条件 + 跳转（可循环）。"""

    def __init__(self, macro_id: int):
        self.id = macro_id
        self._steps = []

    # ---- 三种步骤 ----
    def act(self, cmd: str, duration_ms: int = 0, **params) -> 'Macro':
        """动作步骤：发一条 esp_link 命令。duration_ms=0 表示命令持续生效。"""
        if cmd not in _CMD_PARAMS:
            raise ValueError(f"未知命令: {cmd}（合法: {sorted(_CMD_PARAMS)}）")
        step = {"cmd": cmd, "duration_ms": duration_ms}
        for k in _CMD_PARAMS[cmd]:
            if k not in params:
                raise ValueError(f"命令 {cmd} 缺少参数 {k}")
            step[k] = params[k]
        self._steps.append(step)
        return self

    def wait(self, field: str, op: str, value, timeout_ms: int) -> 'Macro':
        """条件等待：轮询 SensorData[field] 与 value 比较，满足或超时才下一步。"""
        if field not in _VALID_FIELDS:
            raise ValueError(f"未知字段: {field}")
        if op not in _VALID_OPS:
            raise ValueError(f"未知操作符: {op}（合法: {sorted(_VALID_OPS)}）")
        self._steps.append({"wait": {"field": field, "op": op,
                                     "value": value, "timeout_ms": timeout_ms}})
        return self

    def goto(self, step: int) -> 'Macro':
        """跳转到 0-based 步骤索引（可形成循环）；越界 = 宏结束。"""
        if step < 0:
            raise ValueError("goto 索引不能为负")
        self._steps.append({"goto": step})
        return self

    # ---- 序列化 ----
    def to_dict(self) -> dict:
        return {"type": "macro", "id": self.id, "steps": self._steps}

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"))


def abort_macro(macro_id: int) -> dict:
    """中止宏消息（dict，供 send_json / send_macro 直接发送）。"""
    return {"type": "macro_abort", "id": macro_id}
