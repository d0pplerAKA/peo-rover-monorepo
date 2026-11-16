"""
algorithms.vision_line_follow — 视觉巡线模板：演示"视频→cv2→决策→命令"完整链路

给组员的三步上手：
1. pip install opencv-python
2. 只改 process() 里的图像处理逻辑（返回线中心偏差 -1..1，找不到线返回 None）
3. 重启上位机，本算法自动加载、自动接管（on_frame 里实时下发差速命令）

两种下发模式（本模板用①，注释里有②的写法）：
① 实时直控：on_frame 里逐条发标准命令（视觉闭环，10~20Hz）；
② 事件→短宏：视觉检测到事件后构建短宏一次下发（ESP32 自持执行，断连也安全）。
"""
import time

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None    # 未装 opencv 时本算法静默禁用

from .base import NavAlgorithmBase
from rover_comm import Macro


class VisionLineFollow(NavAlgorithmBase):
    name = "模板: 视觉巡线 (cv2)"

    BASE_SPEED = 3000     # 基础速度（0.01 cm/s 单位 → 30 cm/s）
    K_TURN = 8000         # 偏差 → 差速增益（err=±1 时差速 ±80 cm/s）
    CMD_INTERVAL = 0.05   # 命令节流：20Hz（视觉闭环够用，也保护 STM32 命令队列）

    def __init__(self):
        super().__init__()
        self._last_cmd = 0.0

    # ---------------- 视频回调（GUI 主线程） ----------------
    def on_frame(self, frame_id, jpeg):
        if cv2 is None:          # 未装 opencv，静默跳过
            return
        gray = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            return
        err = self.process(gray)
        if err is None:          # 丢失线 → 急停（安全默认行为）
            self.client.stop_all()
            return

        # 模式① 实时直控：差速命令（左轮 = 基础 - K*err，右轮 = 基础 + K*err）
        left = self.BASE_SPEED - int(self.K_TURN * err)
        right = self.BASE_SPEED + int(self.K_TURN * err)
        now = time.monotonic()
        if now - self._last_cmd < self.CMD_INTERVAL:
            return
        self._last_cmd = now
        self.client.send_json({"cmd": "MOTOR_CLOSED", "motor": 0,
                               "speed_d100": left, "duration_ms": 0})
        self.client.send_json({"cmd": "MOTOR_CLOSED", "motor": 1,
                               "speed_d100": right, "duration_ms": 0})

    # ---------------- 组员在这里改算法 ----------------
    def process(self, gray):
        """输入灰度图，返回线中心偏差 err ∈ [-1, 1]（左负右正）；找不到线返回 None。"""
        h, w = gray.shape
        roi = gray[int(h * 0.6):, :]                       # 只看画面下半部分
        _, th = cv2.threshold(roi, 60, 255, cv2.THRESH_BINARY_INV)  # 黑线→白色
        m = cv2.moments(th)
        if m["m00"] < 100:                                 # 有效像素太少 = 没找到线
            return None
        cx = m["m10"] / m["m00"]                           # 线中心 x 坐标
        return (cx - w / 2) / (w / 2)

    # ---------------- 模式② 事件 → 短宏（示例：留作组员参考） ----------------
    def example_event_macro(self):
        """视觉检测到"岔路"等事件时：构建短宏一次下发，ESP32 自持执行。"""
        m = Macro(9)
        m.act("MOTOR_CLOSED", motor=0, speed_d100=-2000, duration_ms=800)  # 后退 0.8s
        m.act("MOTOR_CLOSED", motor=0, speed_d100=2000, duration_ms=500)   # 左轮前进
        m.act("MOTOR_CLOSED", motor=1, speed_d100=-2000, duration_ms=500)  # 右轮后退
        m.act("STOP_ALL", duration_ms=0)                                   # 原地左转后停
        self.client.send_macro(m)
