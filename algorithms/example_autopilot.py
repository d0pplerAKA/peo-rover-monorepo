"""
algorithms.example_autopilot — 示例算法：前方障碍自动刹车

演示组员写算法的完整套路：
1. 继承 NavAlgorithmBase；
2. on_sensor 里读传感器做判断；
3. 直接 self.client 发命令。

逻辑：前测距有效且 < 30cm → 每秒最多触发一次 STOP_ALL（防抖）。
"""
import time

from .base import NavAlgorithmBase


class BrakeOnObstacle(NavAlgorithmBase):
    name = "示例: 前方<30cm自动刹车"

    def __init__(self):
        super().__init__()
        self._last_brake = 0.0

    def on_sensor(self, sensor):
        dist = sensor.dist_front_mm
        ok = sensor.dist_front_status == 0
        if ok and 0 < dist < 300:
            now = time.monotonic()
            if now - self._last_brake > 1.0:
                self._last_brake = now
                if self.client is not None:      # 未连接时 client 为 None（防御）
                    self.client.stop_all()
                print(f"[{self.name}] 前方 {dist}mm < 300mm → STOP_ALL")
