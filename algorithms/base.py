"""
algorithms.base — 组员算法插件接口

组员写算法：新建一个 .py 文件放到本目录，继承 NavAlgorithmBase 实现任意方法，
main.py 启动时会自动扫描加载（无需改 GUI 代码）。

回调都在 GUI 主线程执行，可以直接调 self.client 发命令。
"""
from rover_comm.client import RoverClient
from rover_comm.protocol import SensorData


class NavAlgorithmBase:
    """自动导航算法基类。子类任意实现下面的方法（都是可选的）。"""

    name = "base"          # 显示名（日志/UI）

    def __init__(self):
        self.client: RoverClient | None = None   # on_start 时注入

    # ---- 生命周期 ----
    def on_start(self, client: RoverClient):
        """连接建立后调用（主线程）。保存 client 用于发命令。"""
        self.client = client

    def on_stop(self):
        """连接断开后调用（主线程）。"""
        self.client = None

    # ---- 数据回调（GUI 主线程）----
    def on_sensor(self, sensor: SensorData):
        """50Hz 传感器快照。"""

    def on_frame(self, frame_id: int, jpeg: bytes):
        """视频帧（JPEG 字节流）。"""
