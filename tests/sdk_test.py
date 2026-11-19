"""SDK 冒烟测试（离线：不连真车，验证 API 结构与内部机制）。"""
import os
import sys
import tempfile
import atexit
_test_config = tempfile.TemporaryDirectory(prefix="peo-sdk-test-")
atexit.register(_test_config.cleanup)
os.environ["PEO_CONFIG_PATH"] = os.path.join(_test_config.name, "config.json")

_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _root)                          # rover_comm 所在工程根
sys.path.insert(0, os.path.join(_root, "sdk"))     # peo 包

import peo
from peo import ScriptStopped, NotConnected, CamOff

# 1) 模块结构与命名空间
assert hasattr(peo, "connect") and hasattr(peo, "delay")
for ns in ("mpu", "vl53", "wheels", "sys", "cam"):
    assert hasattr(peo, ns), f"缺命名空间 {ns}"
for act in ("led", "drive_all", "drive", "set_pid", "stop",
            "reset_odometer", "sys_reset"):
    assert hasattr(peo, act), f"缺动作 {act}"
print("1) SDK 结构与命名空间 ✓")

# 2) 未连接行为：属性 None、动作抛 NotConnected
assert peo.vl53.front_mm is None and peo.mpu.quat is None
try:
    peo.drive_all(30)
    raise AssertionError("未连接应抛 NotConnected")
except NotConnected:
    pass
try:
    peo.cam.read()
    raise AssertionError("cam 未开应抛 CamOff")
except CamOff:
    pass
print("2) 未连接行为 ✓  None 属性 + NotConnected/CamOff 异常")

# 3) 传感器快照 → 属性映射（模拟 50Hz 更新）
from rover_comm.protocol import SensorData
s = SensorData(dist_front_mm=150, dist_left_mm=999, dist_right_mm=42,
               dist_front_status=0, dist_left_status=1, dist_right_status=0,
               speed_d100=(3000, -1500, 0, 2500), odometer=(1, 2, 3, 4),
               current_mA=(100, 110, -95, 105),
               quat_w=0.99, quat_x=0.01, quat_y=0.02, quat_z=0.03,
               accel_x=0.0, accel_y=0.0, accel_z=1.0)
peo._bridge._connected = True
peo._bridge._on_sensor(s)
assert peo.vl53.front_mm == 150 and peo.vl53.left_ok is False
assert peo.wheels.speed_cm_s == [30.0, -15.0, 0.0, 25.0]
assert peo.mpu.quat == (0.99, 0.01, 0.02, 0.03)
assert peo._bridge.fresh and peo.fresh is True
print("3) 传感器属性映射 ✓  mm/cm/s/四元数/fresh")

# 4) drive 变参语义（mock 验证发送内容）
sent = []
class FakeClient:
    connected = True
    sys_info = None
    def wait_ready(self, timeout=0.5): return True
    def stop_all(self): return True
    def send_json_batch(self, objects):
        sent.extend(objects)
        return True
    def send_json(self, obj):
        sent.append(obj)
        return True
peo._bridge.client = FakeClient()
peo._bridge._connected = True

def _reset_drive():
    sent.clear()

_reset_drive()
peo.drive(2, 30)                    # 只改 M2（数据更新，其余保持）
assert sent == [
    {"cmd": "MOTOR_CLOSED", "motor": 2, "speed_d100": 3000},
], sent
n_before = len(sent)
peo.drive(2, 30)                    # 重复目标也发送：STOP/拒绝后不能静默跳过
assert len(sent) == n_before + 1, "重复目标必须重新提交"

_reset_drive()
peo.drive(0, 100, 1, 100, 2, 100, 3, 100)   # 四轮同速
assert [o["speed_d100"] for o in sent] == [10000] * 4, sent

_reset_drive()
peo.drive(0, 100, 1, 100, 2, -100, 3, -100)  # 原地转向（负速度）
assert [o["speed_d100"] for o in sent] == [10000, 10000, -10000, -10000], sent

_reset_drive()
peo.drive(0, 0, 1, 100, 2, 0, 3, 0)          # 显式四轮（含 0）
assert [o["speed_d100"] for o in sent] == [0, 10000, 0, 0], sent

# 非法：奇数参数 / 电机越界 / 重复指定
for bad_args, exc in [
    ((1, 100, 2), ValueError),          # 不成对
    ((9, 100), ValueError),             # 越界
    ((1, 100, 1, 50), ValueError),      # 重复
    ((True, 100), ValueError),          # bool 不算 int
]:
    try:
        peo.drive(*bad_args)
        raise AssertionError(f"drive{bad_args} 应抛 {exc.__name__}")
    except exc:
        pass
print("4) drive 变参语义 ✓  单轮=数据更新 / 四轮 / 原地转向 / 非法参数校验")

# 5) delay 停止标志
import threading
peo._clear_stop_flag()
def _stopper():
    import time as t
    t.sleep(0.1)
    peo._request_stop()
threading.Thread(target=_stopper, daemon=True).start()
try:
    peo.delay(2000)
    raise AssertionError("停止标志未生效")
except ScriptStopped:
    pass
peo._clear_stop_flag()
print("5) delay 停止标志 ✓  协作式终止")

# 6) 配置读写 + set_pid 新语义
sent.clear()
peo.set_pid(kp=1.5, ki=0.05)            # 双关键字
assert sent[-1] == {"cmd": "PID", "kp": 1.5, "ki": 0.05}, sent
peo.set_pid(2.0, 0.02)                  # 双位置参数合法
assert sent[-1] == {"cmd": "PID", "kp": 2.0, "ki": 0.02}, sent
peo.set_pid(kp=3.0)                     # 只设 kp：ki 保持缓存 0.02
assert sent[-1] == {"cmd": "PID", "kp": 3.0, "ki": 0.02}, sent
peo.set_pid(ki=0.5)                     # 只设 ki：kp 保持缓存 3.0
assert sent[-1] == {"cmd": "PID", "kp": 3.0, "ki": 0.5}, sent
try:
    peo.set_pid(0.25)                   # 位置单参 = 非法（歧义）
    raise AssertionError("set_pid(0.25) 应抛 TypeError")
except TypeError:
    pass
cfg_path = peo._bridge._config_path()
import json
cfg = json.load(open(cfg_path, encoding="utf-8"))
assert cfg["pid"] == {"kp": 3.0, "ki": 0.5}, cfg
os.remove(cfg_path)   # 清理测试产物
print("6) set_pid 分离语义 ✓  kp=/ki= 独立 / 双参 / 位置单参非法 / 配置缓存")

# 7) 报错消息本地化（默认英文；GUI 注入 app.i18n 后随语言切换）
peo._bridge._connected = False
try:
    peo.drive_all(30)
except NotConnected as e:
    assert "not connected" in str(e), e
# 模拟 GUI 语言 = 中文
import types
fake_i18n = types.ModuleType("app.i18n")
fake_i18n.current_language = lambda: "zh"
sys.modules["app.i18n"] = fake_i18n
try:
    peo.drive_all(30)
except NotConnected as e:
    assert "未连接" in str(e), e
del sys.modules["app.i18n"]      # 还原独立运行
try:
    peo.drive_all(30)
except NotConnected as e:
    assert "not connected" in str(e), e
peo._bridge._connected = True
print("7) 报错消息本地化 ✓  en 默认 / GUI 中文随动 / 独立运行回 en")

print()
print("SDK 冒烟测试全部通过 ✓✓✓")
