"""
tests/smoke_test — 上位机冒烟测试（无 GUI 显示，offscreen）

运行：python tests/smoke_test.py
验证：模块 import、MainWindow 构建、算法自动加载、传感器/宏/视频数据通路。
"""
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PySide6.QtCore import QBuffer, QByteArray
from PySide6.QtGui import QImage
from PySide6.QtWidgets import QApplication

from app.gui import MainWindow
from rover_comm.protocol import SensorData, crc16_ccitt, pack_esp_frame, unpack_esp_frame

app = QApplication(sys.argv)

# 1) 协议层自检
assert crc16_ccitt(b"123456789") == 0x29B1
frame = pack_esp_frame(bytes(112))
assert unpack_esp_frame(frame) == bytes(112)
print("1) 协议层 ✓")

# 2) 主窗口构建 + 算法自动加载
win = MainWindow()
names = [a.name for a in win._algorithms]
assert any("刹车" in n for n in names), f"示例算法未加载: {names}"
print("2) MainWindow 构建 ✓  已加载算法:", names)

# 3) 传感器数据 → 仪表盘
s = SensorData(dist_front_mm=150, dist_left_mm=999, dist_right_mm=0,
               dist_front_status=0, dist_left_status=1, dist_right_status=1,
               speed_d100=(1000, -500, 0, 2000),
               current_mA=(100, 110, -95, 105),
               quat_w=0.99, accel_z=1.0)
win._on_sensor(s)
assert "150 mm" in win.lbl_dist_f.text()
assert "999 mm(无效)" in win.lbl_dist_l.text()
assert "+10.0 cm/s" in win.lbl_spd[0].text()
print("3) 传感器仪表盘 ✓  前距 =", win.lbl_dist_f.text())

# 4) 宏状态
win._on_macro({"id": 1, "step": 2, "total": 4, "state": "running"})
assert "运行中" in win.lbl_macro.text()   # 默认中文：state 已本地化
print("4) 宏状态显示 ✓ ", win.lbl_macro.text())

# 5) 视频帧（生成一个有效 16x16 JPEG）
img = QImage(16, 16, QImage.Format_RGB32)
img.fill(0x0030FF30)
ba = QByteArray()
buf = QBuffer(ba)
buf.open(QBuffer.WriteOnly)
assert img.save(buf, "JPEG")
win._on_frame(42, bytes(ba.data()))
assert win.video._frame_id == 42
print("5) 视频帧显示 ✓  frame #", win.video._frame_id)

# 6) 预设宏构建（不发送，只验证构建）
win._send_preset_macro()  # 未连接时只打日志，不崩
print("6) 预设宏构建 ✓")

# 7) 双语切换
from app.i18n import set_language, current_language
assert current_language() == "zh"
assert win.windowTitle().startswith("PEO Rover 上位机")
assert win.btn_stop.text() == "急停 STOP_ALL"
assert win.connect_btn.text() == "连接"
# 切英文
win.lang_combo.setCurrentIndex(1)
assert current_language() == "en"
assert win.windowTitle() == "PEO Rover Ground Station (PySide6)"
assert win.btn_stop.text() == "E-STOP STOP_ALL"
assert win.connect_btn.text() == "Connect"
assert win.grp_sensor.title() == "Sensor Dashboard"
# 英文下传感器数据重格式化（无效 → invalid）
win._on_sensor(s)
assert "999 mm(invalid)" in win.lbl_dist_l.text()
# 已有宏状态被英文重渲染（第 4 步设的 running）
assert "running" in win.lbl_macro.text()
# 英文下宏状态
win._on_macro({"id": 1, "step": 2, "total": 4, "state": "running"})
assert "running" in win.lbl_macro.text()
# 切回中文
win.lang_combo.setCurrentIndex(0)
assert current_language() == "zh"
assert win.windowTitle().startswith("PEO Rover 上位机")
win._on_sensor(s)
assert "999 mm(无效)" in win.lbl_dist_l.text()
print("7) 双语切换 ✓  zh⇄en 全部界面文本随动")

# 8) 手柄 2.5D 动画面板
from rover_comm.protocol import GamepadState, BTN_A, BTN_LB, BTN_DPAD_UP
assert not win.gamepad.state().connected          # 初始未连接
g = GamepadState(connected=True, battery=87,
                 joy_l=(50000, 15000), joy_r=(32768, 32768),
                 triggers=(800, 0), buttons=frozenset({BTN_A, BTN_DPAD_UP}))
win._on_gamepad(g)
assert win.gamepad.state().connected
assert win.gamepad.state().pressed(BTN_A)
assert win.gamepad.state().joy_l == (50000, 15000)   # 摇杆位置保留原始值
img = win.gamepad.grab().toImage()                   # offscreen 渲染非空
assert not img.isNull() and img.width() > 100
win._on_disconnected()                               # 断开 → 手柄复位
assert not win.gamepad.state().connected
print("8) 手柄 2.5D 面板 ✓  按键/摇杆/扳机状态 + 渲染非空")

# 9) client 层 gamepad JSON 解析
from rover_comm import GamepadState as GS
j = {"connected": True, "battery": 55,
     "joy_l": [100, 200], "joy_r": [300, 400],
     "triggers": [10, 20], "buttons": ["A", "LB"]}
gs = GS.from_json(j)
assert gs.connected and gs.battery == 55
assert gs.joy_l == (100, 200) and gs.triggers == (10, 20)
assert gs.pressed("LB") and not gs.pressed("X")
print("9) GamepadState.from_json ✓  sensor JSON gamepad 字段解析")

# 10) 主题固定暗色（不再切换）
app = QApplication.instance()
assert "1e1f24" in app.styleSheet()          # 暗色 QSS 已应用
assert not hasattr(win, "theme_combo")       # 主题切换控件已移除
print("10) 暗色主题固定 ✓  无切换控件")

# 11) 命令 ACK 展示
win._on_ack({"ack": "MOTOR", "status": 0})
assert "MOTOR" in win.lbl_ack.text() and "OK" in win.lbl_ack.text()
win._on_ack({"ack": "PID", "status": 2})
assert "PID" in win.lbl_ack.text() and "错误" in win.lbl_ack.text()
print("11) 命令 ACK ✓  日志 + 仪表盘回显")

# 12) 直控面板（未连接安全下发，不崩）
win.kp_spin.setValue(1.5); win.ki_spin.setValue(0.05)
win._send_pid()
win.motor_spins[2][1].setValue(-3000)
win._send_motor(2)
win._send_motor_all()
assert win.motor_spins[3][1].value() == win.motor_spins[0][1].value()
print("12) 直控面板 ✓  PID/四轮速度发送（未连接安全返回）")

# 13) 日志行数上限（自动覆盖最旧）
for i in range(560):
    win._append_log(f"filler {i}")
assert win.log_view.document().blockCount() <= 500
print(f"13) 日志上限 ✓  blockCount={win.log_view.document().blockCount()} ≤ 500")

# 14) 性能/帧率计数
win._sensor_count = 0      # 清零后再测（前面步骤已累计）
win._frame_count = 0
for _ in range(10):
    win._on_sensor(s)
win._on_frame(1, bytes(ba.data()))
win._update_perf()
assert "10 Hz" in win.lbl_perf.text() and "1 fps" in win.lbl_perf.text()
from app.i18n import tr
for key in ("perf_cpu0", "perf_cpu1", "perf_ram", "perf_rtt"):
    assert f"{tr(key)} -" in win.lbl_perf.text()
assert win._sensor_count == 0 and win._frame_count == 0
win.client.sys_info = {"cpu0": 12, "cpu1": 34, "ram_free": 102400, "ram_total": 204800}
win.client.last_rtt_ms = 23
win._update_perf()
assert "0 Hz" in win.lbl_perf.text() and "0 fps" in win.lbl_perf.text()
assert "CPU0 12%" in win.lbl_perf.text() and "CPU1 34%" in win.lbl_perf.text()
assert f"{tr('perf_ram')} 50%" in win.lbl_perf.text()   # RAM 只显示百分比（与 CPU 同格式）
assert f"{tr('perf_rtt')} 23 ms" in win.lbl_perf.text()
print("14) 性能显示 ✓ ", win.lbl_perf.text())

# 15) 心跳常量 + 回调注册
from rover_comm.client import HEARTBEAT_INTERVAL_S
assert 0 < HEARTBEAT_INTERVAL_S <= 1.0
assert win.client.on_ack is not None
print(f"15) 心跳 ✓  间隔 {HEARTBEAT_INTERVAL_S}s（ESP32 超时 2s 自动急停）")

print()
print("冒烟测试全部通过 ✓✓✓")
