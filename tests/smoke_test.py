"""
tests/smoke_test — 上位机冒烟测试（无 GUI 显示，offscreen）

运行：python tests/smoke_test.py
验证：协议层、MainWindow v2（Tab1 监控 + Tab2 IDE）、传感器/视频数据通路、
双语、急停、日志上限、性能显示、心跳。
（2026-11-16：直控面板/手柄面板/算法插件/宏面板已随 GUI v2 删除。）
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

# 1) 协议层自检（含 SYS_RESET 0x08）
assert crc16_ccitt(b"123456789") == 0x29B1
frame = pack_esp_frame(bytes(112))
assert unpack_esp_frame(frame) == bytes(112)
from rover_comm.protocol import CMD_SYS_RESET, cmd_sys_reset
assert CMD_SYS_RESET == 0x08
assert cmd_sys_reset() == bytes([0x08])   # payload = cmd_id(1B)
print("1) 协议层 ✓  含 SYS_RESET 0x08 帧编码")

# 2) 主窗口构建 + Tab 结构
win = MainWindow()
assert win.tabs.count() == 2
assert hasattr(win, "video") and hasattr(win, "btn_stop")       # Tab1 监控
assert hasattr(win, "ide_edit") and hasattr(win, "btn_run")     # Tab2 IDE
assert hasattr(win, "shell_input") and hasattr(win, "script_output")
# 直控/手柄/宏已删除
assert not hasattr(win, "kp_spin") and not hasattr(win, "motor_spins")
assert not hasattr(win, "grp_macro")
print("2) MainWindow v2 构建 ✓  Tab1 监控 + Tab2 IDE；直控/宏已删")

# 3) 传感器数据 → 仪表盘
s = SensorData(dist_front_mm=150, dist_left_mm=999, dist_right_mm=0,
               dist_front_status=0, dist_left_status=1, dist_right_status=1,
               speed_d100=(1000, -500, 0, 2000),
               current_mA=(100, 110, -95, 105),
               quat_w=0.99, accel_z=1.0)
win._on_sensor(s)
assert "150 mm" in win.lbl_dist_f.text()
from app.i18n import tr as _tr
assert f"999 mm({_tr('invalid')})" in win.lbl_dist_l.text()
assert "+10.0 cm/s" in win.lbl_spd[0].text()
print("3) 传感器仪表盘 ✓  前距 =", win.lbl_dist_f.text())

# 4) 视频帧（生成一个有效 16x16 JPEG）
img = QImage(16, 16, QImage.Format_RGB32)
img.fill(0x0030FF30)
ba = QByteArray()
buf = QBuffer(ba)
buf.open(QBuffer.WriteOnly)
assert img.save(buf, "JPEG")
win._on_frame(42, bytes(ba.data()))
assert win.video._frame_id == 42
print("4) 视频帧显示 ✓  frame #", win.video._frame_id)

# 5) 双语切换（顶部语言按钮 zh ⇄ en；默认英文——GUI 主要受众为国际组员）
from app.i18n import set_language, current_language
assert current_language() == "en"                # 启动默认英文
assert win.windowTitle() == "PEO Rover Ground Station (PySide6)"
assert win.btn_stop.text() == "E-STOP STOP_ALL"
assert win.tabs.tabText(0) == "Monitor" and win.tabs.tabText(1) == "Team IDE"
assert win.lang_btn.text() == "中文"              # 按钮显示目标语言（一眼可辨）
win._on_lang_toggle()
assert current_language() == "zh"
assert win.windowTitle().startswith("PEO Rover 上位机")
assert win.btn_stop.text() == "急停 STOP_ALL"
assert win.tabs.tabText(0) == "手动监控" and win.tabs.tabText(1) == "组员 IDE"
assert win.lang_btn.text() == "EN"
win._on_sensor(s)
assert "999 mm(无效)" in win.lbl_dist_l.text()
win._on_lang_toggle()
assert current_language() == "en"
assert win.lang_btn.text() == "中文"
win._on_sensor(s)
assert "999 mm(invalid)" in win.lbl_dist_l.text()
print("5) 双语切换 ✓  默认英文 + 顶部语言按钮 zh⇄en 全部界面文本随动")

# 6) 命令 ACK 展示（当前英文界面）
win._on_ack({"ack": "MOTOR", "status": 0})
assert "MOTOR" in win.lbl_ack.text() and "OK" in win.lbl_ack.text()
win._on_ack({"ack": "PID", "status": 2})
assert "PID" in win.lbl_ack.text() and "error" in win.lbl_ack.text()
print("6) 命令 ACK ✓  日志 + 仪表盘回显")

# 7) 急停（Tab1 唯一命令按钮，未连接安全下发）
win._send_json({"cmd": "STOP_ALL"})
print("7) 急停按钮 ✓  未连接安全返回")

# 8) shell REPL（InteractiveConsole：表达式回显 + 变量持久 + 多行续行 + 提示符）
import time as _t

def _shell_drain():
    win._drain_shell_out()
    return win.script_output.toPlainText()

win.shell_input.setText("1 + 1")
win._on_shell_enter()
_t.sleep(0.4)
txt = _shell_drain()
assert "2" in txt, txt                            # 表达式回显
win.shell_input.setText("x = 5")
win._on_shell_enter()
_t.sleep(0.3)
win.shell_input.setText("x * 2")
win._on_shell_enter()
_t.sleep(0.3)
txt = _shell_drain()
assert "10" in txt, txt                           # 变量会话持久
win.shell_input.setText("for i in range(3):")
win._on_shell_enter()
_t.sleep(0.3)
_shell_drain()
assert win._shell_more_flag is True               # 续行模式
win.shell_input.setText("    print(i)")
win._on_shell_enter()
_t.sleep(0.3)
win.shell_input.setText("")
win._on_shell_enter()
_t.sleep(0.3)
txt = _shell_drain()
assert "\n0\n" in txt and "\n1\n" in txt and "\n2" in txt, txt   # 多行块执行
assert win._shell_more_flag is False              # 回到主提示符
# peo 环境注入 + 未连接异常
win.shell_input.setText("peo.drive_all(30)")
win._on_shell_enter()
_t.sleep(0.4)
txt = _shell_drain()
assert "NotConnected" in txt, txt
print("8) shell REPL ✓  回显/变量持久/多行续行/提示符/peo 注入")

# 9) IDE 状态机（空编辑区保护）
win._on_run_script()
assert not win._script_running   # 空脚本不启动
assert "为空" in win.lbl_ide_status.text() or "empty" in win.lbl_ide_status.text()
print("9) IDE 空脚本保护 ✓")

# 10) 日志行数上限（自动覆盖最旧）
for i in range(560):
    win._append_log(f"filler {i}")
assert win.script_output.document().blockCount() <= 500
print(f"10) 日志上限 ✓  blockCount={win.script_output.document().blockCount()} ≤ 500")

# 11) 两个 Tab 只显示 CPU0/CPU1/RAM，断线或缺字段使用占位符。
from app.i18n import tr
win._update_perf()
assert win.lbl_perf.text() == "CPU0 -   CPU1 -   RAM -"
info = {"cpu0": 12, "cpu1": 34, "ram_free": 102400, "ram_total": 204800}
win.client.sys_info = info
win._update_perf()
assert win.lbl_perf.text() == "CPU0 12%   CPU1 34%   RAM 50%"
assert not hasattr(win, "_sensor_count") and not hasattr(win, "_frame_count")
import peo
peo._bridge.client.sys_info = info
win._update_perf_ide()
assert win.lbl_perf.text() == "CPU0 12%   CPU1 34%   RAM 50%"
assert (peo.sys.cpu0, peo.sys.cpu1, peo.sys.ram) == (12, 34, 50)
assert not hasattr(peo.sys, "rtt_ms")
win._on_lang_toggle()
win._update_perf_ide()
assert win.lbl_perf.text() == "CPU0 12%   CPU1 34%   RAM 50%"
win._on_lang_toggle()
peo._bridge.client.sys_info = None
win._update_perf_ide()
assert win.lbl_perf.text() == "CPU0 -   CPU1 -   RAM -"
win.client.sys_info = None
print("11) 性能显示 ✓ 两个 Tab / 双语只保留 CPU0、CPU1、RAM")

# 12) 单向安全心跳常量 + 回调注册；PING/RTT/坏帧统计接口已删除。
from rover_comm.client import HEARTBEAT_INTERVAL_S
assert 0 < HEARTBEAT_INTERVAL_S <= 1.0
assert win.client.on_ack is not None
for name in ("ping", "last_rtt_ms", "video_stats"):
    assert not hasattr(win.client, name)
print(f"12) 单向安全心跳 ✓  间隔 {HEARTBEAT_INTERVAL_S}s（ESP32 超时 2s 自动急停）")

print()
print("冒烟测试全部通过 ✓✓✓")
