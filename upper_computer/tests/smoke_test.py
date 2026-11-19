"""
tests/smoke_test - ground-station smoke test (headless, offscreen).

Run: python tests/smoke_test.py
Verifies: protocol layer, MainWindow v2 (Tab1 monitor + Tab2 IDE), sensor/video
data path, bilingual UI, E-STOP, log cap, performance display, heartbeat.
(2026-11-16: direct-control / gamepad / algorithm-plugin / macro panels were
removed along with GUI v2.)

Copyright (c) 2026 Danny
SPDX-License-Identifier: MIT
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

# 1) protocol self-check (including SYS_RESET 0x08)
assert crc16_ccitt(b"123456789") == 0x29B1
frame = pack_esp_frame(bytes(116))
assert unpack_esp_frame(frame) == bytes(116)
from rover_comm.protocol import CMD_SYS_RESET, cmd_sys_reset
assert CMD_SYS_RESET == 0x08
assert cmd_sys_reset() == bytes([0x08])   # payload = cmd_id(1B)
print("1) 协议层 ✓  含 SYS_RESET 0x08 帧编码")

# 2) main window construction + tab structure
win = MainWindow()
assert win.tabs.count() == 2
assert hasattr(win, "video") and hasattr(win, "btn_stop")       # Tab1 monitor
assert hasattr(win, "ide_edit") and hasattr(win, "btn_run")     # Tab2 IDE
assert hasattr(win, "shell_input") and hasattr(win, "script_output")
# direct-control / gamepad / macro are gone
assert not hasattr(win, "kp_spin") and not hasattr(win, "motor_spins")
assert not hasattr(win, "grp_macro")
print("2) MainWindow v2 构建 ✓  Tab1 监控 + Tab2 IDE；直控/宏已删")

# 3) sensor data -> dashboard
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

# 4) video frame (generate a valid 16x16 JPEG)
img = QImage(16, 16, QImage.Format_RGB32)
img.fill(0x0030FF30)
ba = QByteArray()
buf = QBuffer(ba)
buf.open(QBuffer.WriteOnly)
assert img.save(buf, "JPEG")
win._on_frame(42, bytes(ba.data()))
assert win.video._frame_id == 42
print("4) 视频帧显示 ✓  frame #", win.video._frame_id)

# 5) bilingual switch (top language button zh <-> en; default English — the GUI
#    audience is the international team)
from app.i18n import set_language, current_language
assert current_language() == "en"                # default English at startup
assert win.windowTitle() == "PEO Rover Ground Station (PySide6)"
assert win.btn_stop.text() == "E-STOP STOP_ALL"
assert win.tabs.tabText(0) == "Monitor" and win.tabs.tabText(1) == "Team IDE"
assert win.lang_btn.text() == "中文"              # button shows the target language
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

# 6) command ACK display (current interface is English)
win._on_ack({"ack": "MOTOR", "status": 0})
assert "MOTOR" in win.lbl_ack.text() and "OK" in win.lbl_ack.text()
win._on_ack({"ack": "PID", "status": 2})
assert "PID" in win.lbl_ack.text() and "error" in win.lbl_ack.text()
print("6) 命令 ACK ✓  日志 + 仪表盘回显")

# 7) E-STOP (the only command button on Tab 1; safe to send while disconnected)
win._send_json({"cmd": "STOP_ALL"})
print("7) 急停按钮 ✓  未连接安全返回")

# 8) shell REPL (InteractiveConsole: echo + persistent vars + multi-line + prompt)
import time as _t

def _shell_drain():
    win._drain_shell_out()
    return win.script_output.toPlainText()

win.shell_input.setText("1 + 1")
win._on_shell_enter()
_t.sleep(0.4)
txt = _shell_drain()
assert "2" in txt, txt                            # expression echo
win.shell_input.setText("x = 5")
win._on_shell_enter()
_t.sleep(0.3)
win.shell_input.setText("x * 2")
win._on_shell_enter()
_t.sleep(0.3)
txt = _shell_drain()
assert "10" in txt, txt                           # persistent variables
win.shell_input.setText("for i in range(3):")
win._on_shell_enter()
_t.sleep(0.3)
_shell_drain()
assert win._shell_more_flag is True               # continuation mode
win.shell_input.setText("    print(i)")
win._on_shell_enter()
_t.sleep(0.3)
win.shell_input.setText("")
win._on_shell_enter()
_t.sleep(0.3)
txt = _shell_drain()
assert "\n0\n" in txt and "\n1\n" in txt and "\n2" in txt, txt   # block ran
assert win._shell_more_flag is False              # back to the primary prompt
# peo env injection + unconnected exception
win.shell_input.setText("peo.drive_all(30)")
win._on_shell_enter()
_t.sleep(0.4)
txt = _shell_drain()
assert "NotConnected" in txt, txt
print("8) shell REPL ✓  回显/变量持久/多行续行/提示符/peo 注入")

# 9) IDE state machine (default code pre-filled + empty editor guard)
assert "peo.cam.set_resolution" in win.ide_edit.toPlainText()   # default code present
win.ide_edit.setPlainText("")                                    # empty to test the guard
win._on_run_script()
assert not win._script_running   # empty script must not start
assert "为空" in win.lbl_ide_status.text() or "empty" in win.lbl_ide_status.text()
from app.gui import _DEFAULT_SCRIPT
win.ide_edit.setPlainText(_DEFAULT_SCRIPT)   # restore default code (later items)
print("9) IDE 默认代码 + 空脚本保护 ✓")

# 10) log line cap (oldest lines scroll off)
for i in range(560):
    win._append_log(f"filler {i}")
assert win.script_output.document().blockCount() <= 500
print(f"10) 日志上限 ✓  blockCount={win.script_output.document().blockCount()} ≤ 500")

# 11) both tabs show only CPU0/CPU1/RAM; placeholders when offline/missing.
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

# 12) one-way safety heartbeat constant + callback registration; PING/RTT/
#     bad-frame counters were removed.
from rover_comm.client import HEARTBEAT_INTERVAL_S
assert 0 < HEARTBEAT_INTERVAL_S <= 1.0
assert win.client.on_ack is not None
for name in ("ping", "last_rtt_ms", "video_stats"):
    assert not hasattr(win.client, name)
print(f"12) 单向安全心跳 ✓  间隔 {HEARTBEAT_INTERVAL_S}s（ESP32 超时 2s 自动急停）")

print()
print("冒烟测试全部通过 ✓✓✓")
