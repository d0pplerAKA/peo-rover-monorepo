"""
app.gui — 主窗口：视频 + 传感器仪表盘 + 命令/宏面板 + 日志 + 算法插件

线程模型：
- RoverClient 的接收线程回调 → 跨线程 emit Qt signal → 主线程槽更新 UI。
- 算法插件的回调都在主线程执行，可直接 self.client 发命令。

双语：所有界面文本经 app.i18n.tr() 取；语言下拉框切换时 _retranslate() 刷新。
"""
import importlib
import json as _json
import os
import pkgutil
import time
import threading

from PySide6.QtCore import Qt, Signal, Slot, QTimer
from PySide6.QtGui import QImage, QShortcut, QKeySequence, QTextCursor
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QFormLayout,
    QLabel, QLineEdit, QPushButton, QGroupBox, QTextEdit, QSplitter,
    QPlainTextEdit, QComboBox, QSpinBox, QDoubleSpinBox, QApplication,
)

from rover_comm.client import RoverClient
from rover_comm.protocol import SensorData, GamepadState, TCP_CMD_PORT
from rover_comm.script import Macro, abort_macro
from app.i18n import tr, set_language, LANG_NAMES
from app.theme import apply_theme
from app.video import VideoWidget
from app.gamepad_widget_improved import GamepadWidget
import algorithms
from algorithms.base import NavAlgorithmBase

_MACRO_STATE_KEY = {"running": "macro_running", "done": "macro_done",
                    "aborted": "macro_aborted"}

_MAX_LOG_BLOCKS = 500      # 日志行数上限：超出自动覆盖最旧
_PERF_INTERVAL_MS = 1000   # 性能计数刷新周期

# 坏帧取证：解码失败/结构坏的帧落盘（最近 20 个），供字节级分析定位根因
_BAD_FRAME_DIR = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "bad_frames")
_BAD_FRAME_MAX = 20


def _save_bad_frame(jpeg: bytes, frame_id: int, reason: str):
    """坏帧落盘（调试取证用；限最近 20 个，最旧自动删除）。"""
    try:
        os.makedirs(_BAD_FRAME_DIR, exist_ok=True)
        path = os.path.join(_BAD_FRAME_DIR,
                            f"f{frame_id}_{time.strftime('%H%M%S')}_{reason}.jpg")
        with open(path, "wb") as f:
            f.write(jpeg)
        files = sorted(os.listdir(_BAD_FRAME_DIR))
        for old in files[:-_BAD_FRAME_MAX]:
            try:
                os.remove(os.path.join(_BAD_FRAME_DIR, old))
            except OSError:
                pass
    except OSError:
        pass   # 取证失败不影响主流程
_CAM_RESOLUTIONS = (("QQVGA (160x120)", "QQVGA"),
                    ("QVGA (320x240)", "QVGA"),
                    ("VGA (640x480)", "VGA"),
                    ("SVGA (800x600)", "SVGA"))   # 相机分辨率选项（显示名含实际像素，PROTOCOL §2 CAM_RES）


def _load_algorithms():
    """扫描 algorithms/ 下所有实现 NavAlgorithmBase 的类并实例化。"""
    found = []
    for mod in pkgutil.iter_modules(algorithms.__path__):
        if mod.name.startswith("_"):
            continue
        try:
            m = importlib.import_module(f"algorithms.{mod.name}")
        except ImportError:
            continue
        for obj in vars(m).values():
            if isinstance(obj, type) and issubclass(obj, NavAlgorithmBase) \
                    and obj is not NavAlgorithmBase:
                found.append(obj())
    return found


class MainWindow(QMainWindow):
    # 跨线程信号（RoverClient 接收线程 emit → 主线程槽，Qt 自动 queued）
    _sig_sensor = Signal(object)      # SensorData
    _sig_frame = Signal(int, bytes)   # frame_id, jpeg
    _sig_macro = Signal(object)       # dict
    _sig_gamepad = Signal(object)     # GamepadState
    _sig_ack = Signal(object)         # dict {"ack":cmd,"status":n}
    _sig_bad_frame = Signal(int, bytes)  # (frame_id, jpeg) 组装层坏帧取证
    _sig_log = Signal(str)
    _sig_disconn = Signal()
    _sig_conn = Signal(bool, str)       # (成功?, host 或错误信息)

    def __init__(self):
        super().__init__()
        self._algorithms = _load_algorithms()
        self._macro_state = None      # 最近宏状态 dict（切语言时重渲染）
        self._sensor_count = 0        # 性能计数（1s 窗口）
        self._perf_tick = 0           # PING 节拍计数（每 5s 发一次，降低链路开销）
        self._frame_count = 0
        self._video_on = False

        self.client = RoverClient()
        self.client.on_sensor = self._sig_sensor.emit          # 接收线程
        self.client.on_frame = lambda fid, jpg: self._sig_frame.emit(fid, jpg)
        self.client.on_macro_status = self._sig_macro.emit
        self.client.on_gamepad = self._sig_gamepad.emit
        self.client.on_ack = self._sig_ack.emit
        self.client.on_bad_frame = lambda jpg, fid: self._sig_bad_frame.emit(fid, jpg)
        self.client.on_disconnected = self._sig_disconn.emit

        self._sig_sensor.connect(self._on_sensor)
        self._sig_frame.connect(self._on_frame)
        self._sig_macro.connect(self._on_macro)
        self._sig_gamepad.connect(self._on_gamepad)
        self._sig_ack.connect(self._on_ack)
        self._sig_bad_frame.connect(self._on_bad_frame)
        self._sig_log.connect(self._append_log)
        self._sig_conn.connect(self._on_connect_result)
        self._sig_disconn.connect(self._on_disconnected)

        self._build_ui()          # 只建控件和布局
        self._retranslate()       # 按当前语言填文本

        # 固定暗色主题
        apply_theme(QApplication.instance())

        # 急停快捷键：空格（文本框聚焦时不触发）
        self._stop_sc = QShortcut(QKeySequence(Qt.Key_Space), self)
        self._stop_sc.activated.connect(self._on_shortcut_stop)

        # 性能计数：1s 刷新一次状态栏
        self._perf_timer = QTimer(self)
        self._perf_timer.timeout.connect(self._update_perf)
        self._perf_timer.start(_PERF_INTERVAL_MS)
        self._update_perf()     # 立即显示初始 "0 Hz / 0 fps"

        self._log(tr("log_ready"))
        if self._algorithms:
            self._log(tr("log_algo") + ", ".join(a.name for a in self._algorithms))

    # ============================ UI 构建 ============================
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(4, 8, 4, 4)
        splitter = QSplitter(Qt.Horizontal)
        root.addWidget(splitter)

        # ── 列1：视频 + 分辨率切换 + 连接（连接面板在视频流下方）──
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)
        camrow = QHBoxLayout()
        self.lbl_cam_res = QLabel()
        camrow.addWidget(self.lbl_cam_res)
        self.cam_res_combo = QComboBox()
        self.cam_res_combo.addItems([d for d, _ in _CAM_RESOLUTIONS])
        self.cam_res_combo.setCurrentIndex(1)   # 默认 QVGA (320x240)
        self.cam_res_combo.activated.connect(self._on_cam_res_changed)
        camrow.addWidget(self.cam_res_combo)
        camrow.addStretch(1)
        lv.addLayout(camrow)
        self.video = VideoWidget()
        lv.addWidget(self.video, 3)
        lv.addWidget(self._build_conn_group(), 2)
        splitter.addWidget(left)

        # ── 列2：手柄实时 + 传感器仪表盘 + 命令 + 直控（PID/电机） ──
        mid = QWidget()
        mv = QVBoxLayout(mid)
        mv.setContentsMargins(0, 0, 0, 0)
        self.grp_gamepad = QGroupBox()
        gp = QVBoxLayout(self.grp_gamepad)
        self.gamepad = GamepadWidget(colorway="velocity_green", background=None)
        gp.addWidget(self.gamepad)
        mv.addWidget(self.grp_gamepad, 2)
        mv.addWidget(self._build_sensor_group())
        mv.addWidget(self._build_cmd_group())
        mv.addWidget(self._build_ctrl_group())
        mv.addStretch(1)
        splitter.addWidget(mid)

        # ── 列3：宏脚本 + 控制台（日志占满剩余） ──
        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(0, 0, 0, 0)
        rv.addWidget(self._build_macro_group())
        # 控制台标题 + 清空按钮 + 日志
        console_row = QHBoxLayout()
        self.lbl_console = QLabel()
        console_row.addWidget(self.lbl_console)
        console_row.addStretch(1)
        self.btn_console_clear = QPushButton()
        self.btn_console_clear.setFixedWidth(64)
        self.btn_console_clear.clicked.connect(self._on_clear_console)
        console_row.addWidget(self.btn_console_clear)
        rv.addLayout(console_row)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMinimumHeight(80)
        rv.addWidget(self.log_view, 1)
        splitter.addWidget(right)

        splitter.setSizes([440, 400, 340])

        # 状态栏：性能/帧率
        self.lbl_perf = QLabel()
        self.statusBar().addPermanentWidget(self.lbl_perf)

    def _build_conn_group(self):
        g = QGroupBox()
        self.grp_conn = g
        f = QFormLayout(g)
        f.setContentsMargins(6, 2, 6, 4)
        f.setVerticalSpacing(3)

        row = QHBoxLayout()
        self.host_edit = QLineEdit("192.168.4.1")
        self.connect_btn = QPushButton()
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        self.disconnect_btn = QPushButton()
        self.disconnect_btn.clicked.connect(self._on_disconnect_clicked)
        self.disconnect_btn.setEnabled(False)   # 未连接时不可点
        self.video_btn = QPushButton()
        self.video_btn.setEnabled(False)
        self.video_btn.clicked.connect(self._on_video_clicked)
        row.addWidget(self.host_edit, 1)
        row.addWidget(self.connect_btn)
        row.addWidget(self.disconnect_btn)
        row.addWidget(self.video_btn)
        self.lbl_ip = QLabel()
        f.addRow(self.lbl_ip, row)

        self.lbl_status = QLabel()
        self.conn_status = QLabel()
        f.addRow(self.lbl_status, self.conn_status)
        # 语言（下拉切换中英文）
        self.lbl_lang = QLabel()
        self.lang_combo = QComboBox()
        self.lang_combo.addItems([name for _, name in LANG_NAMES])
        self.lang_combo.currentIndexChanged.connect(self._on_lang_changed)
        row2 = QHBoxLayout()
        row2.addWidget(self.lbl_lang)
        row2.addWidget(self.lang_combo)
        row2.addStretch(1)
        f.addRow(row2)
        return g

    def _build_sensor_group(self):
        g = QGroupBox()
        self.grp_sensor = g
        grid = QGridLayout(g)
        grid.setContentsMargins(6, 2, 6, 4)
        grid.setVerticalSpacing(2)

        self.lbl_dist_f = QLabel("—"); self.lbl_dist_l = QLabel("—"); self.lbl_dist_r = QLabel("—")
        self.lbl_spd = [QLabel("—") for _ in range(4)]
        self.lbl_cur = [QLabel("—") for _ in range(4)]
        self.lbl_imu = QLabel("—")
        self.lbl_ack = QLabel("—")
        self.lbl_macro = QLabel()

        self._sensor_labels = []          # (QLabel 标题, tr key)，_retranslate 时刷新标题
        r = 0
        for key, val in (("dist_front", self.lbl_dist_f), ("dist_left", self.lbl_dist_l),
                         ("dist_right", self.lbl_dist_r)):
            t = QLabel(); grid.addWidget(t, r, 0); grid.addWidget(val, r, 1, 1, 5)
            self._sensor_labels.append((t, key)); r += 1
        self._spd_title = [QLabel() for _ in range(4)]
        self._cur_title = [QLabel() for _ in range(4)]
        for i in range(4):
            grid.addWidget(self._spd_title[i], r, i * 2)
            grid.addWidget(self.lbl_spd[i], r, i * 2 + 1)
        r += 1
        for i in range(4):
            grid.addWidget(self._cur_title[i], r, i * 2)
            grid.addWidget(self.lbl_cur[i], r, i * 2 + 1)
        r += 1
        t = QLabel(); grid.addWidget(t, r, 0); grid.addWidget(self.lbl_imu, r, 1, 1, 5)
        self._sensor_labels.append((t, "imu"))
        r += 1
        # ACK 与宏状态合并一行（各占半行，省纵向空间）
        t = QLabel(); grid.addWidget(t, r, 0); grid.addWidget(self.lbl_ack, r, 1, 1, 2)
        self._sensor_labels.append((t, "cmd_ack"))
        t = QLabel(); grid.addWidget(t, r, 3); grid.addWidget(self.lbl_macro, r, 4, 1, 2)
        self._sensor_labels.append((t, "macro"))
        return g

    def _build_cmd_group(self):
        g = QGroupBox()
        self.grp_cmd = g
        v = QVBoxLayout(g)
        v.setContentsMargins(6, 2, 6, 4)
        row = QHBoxLayout()
        self.btn_stop = QPushButton()
        self.btn_stop.setStyleSheet(
            "QPushButton{background:#c0392b;color:white;font-weight:bold;padding:8px}"
            "QPushButton:hover{background:#e74c3c}")
        self.btn_stop.clicked.connect(lambda: self._send_json({"cmd": "STOP_ALL"}))
        self.btn_led = QPushButton()
        self.btn_led.clicked.connect(lambda: self._send_json({"cmd": "LED_TOGGLE"}))
        self.btn_srst = QPushButton()
        self.btn_srst.clicked.connect(lambda: self._send_json({"cmd": "SENSOR_RESET"}))
        self.btn_oreset = QPushButton()
        self.btn_oreset.clicked.connect(lambda: self._send_json({"cmd": "ODOMETER_RESET"}))
        row.addWidget(self.btn_stop, 2)
        row.addWidget(self.btn_led, 1)
        row.addWidget(self.btn_srst, 1)
        row.addWidget(self.btn_oreset, 1)
        v.addLayout(row)
        return g

    def _build_ctrl_group(self):
        """直控面板：PID 增益 + 四轮闭环速度（2×2 网格，省纵向空间）。"""
        g = QGroupBox()
        self.grp_ctrl = g
        v = QVBoxLayout(g)
        v.setContentsMargins(6, 2, 6, 4)

        # PID 行（标题并入行内，省纵向空间）
        pid_row = QHBoxLayout()
        self.lbl_pid = QLabel()
        self.kp_spin = QDoubleSpinBox()
        self.kp_spin.setRange(0.0, 100.0)
        self.kp_spin.setDecimals(3)
        self.kp_spin.setSingleStep(0.005)
        self.kp_spin.setValue(0.25)
        self.ki_spin = QDoubleSpinBox()
        self.ki_spin.setRange(0.0, 10.0)
        self.ki_spin.setDecimals(3)
        self.ki_spin.setSingleStep(0.001)
        self.ki_spin.setValue(0.005)
        self.lbl_kp = QLabel()
        self.lbl_ki = QLabel()
        self.btn_pid = QPushButton()
        self.btn_pid.clicked.connect(self._send_pid)
        pid_row.addWidget(self.lbl_pid)
        pid_row.addWidget(self.lbl_kp)
        pid_row.addWidget(self.kp_spin)
        pid_row.addWidget(self.lbl_ki)
        pid_row.addWidget(self.ki_spin)
        pid_row.addWidget(self.btn_pid)
        v.addLayout(pid_row)

        # 四轮速度 2×2 网格（标题并入"四轮同速"按钮行前）
        grid = QGridLayout()
        self.motor_spins = []
        for i in range(4):
            row = QHBoxLayout()
            lbl = QLabel(f"M{i + 1}")
            sp = QSpinBox()
            sp.setRange(-5000, 5000)       # 0.01 cm/s
            sp.setValue(0)
            sp.setSingleStep(100)
            btn = QPushButton()
            btn.clicked.connect(lambda _c=False, mi=i: self._send_motor(mi))
            row.addWidget(lbl)
            row.addWidget(sp, 1)
            row.addWidget(btn)
            grid.addLayout(row, i // 2, i % 2)
            self.motor_spins.append((lbl, sp, btn))
        v.addLayout(grid)
        self.btn_motor_all = QPushButton()
        self.btn_motor_all.clicked.connect(self._send_motor_all)
        row_all = QHBoxLayout()
        self.lbl_motor = QLabel()
        row_all.addWidget(self.lbl_motor)
        row_all.addWidget(self.btn_motor_all)
        v.addLayout(row_all)
        return g

    def _build_macro_group(self):
        g = QGroupBox()
        self.grp_macro = g
        v = QVBoxLayout(g)
        v.setContentsMargins(6, 2, 6, 4)
        self.btn_preset = QPushButton()
        self.btn_preset.clicked.connect(self._send_preset_macro)
        self.macro_edit = QPlainTextEdit()
        self.macro_edit.setMaximumHeight(48)
        row = QHBoxLayout()
        self.btn_send_macro = QPushButton()
        self.btn_send_macro.clicked.connect(self._send_custom_macro)
        self.btn_abort = QPushButton()
        self.btn_abort.clicked.connect(lambda: self._send_json(abort_macro(1)))
        row.addWidget(self.btn_send_macro)
        row.addWidget(self.btn_abort)
        v.addWidget(self.btn_preset)
        v.addWidget(self.macro_edit)
        v.addLayout(row)
        return g

    # ============================ 文本（双语刷新） ============================
    def _retranslate(self):
        self.setWindowTitle(tr("app_title"))
        self.grp_conn.setTitle(tr("grp_conn"))
        self.lbl_ip.setText(tr("ip"))
        self.connect_btn.setText(tr("btn_connect"))
        self.disconnect_btn.setText(tr("btn_disconnect"))
        self.video_btn.setText(tr("btn_video_off" if self.client.connected and self._video_on else "btn_video_on"))
        self.lbl_status.setText(tr("status"))
        self.conn_status.setText(tr("conn_yes") if self.client.connected else tr("conn_no"))
        self.lbl_lang.setText(tr("lang"))
        self.lbl_cam_res.setText(tr("cam_res"))

        self.grp_sensor.setTitle(tr("grp_sensor"))
        for t, key in self._sensor_labels:
            t.setText(tr(key))
        for i in range(4):
            self._spd_title[i].setText(tr("spd_col", i + 1))
            self._cur_title[i].setText(tr("cur_col", i + 1))
        self._render_macro_state()

        self.grp_cmd.setTitle(tr("grp_cmd"))
        self.btn_stop.setText(tr("btn_stop"))
        self.btn_led.setText(tr("btn_led"))
        self.btn_srst.setText(tr("btn_srst"))
        self.btn_oreset.setText(tr("btn_oreset"))

        self.grp_ctrl.setTitle(tr("grp_ctrl"))
        self.lbl_pid.setText(tr("lbl_pid"))
        self.lbl_kp.setText(tr("lbl_kp"))
        self.lbl_ki.setText(tr("lbl_ki"))
        self.btn_pid.setText(tr("btn_pid_apply"))
        self.lbl_motor.setText(tr("lbl_motor"))
        for _lbl, _sp, btn in self.motor_spins:
            btn.setText(tr("btn_motor_set"))
        self.btn_motor_all.setText(tr("btn_motor_all"))

        self.grp_macro.setTitle(tr("grp_macro"))
        self.btn_preset.setText(tr("btn_preset"))
        self.macro_edit.setPlaceholderText(tr("macro_ph"))
        self.btn_send_macro.setText(tr("btn_send"))
        self.btn_abort.setText(tr("btn_abort"))
        self.lbl_console.setText(tr("grp_console"))
        self.btn_console_clear.setText(tr("console_clear"))

        self.video.update()    # 视频占位文本按新语言重绘
        self.grp_gamepad.setTitle(tr("grp_gamepad"))
        self.gamepad.update()  # 手柄面板文本（电量/未连接提示）按新语言重绘

    def _render_macro_state(self):
        if self._macro_state is None:
            self.lbl_macro.setText(tr("macro_none"))
            return
        m = self._macro_state
        state_txt = tr(_MACRO_STATE_KEY.get(m.get("state"), "macro_running"))
        self.lbl_macro.setText(
            f"{tr('macro_hash').format(m.get('id'))} "
            f"{tr('macro_step')}{m.get('step')}/{m.get('total')} [{state_txt}]")

    def _on_lang_changed(self, idx: int):
        if 0 <= idx < len(LANG_NAMES):
            set_language(LANG_NAMES[idx][0])
        self._retranslate()

    def _on_shortcut_stop(self):
        """空格急停；文本框聚焦时不触发。"""
        fw = QApplication.focusWidget()
        if isinstance(fw, (QLineEdit, QPlainTextEdit, QTextEdit)):
            return
        if self._send_json({"cmd": "STOP_ALL"}):
            self._log(tr("log_sc_stop"))

    def _on_cam_res_changed(self, idx: int):
        """相机分辨率热切换：{\"cmd\":\"CAM_RES\",\"res\":...}（ESP32-S3 本地命令，
        相机任务 deinit+reinit 应用，约半秒黑屏；失败自动回退原分辨率）。"""
        if not 0 <= idx < len(_CAM_RESOLUTIONS):
            return
        disp, res = _CAM_RESOLUTIONS[idx]
        if self._send_json({"cmd": "CAM_RES", "res": res}):
            self._log(tr("log_cam_res").format(disp))

    def _update_perf(self):
        self._perf_tick = (self._perf_tick + 1) % 5
        if self.client.connected and self._perf_tick == 0:
            self.client.ping()   # 每 5s 一次 PING（RTT 采样），不占用 1s 节拍
        sys_info = self.client.sys_info or {}
        cpu0 = sys_info.get("cpu0")
        cpu1 = sys_info.get("cpu1")
        cpu0_txt = "-" if cpu0 is None else f"{cpu0}%"
        cpu1_txt = "-" if cpu1 is None else f"{cpu1}%"
        free, total = sys_info.get("ram_free"), sys_info.get("ram_total")
        ram_txt = "-"
        if isinstance(free, (int, float)) and isinstance(total, (int, float)) and total > 0:
            ram_txt = f"{round((total - free) * 100 / total)}%"   # 与 CPU 同格式：只显示百分比
        rtt = self.client.last_rtt_ms
        rtt_txt = "-" if rtt is None else f"{rtt} ms"
        txt = (f"{tr('perf_sensor')} {self._sensor_count} Hz   "
               f"{tr('perf_video')} {self._frame_count} fps   "
               f"{tr('perf_cpu0')} {cpu0_txt}   {tr('perf_cpu1')} {cpu1_txt}   "
               f"{tr('perf_ram')} {ram_txt}   {tr('perf_rtt')} {rtt_txt}")
        self.lbl_perf.setText(txt)
        self._sensor_count = 0
        self._frame_count = 0

    # ============================ 槽（主线程） ============================
    @Slot(bool)
    def _on_connect_clicked(self, _checked=False):
        host = self.host_edit.text().strip() or "192.168.4.1"
        self.connect_btn.setEnabled(False)          # 连接期间禁点防重入
        self.disconnect_btn.setEnabled(False)       # 连接期间不可断开（防竞态：此时 close() 是空操作）
        self.video_btn.setEnabled(False)
        self.conn_status.setText(tr("conn_connecting"))
        # 连接在后台线程执行：create_connection 最多阻塞 5s，绝不冻结 GUI
        threading.Thread(target=self._connect_worker, args=(host,), daemon=True,
                         name="gui-connect").start()

    def _connect_worker(self, host: str):
        ok, msg = False, host
        try:
            ok = self.client.connect(host, TCP_CMD_PORT)
        except ConnectionError as e:
            msg = str(e)
        self._sig_conn.emit(ok, msg)

    @Slot(bool)
    def _on_disconnect_clicked(self, _checked=False):
        """主动断开：close() 幂等且会唤醒所有阻塞线程；UI 复位由 on_disconnected 信号完成。"""
        self.video_btn.setEnabled(False)
        self.client.close()

    @Slot(bool)
    def _on_video_clicked(self, _checked=False):
        if not self.client.connected or not self.video_btn.isEnabled():
            return
        sent = self.client.video_off() if self._video_on else self.client.video_on()
        if sent:
            self._video_on = not self._video_on
            self.video_btn.setText(tr("btn_video_off" if self._video_on else "btn_video_on"))
        else:
            self._log(tr("log_send_fail"))

    @Slot(bool, str)
    def _on_connect_result(self, ok: bool, msg: str):
        self.connect_btn.setEnabled(True)
        self._video_on = False
        self.video_btn.setText(tr("btn_video_on"))
        self.video_btn.setEnabled(ok and self.client.connected)
        if ok and self.client.connected:   # 过期成功事件防护：断连先到则不入成功分支
            self.conn_status.setText(tr("conn_yes"))
            self._log("✓ " + tr("log_conn_ok") + msg)
            self.disconnect_btn.setEnabled(True)
            # 断开后配置回默认：连接成功即下发默认分辨率，与重置后的 UI 一致
            self.cam_res_combo.setCurrentIndex(1)          # QVGA
            if not self._send_json({"cmd": "CAM_RES", "res": "QVGA"}):
                self._log(tr("log_cam_res").format("QVGA") + " (send failed)")
            for a in self._algorithms:
                a.on_start(self.client)
        else:
            self.conn_status.setText(tr("conn_no"))
            self._log("✗ " + tr("log_conn_fail") + msg)

    @Slot()
    def _on_disconnected(self):
        self.conn_status.setText(tr("conn_no"))
        self.disconnect_btn.setEnabled(False)
        self.video_btn.setEnabled(False)
        self._video_on = False
        self.video_btn.setText(tr("btn_video_on"))
        self.video.clear()
        self.gamepad.set_state(GamepadState())   # 手柄复位为未连接
        self._log("✗ " + tr("log_disconn"))
        for a in self._algorithms:
            a.on_stop()

    @Slot(object)
    def _on_sensor(self, s: SensorData):
        self._sensor_count += 1
        def fmt_mm(v, ok):
            return f"{v} mm" if ok else f"{v} mm({tr('invalid')})"
        self.lbl_dist_f.setText(fmt_mm(s.dist_front_mm, s.dist_front_status == 0))
        self.lbl_dist_l.setText(fmt_mm(s.dist_left_mm, s.dist_left_status == 0))
        self.lbl_dist_r.setText(fmt_mm(s.dist_right_mm, s.dist_right_status == 0))
        for i in range(4):
            self.lbl_spd[i].setText(f"{s.speed_d100[i] / 100:+.1f} cm/s")
            self.lbl_cur[i].setText(f"{s.current_mA[i]} mA")
        self.lbl_imu.setText(
            f"quat=({s.quat_w:.2f},{s.quat_x:.2f},{s.quat_y:.2f},{s.quat_z:.2f}) "
            f"accel=({s.accel_x:.2f},{s.accel_y:.2f},{s.accel_z:.2f}) g")
        self.lbl_ack.setText(
            f"id={s.last_cmd_id} "
            + (tr("st_ok") if s.last_cmd_status == 0 else tr("st_err") + str(s.last_cmd_status)))
        for a in self._algorithms:
            a.on_sensor(s)

    @Slot(object)
    def _on_ack(self, ack: dict):
        """ESP32 命令回执（PROTOCOL.md §2.1 ACK）。"""
        cmd = ack.get("ack", "?")
        status = ack.get("status", -1)
        self._log(f"{tr('log_ack')} {cmd} → "
                  + (tr("st_ok") if status == 0 else f"{tr('st_err')}{status}"))
        self.lbl_ack.setText(f"{cmd} → "
                             + (tr("st_ok") if status == 0 else f"{tr('st_err')}{status}"))

    @Slot(object)
    def _on_gamepad(self, g: GamepadState):
        """手柄状态（50Hz）→ 2.5D 动画面板。"""
        self.gamepad.set_state(g)

    @Slot(int, bytes)
    def _on_bad_frame(self, frame_id: int, jpeg: bytes):
        """组装层拦截的坏帧 → 落盘取证（分析字节结构定位根因）。"""
        _save_bad_frame(jpeg, frame_id, "assembler")

    @Slot(int, bytes)
    def _on_frame(self, frame_id: int, jpeg: bytes):
        """真 fps：只统计能解码渲染的帧。SOI/EOI 已在组装层（FrameAssembler）
        拦截，此处由 QImage 解码做最终校验，坏帧不进视频/算法。"""
        img = QImage.fromData(jpeg, "JPEG")
        if img.isNull():
            _save_bad_frame(jpeg, frame_id, "gui_decode")
            return
        self._frame_count += 1
        self.video.show_image(frame_id, img)
        for a in self._algorithms:
            a.on_frame(frame_id, jpeg)

    @Slot(object)
    def _on_macro(self, m: dict):
        self._macro_state = m
        self._render_macro_state()
        state = m.get("state")
        if state == "done":
            self._log(tr("log_macro_done") + str(m))
        elif state == "aborted":
            self._log(tr("log_macro_abort") + str(m))

    @Slot(str)
    def _append_log(self, text: str):
        self.log_view.append(text)
        # 行数上限：超出自动裁剪最旧（滚动日志）
        doc = self.log_view.document()
        while doc.blockCount() > _MAX_LOG_BLOCKS:
            cur = QTextCursor(doc)
            cur.movePosition(QTextCursor.Start)
            cur.select(QTextCursor.BlockUnderCursor)
            cur.removeSelectedText()
            cur.deleteChar()

    def _log(self, text: str):
        self._append_log(text)

    @Slot()
    def _on_clear_console(self):
        """清空控制台日志。"""
        self.log_view.clear()
        self._log(tr("console_cleared"))

    # ============================ 发送 ============================
    def _send_json(self, obj) -> bool:
        ok = self.client.send_json(obj)
        self._log((("✓ " if ok else "✗ ") +
                   (tr("log_sent") if ok else tr("log_send_fail"))) + str(obj)[:80])
        return ok

    def _send_preset_macro(self):
        m = Macro(1)
        for i in range(4):
            m.act("MOTOR_CLOSED", motor=i, speed_d100=4000, duration_ms=0)
        m.wait("dist_front_mm", "<", 200, timeout_ms=10000)
        m.act("STOP_ALL", duration_ms=0)
        self._send_json(m.to_dict())

    def _send_pid(self):
        self._send_json({"cmd": "PID", "kp": self.kp_spin.value(), "ki": self.ki_spin.value()})

    def _send_motor(self, motor: int):
        _lbl, sp, _btn = self.motor_spins[motor]
        self._send_json({"cmd": "MOTOR_CLOSED", "motor": motor,
                         "speed_d100": sp.value(), "duration_ms": 0})

    def _send_motor_all(self):
        v = self.motor_spins[0][1].value()
        for i in range(4):
            self.motor_spins[i][1].setValue(v)
            self._send_motor(i)

    def _send_custom_macro(self):
        try:
            obj = _json.loads(self.macro_edit.toPlainText().strip())
        except ValueError as e:
            self._log("✗ " + tr("log_json_fail") + str(e))
            return
        if isinstance(obj, list):
            obj = {"type": "macro", "id": 1, "steps": obj}
        self._send_json(obj)
