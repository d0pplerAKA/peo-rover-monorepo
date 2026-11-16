"""
app.gui — 主窗口 v2（2026-11-16 组员编程模型定案后重构）

Tab1 手动监控：连接 + 视频 + 参数仪表盘（只读）+ 急停。
Tab2 组员 IDE：脚本编辑/运行/停止 + shell REPL + 视频窗 + 输出控制台（含全局日志）。
    - 脚本跑在独立线程（阻塞式脚本不卡 GUI）；停止 = 协作式（peo.delay 检查标志）。
    - 切到 Tab2 自动断开 GUI 连接（单客户端规范，PROTOCOL §2.3）。
    - shell 与运行中的脚本互斥（单控制源原则）。

线程模型：RoverClient 接收线程回调 → 跨线程 emit Qt signal → 主线程槽更新 UI。
双语：所有界面文本经 app.i18n.tr() 取；语言切换时 _retranslate() 刷新。
"""
import code
import contextlib
import io
import os
import queue
import rlcompleter
import sys
import threading
import time
from collections import deque

from PySide6.QtCore import Qt, Signal, Slot, QTimer, QSize, QRect
from PySide6.QtGui import (QImage, QShortcut, QKeySequence, QTextCursor, QFont,
                           QPainter, QColor, QTextFormat)
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QFormLayout,
    QLabel, QLineEdit, QPushButton, QGroupBox, QTextEdit, QSplitter,
    QPlainTextEdit, QComboBox, QApplication, QTabWidget, QFileDialog,
)

from rover_comm.client import RoverClient
from rover_comm.control import ControlLock
from rover_comm.protocol import SensorData, TCP_CMD_PORT
from app.i18n import tr, set_language, current_language
from app.theme import apply_theme
from app.video import VideoWidget


def _enqueue_output(target, text):
    # Bound both queued item count and each item's bytes/characters.
    for offset in range(0, len(text), 8192):
        chunk = text[offset:offset + 8192]
        try:
            target.put_nowait(chunk)
        except queue.Full:
            try:
                target.get_nowait()
            except queue.Empty:
                pass
            try:
                target.put_nowait(chunk)
            except queue.Full:
                pass


class _StreamRelay(io.TextIOBase):
    """流式输出转发：脚本线程每次 write 立即入队（主线程 50ms drain 显示）。

    替代 io.StringIO 缓冲——脚本运行中 print 的内容实时进控制台，
    而不是结束后一次性刷出。
    libjpeg 噪音行（解码警告）过滤丢弃——不进控制台、不刷屏幕
    （C 层警告另有 fd 2 → stderr.log 归档）。
    """

    _LIBJPEG_NOISE = (
        "Corrupt JPEG data",
        "Empty JPEG image",
        "Unsupported marker type",
        "Bogus marker",
        "premature end of data segment",
        "extraneous bytes before marker",
        "Invalid JPEG file structure",
        "missing SOS marker",
        "JPEG datastream contains no image",
    )

    def __init__(self, q: "queue.Queue"):
        super().__init__()
        self._q = q

    def write(self, s: str) -> int:
        if s and not any(pat in s for pat in self._LIBJPEG_NOISE):
            _enqueue_output(self._q, s)
        return len(s)

    def flush(self) -> None:
        pass


class LineNumberArea(QWidget):
    """代码编辑器左侧行号栏（Qt 官方 LineNumberArea 移植）。"""

    def __init__(self, editor: "CodeEditor"):
        super().__init__(editor)
        self._editor = editor

    def sizeHint(self):
        return QSize(self._editor.line_number_area_width(), 0)

    def paintEvent(self, event):
        self._editor.line_number_area_paint(event)


class CodeEditor(QPlainTextEdit):
    """带行号 + 当前行高亮的纯文本编辑器（组员 IDE 用）。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        # Tab 键缩进 = 4 空格（Python 惯例，默认 8 太宽）
        self.setTabStopDistance(4 * self.fontMetrics().horizontalAdvance(" "))
        self._line_area = LineNumberArea(self)
        self.blockCountChanged.connect(self._update_area_width)
        self.updateRequest.connect(self._update_area)
        self.cursorPositionChanged.connect(self._highlight_current)
        self._update_area_width(0)
        self._highlight_current()

    def setFont(self, font):
        """换字体后重算 Tab 宽度，任何字体下都保持 4 空格缩进。"""
        super().setFont(font)
        self.setTabStopDistance(4 * self.fontMetrics().horizontalAdvance(" "))

    def line_number_area_width(self) -> int:
        digits = max(3, len(str(max(1, self.blockCount()))))
        return 14 + self.fontMetrics().horizontalAdvance("9") * digits

    def _update_area_width(self, _=None):
        self.setViewportMargins(self.line_number_area_width(), 0, 0, 0)

    def _update_area(self, rect, dy):
        if dy:
            self._line_area.scroll(0, dy)
        else:
            self._line_area.update(0, rect.y(),
                                   self._line_area.width(), rect.height())
        if rect.contains(self.viewport().rect()):
            self._update_area_width(0)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        cr = self.contentsRect()
        self._line_area.setGeometry(
            QRect(cr.left(), cr.top(), self.line_number_area_width(),
                  cr.height()))

    def line_number_area_paint(self, event):
        painter = QPainter(self._line_area)
        painter.fillRect(event.rect(), QColor("#14161b"))
        block = self.firstVisibleBlock()
        number = block.blockNumber()
        top = round(self.blockBoundingGeometry(block)
                    .translated(self.contentOffset()).top())
        bottom = top + round(self.blockBoundingRect(block).height())
        fm = self.fontMetrics()
        while block.isValid() and top <= event.rect().bottom():
            if block.isVisible() and bottom >= event.rect().top():
                painter.setPen(QColor("#545b68"))
                painter.drawText(0, top, self._line_area.width() - 8,
                                 fm.height(), Qt.AlignRight, str(number + 1))
            block = block.next()
            top = bottom
            bottom = top + round(self.blockBoundingRect(block).height())
            number += 1

    def _highlight_current(self):
        """当前行整行高亮（写代码时的光标行指示）。"""
        sel = QTextEdit.ExtraSelection()
        sel.format.setBackground(QColor("#242c3a"))   # 明显亮于编辑器底色
        sel.format.setProperty(QTextFormat.FullWidthSelection, True)
        sel.cursor = self.textCursor()
        sel.cursor.clearSelection()
        self.setExtraSelections([sel])

_MAX_LOG_BLOCKS = 500      # 日志行数上限：超出自动覆盖最旧
_PERF_INTERVAL_MS = 1000   # 性能计数刷新周期
_IDE_VIDEO_MS = 33         # Main-thread latest-frame polling; bounded rendering cadence
_IDE_SHOW_MIN_S = 0.030    # Tab2 显示最小间隔：均匀节奏（24fps=42ms 帧间隔不丢帧）

_CAM_RESOLUTIONS = (("QQVGA (160x120)", "QQVGA"),
                    ("QVGA (320x240)", "QVGA"),
                    ("VGA (640x480)", "VGA"),
                    ("SVGA (800x600)", "SVGA"))


class _PeoConsole(code.InteractiveConsole):
    """真 REPL：标准库 InteractiveConsole，输出转发到 GUI 控制台。

    提供：多行续行（... 提示）、表达式回显、变量会话持久、异常捕获。
    """

    def __init__(self, env: dict, write_cb):
        super().__init__(env)
        self._write_cb = write_cb

    def write(self, data: str):
        self._write_cb(data)

    def flush(self):
        pass

    def runcode(self, code_):
        """把 displayhook（表达式回显）/ print / stderr 全部收敛到 write。

        默认实现写进程 sys.stdout（会打到终端而非 GUI 控制台）；此处重定向。
        shell 与脚本互斥运行（脚本运行中 shell 锁住），全局 stdout 无并发写。"""
        old_out, old_err = sys.stdout, sys.stderr
        old_hook = sys.displayhook
        sys.stdout = sys.stderr = self
        sys.displayhook = (lambda obj: self.write(repr(obj) + "\n")
                           if obj is not None else None)
        try:
            super().runcode(code_)
        finally:
            sys.stdout, sys.stderr = old_out, old_err
            sys.displayhook = old_hook


class ShellLineEdit(QLineEdit):
    """shell 输入行：↑/↓ 历史、Tab 补全（rlcompleter）。"""

    def __init__(self, window, parent=None):
        super().__init__(parent)
        self._win = window

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_Up:
            self._win._shell_hist_prev()
            return
        if key == Qt.Key_Down:
            self._win._shell_hist_next()
            return
        if key == Qt.Key_Tab:
            self._win._shell_complete()
            return
        super().keyPressEvent(event)


class MainWindow(QMainWindow):
    # 跨线程信号（RoverClient 接收线程 emit → 主线程槽，Qt 自动 queued）
    _sig_sensor = Signal(object)      # SensorData
    _sig_frame = Signal(int, bytes)   # frame_id, jpeg
    _sig_ack = Signal(object)         # dict {"ack":cmd,"status":n}
    _sig_log = Signal(str)
    _sig_disconn = Signal()
    _sig_conn = Signal(bool, str)       # (成功?, host 或错误信息)
    # IDE 输出（脚本/shell 线程 → 队列 → 主线程 QTimer drain 写控制台）
    _sig_shell_out = Signal(str)
    _sig_ide_status = Signal(str)
    _sig_script_finished = Signal()     # 脚本线程结束 → 主线程恢复按钮状态
    _sig_ide_frame = Signal()          # Tab2 视频帧到达通知（无参：主线程取最新帧）

    def __init__(self):
        super().__init__()
        self.resize(1280, 800)        # 初始窗口大小（三端工作台合理尺寸）
        self._sensor_count = 0        # 性能计数（1s 窗口）
        self._perf_tick = 0           # UI performance sampling only; client owns heartbeat
        self._frame_count = 0
        self._video_on = False

        self._control_lease = ControlLock()
        self._closing = False
        self._connecting = False
        self._connect_cancel = threading.Event()
        self._frame_mailbox_lock = threading.Lock()
        self._frame_mailbox = None
        self._frame_notification = False
        self._shell_busy = threading.Event()
        self.client = RoverClient()
        self.client.on_sensor = self._sig_sensor.emit          # 接收线程
        self.client.on_frame = self._queue_frame
        self.client.on_ack = self._sig_ack.emit
        self.client.on_disconnected = self._client_disconnected

        self._sig_sensor.connect(self._on_sensor)
        self._sig_frame.connect(self._on_frame)
        self._sig_ack.connect(self._on_ack)
        self._sig_log.connect(self._append_log)
        self._sig_conn.connect(self._on_connect_result)
        self._sig_disconn.connect(self._on_disconnected)
        self._sig_shell_out.connect(lambda t: self._append_ide(self.script_output, t))
        self._sig_ide_status.connect(self._set_ide_status)
        self._sig_script_finished.connect(self._on_script_finished)
        self._sig_ide_frame.connect(self._on_ide_frame)

        # IDE 运行时状态
        self._script_thread = None
        self._shell_thread = None
        self._script_running = False
        self._bad_seq = -1             # Tab2 已知坏帧 seq（只解码一次）
        self._shown_seq = -1           # Tab2 已显示帧 seq（积压重复通知去重）
        self._last_ide_show = None     # Tab2 上次显示时刻（节流用）
        self._bad_dump_count = 0       # 坏帧取证落盘计数（上限 8）
        self._sdk_sensor_base = 0      # Tab2 状态栏：SDK 遥测计数基线
        self._sdk_frame_base = 0
        # shell REPL 状态
        self._shell_q = queue.Queue(maxsize=4)      # 主线程 → shell 线程的输入行
        self._shell_out_q = queue.Queue(maxsize=256)  # shell 线程 → 主线程的输出（QTimer drain）
        self._shell_more = False           # 续行模式（提示符 ...，主线程 UI 状态）
        self._shell_more_flag = False      # shell 线程写入的续行标志（drain 同步到 UI）
        self._shell_history = deque(maxlen=200)
        self._shell_hist_idx = None
        self._shell_completer = None       # rlcompleter.Completer（shell 线程填充）
        self._shell_started = False

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

        # Tab2 latest-frame mailbox polling (~30Hz, capture target is 24fps)
        self._ide_video_timer = QTimer(self)
        self._ide_video_timer.timeout.connect(self._pull_ide_video)
        self._ide_video_timer.start(_IDE_VIDEO_MS)

        # shell 输出 drain（shell 线程 → 队列 → 主线程写控制台，50ms）
        self._shell_drain_timer = QTimer(self)
        self._shell_drain_timer.timeout.connect(self._drain_shell_out)
        self._shell_drain_timer.start(50)

        self._log(tr("log_ready"))

    # ============================ UI 构建 ============================
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 6, 8, 4)
        root.setSpacing(4)

        # 顶部栏：应用标题 + 双语切换按钮（右侧固定，不挤压 Tab 标题）
        topbar = QHBoxLayout()
        self.lbl_app = QLabel()
        self.lbl_app.setStyleSheet("color: #9aa0ac; font-size: 12px;")
        topbar.addWidget(self.lbl_app)
        topbar.addStretch(1)
        self.lang_btn = QPushButton()
        self.lang_btn.setMinimumWidth(64)      # 自适应内容，避免"中文"被截半
        self.lang_btn.setMaximumWidth(80)
        self.lang_btn.setToolTip("Switch language / 切换语言")
        self.lang_btn.clicked.connect(self._on_lang_toggle)
        topbar.addWidget(self.lang_btn)
        root.addLayout(topbar)

        self.tabs = QTabWidget()
        self.tabs.currentChanged.connect(self._on_tab_changed)
        root.addWidget(self.tabs)
        self.tabs.addTab(self._build_tab1(), "")   # 文本 _retranslate 填
        self.tabs.addTab(self._build_tab2(), "")

        # 状态栏：性能/帧率（右留边距防贴边截断）
        self.lbl_perf = QLabel()
        self.lbl_perf.setContentsMargins(0, 0, 12, 0)
        self.statusBar().addPermanentWidget(self.lbl_perf)

    # ---------------- Tab1：手动监控 ----------------

    def _build_tab1(self):
        w = QWidget()
        lay = QHBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        splitter = QSplitter(Qt.Horizontal)
        lay.addWidget(splitter)

        # 左列：分辨率 + 视频
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
        lv.addWidget(self.video, 1)
        splitter.addWidget(left)

        # 右列：连接 + 仪表盘（只读）+ 急停
        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(0, 0, 0, 0)
        rv.addWidget(self._build_conn_group())
        rv.addWidget(self._build_sensor_group())
        # 急停（安全设施：Tab1 唯一命令按钮）
        self.btn_stop = QPushButton()
        self.btn_stop.setMinimumHeight(44)
        self.btn_stop.setObjectName("stopBtn")
        self.btn_stop.clicked.connect(lambda: self._send_json({"cmd": "STOP_ALL"}))
        rv.addWidget(self.btn_stop)
        rv.addStretch(1)
        splitter.addWidget(right)
        splitter.setSizes([640, 360])
        return w

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
        self.video_btn = QPushButton()
        self.video_btn.clicked.connect(self._on_video_clicked)
        row.addWidget(self.host_edit, 1)
        row.addWidget(self.connect_btn)
        row.addWidget(self.disconnect_btn)
        row.addWidget(self.video_btn)
        f.addRow("", row)

        self.lbl_ip = QLabel()
        self.lbl_status = QLabel()
        self.conn_status = QLabel()
        st_row = QHBoxLayout()
        st_row.addWidget(self.lbl_ip)
        st_row.addWidget(self.conn_status)
        st_row.addStretch(1)
        f.addRow(self.lbl_status, st_row)
        self.disconnect_btn.setEnabled(False)
        self.video_btn.setEnabled(False)
        return g

    def _build_sensor_group(self):
        g = QGroupBox()
        self.grp_sensor = g
        grid = QGridLayout(g)
        grid.setContentsMargins(6, 2, 6, 4)
        grid.setVerticalSpacing(3)

        self.lbl_dist_f = QLabel("—")
        self.lbl_dist_l = QLabel("—")
        self.lbl_dist_r = QLabel("—")
        self.lbl_spd = [QLabel("—") for _ in range(4)]
        self.lbl_cur = [QLabel("—") for _ in range(4)]
        self.lbl_imu = QLabel("—")
        self.lbl_ack = QLabel("—")

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
            grid.addWidget(self.lbl_spd[i], r + 1, i * 2)
            grid.addWidget(self._cur_title[i], r, i * 2 + 1)
            grid.addWidget(self.lbl_cur[i], r + 1, i * 2 + 1)
        r += 2
        t = QLabel(); grid.addWidget(t, r, 0); grid.addWidget(self.lbl_imu, r, 1, 1, 5)
        self._sensor_labels.append((t, "imu"))
        r += 1
        # ACK 回显行
        t = QLabel(); grid.addWidget(t, r, 0); grid.addWidget(self.lbl_ack, r, 1, 1, 5)
        self._sensor_labels.append((t, "cmd_ack"))
        return g

    # ---------------- Tab2：组员 IDE ----------------

    def _build_tab2(self):
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)

        # 工具行：打开 / 运行 / 停止 + 状态
        toolbar = QHBoxLayout()
        self.btn_open = QPushButton()
        self.btn_open.clicked.connect(self._on_open_script)
        self.btn_run = QPushButton()
        self.btn_run.clicked.connect(self._on_run_script)
        self.btn_stop_script = QPushButton()
        self.btn_stop_script.clicked.connect(self._on_stop_script)
        self.btn_stop_script.setEnabled(False)
        self.lbl_ide_status = QLabel()
        toolbar.addWidget(self.btn_open)
        toolbar.addWidget(self.btn_run)
        toolbar.addWidget(self.btn_stop_script)
        toolbar.addWidget(self.lbl_ide_status)
        toolbar.addStretch(1)
        lay.addLayout(toolbar)

        # 上：脚本编辑区（带行号 + 当前行高亮）
        self.ide_edit = CodeEditor()
        self.ide_edit.setFont(QFont("Consolas", 10))
        lay.addWidget(self.ide_edit, 3)

        # 下：左视频窗 / 右 shell + 控制台（含全局日志）
        lower = QSplitter(Qt.Horizontal)
        self.ide_video = VideoWidget()
        lower.addWidget(self.ide_video)

        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(0, 0, 0, 0)
        # 统一控制台（shell 问答 + 脚本 stdout + 系统日志混流）+ 清空按钮
        console_row = QHBoxLayout()
        self.lbl_script_console = QLabel()
        console_row.addWidget(self.lbl_script_console)
        console_row.addStretch(1)
        self.btn_console_clear = QPushButton()
        self.btn_console_clear.setFixedWidth(64)
        self.btn_console_clear.clicked.connect(self._on_clear_console)
        console_row.addWidget(self.btn_console_clear)
        rv.addLayout(console_row)
        self.script_output = QPlainTextEdit()
        self.script_output.setReadOnly(True)
        rv.addWidget(self.script_output, 3)
        # shell 输入行（回车执行；输入回显与结果进统一控制台）
        shell_row = QHBoxLayout()
        self.lbl_shell_prompt = QLabel(">>> ")
        self.lbl_shell_prompt.setFont(QFont("Consolas", 10))
        self.lbl_shell_prompt.setFixedWidth(40)
        shell_row.addWidget(self.lbl_shell_prompt)
        self.shell_input = ShellLineEdit(self)
        self.shell_input.returnPressed.connect(self._on_shell_enter)
        shell_row.addWidget(self.shell_input, 1)
        rv.addLayout(shell_row)
        lower.addWidget(right)
        lower.setSizes([560, 440])
        lay.addWidget(lower, 2)
        return w

    def _append_ide(self, widget: QPlainTextEdit, text: str):
        widget.appendPlainText(text.rstrip("\n"))

    def _set_ide_status(self, text: str):
        self.lbl_ide_status.setText(text)

    # ---------------- 文本（双语刷新） ----------------

    def _retranslate(self):
        self.setWindowTitle(tr("app_title"))
        self.tabs.setTabText(0, tr("tab1_manual"))
        self.tabs.setTabText(1, tr("tab2_ide"))
        self.grp_conn.setTitle(tr("grp_conn"))
        self.lbl_ip.setText(tr("ip"))
        self.connect_btn.setText(tr("btn_connect"))
        self.disconnect_btn.setText(tr("btn_disconnect"))
        self.video_btn.setText(tr("btn_video_off") if self.client.connected and self._video_on else tr("btn_video_on"))
        self.lbl_status.setText(tr("status"))
        self.conn_status.setText(tr("conn_yes") if self.client.connected else tr("conn_no"))
        self.lbl_cam_res.setText(tr("cam_res"))
        self.btn_stop.setText(tr("btn_stop"))
        # 顶部语言按钮：显示目标语言（当前 zh → "EN"，当前 en → "中文"）
        self.lang_btn.setText("EN" if current_language() == "zh" else "中文")
        self.lbl_app.setText(tr("app_brand"))

        self.grp_sensor.setTitle(tr("grp_sensor"))
        for t, key in self._sensor_labels:
            t.setText(tr(key))
        for i in range(4):
            self._spd_title[i].setText(tr("spd_col", i + 1))
            self._cur_title[i].setText(tr("cur_col", i + 1))

        # Tab2 文本
        self.btn_open.setText(tr("btn_open"))
        self.btn_run.setText(tr("btn_run"))
        self.btn_stop_script.setText(tr("btn_stop_script"))
        self.lbl_script_console.setText(tr("grp_console"))
        self.ide_edit.setPlaceholderText(tr("ide_placeholder"))
        self.shell_input.setPlaceholderText(tr("shell_placeholder"))
        self.btn_console_clear.setText(tr("console_clear"))
        self.video.update()    # 视频占位文本按新语言重绘
        self.ide_video.update()

    # ---------------- Tab 切换 ----------------

    def _on_tab_changed(self, idx: int):
        if idx == 1 and (self.client.connected or self._connecting):
            self._connect_cancel.set()
            # 切到 IDE：自动断开 GUI 连接（单客户端规范），释放控制权给组员脚本
            self.client.close()
            self._log(tr("log_tab2_disconn"))
            self._set_ide_status(tr("ide_ready"))

    # ---------------- IDE 运行 ----------------

    def _ensure_sdk_path(self):
        """把 sdk/ 加进 sys.path（组员脚本与 GUI 本体的 peo import 都依赖）。"""
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        sdk = os.path.join(root, "sdk")
        if sdk not in sys.path:
            sys.path.insert(0, sdk)
        return sdk

    def _peo_env(self):
        """构造脚本执行环境：注入 peo 模块（sdk 路径）——两种写法都支持：
        `peo.drive_all(30)`（import peo 风格）与 `drive_all(30)`（from peo import * 风格）。"""
        self._ensure_sdk_path()
        import peo
        env = {"__name__": "__main__", "peo": peo}
        env.update({k: getattr(peo, k) for k in peo.__all__})
        return env

    def _run_code_in_thread(self, code: str, out_q: queue.Queue):
        """在独立线程 exec 脚本代码，stdout/stderr 流式重定向到队列（每次 write 即入队）。"""
        env = self._peo_env()
        relay = _StreamRelay(out_q)
        try:
            with contextlib.redirect_stdout(relay), contextlib.redirect_stderr(relay):
                try:
                    exec(compile(code, "<script>", "exec"), env, env)
                except SystemExit:
                    pass
                except Exception as e:
                    # 异常也进输出窗（组员在 IDE 控制台看到，不进终端）
                    relay.write(f"Traceback: {type(e).__name__}: {e}\n")
        finally:
            relay.flush()

    def _on_open_script(self):
        path, _ = QFileDialog.getOpenFileName(
            self, tr("dlg_open"), "",
            "Python (*.py);;All Files (*)")
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                self.ide_edit.setPlainText(f.read())
            self._set_ide_status(os.path.basename(path))
        except OSError as e:
            self._log("✗ " + tr("log_open_fail") + str(e))

    def _on_run_script(self):
        if self._script_running:
            return
        if self._shell_busy.is_set() or self._shell_more_flag:
            self._append_ide(self.script_output, tr("shell_locked"))
            return
        code = self.ide_edit.toPlainText().strip()
        if not code:
            self._set_ide_status(tr("ide_empty"))
            return
        if self.client.connected or self._connecting:
            self._connect_cancel.set()
            self.client.close()   # 保险：脚本连接前 GUI 必须先断开

        self._script_running = True
        self.btn_run.setEnabled(False)
        self.btn_stop_script.setEnabled(True)
        self._set_ide_status(tr("ide_running"))
        self.script_output.clear()
        # 排空上次运行的残留输出（流式队列）
        q = getattr(self, "_script_out_q", None)
        if q is not None:
            while True:
                try:
                    q.get_nowait()
                except queue.Empty:
                    break

        self._ensure_sdk_path()
        import peo as _peo
        _peo._clear_stop_flag()

        self._script_out_q = queue.Queue(maxsize=256)   # 脚本线程流式输出 → 主线程 50ms drain

        def _runner():
            try:
                self._run_code_in_thread(code, self._script_out_q)
            finally:
                # 脚本结束（正常/停止/异常）→ 断开连接 + 释放单实例锁
                # （单客户端规范：脚本不占着连接，下一脚本/GUI 可立即连）
                self._ensure_sdk_path()
                import peo as _peo
                try:
                    _peo._bridge.disconnect()
                except Exception:
                    pass
                self._sig_ide_status.emit(tr("ide_done"))
                self._sig_script_finished.emit()   # 按钮状态回主线程恢复

        self._script_thread = threading.Thread(target=_runner, daemon=True,
                                               name="peo-script")
        self._script_thread.start()

    @Slot()
    def _on_script_finished(self):
        """脚本线程结束（Qt 控件限主线程操作，按钮状态在此恢复）。"""
        self._script_running = False
        self.btn_run.setEnabled(True)
        self.btn_stop_script.setEnabled(False)

    def _on_stop_script(self):
        self._ensure_sdk_path()
        import peo as _peo
        _peo._request_stop()      # 立即排入 STOP；后续 SDK 动作/等待抛 ScriptStopped
        self._set_ide_status(tr("ide_stopping"))

    # ---------------- shell REPL（真 REPL：InteractiveConsole） ----------------

    def _start_shell(self):
        """常驻 shell 线程：InteractiveConsole 有状态（变量持久），必须同一线程。"""
        if self._shell_started:
            return
        self._shell_started = True
        env = self._peo_env()

        def _write(data: str):
            _enqueue_output(self._shell_out_q, data)   # 线程安全队列；主线程 QTimer drain 写控件

        def _loop():
            console = _PeoConsole(env, _write)
            self._shell_completer = rlcompleter.Completer(env)
            banner = (f"Python {sys.version} on {sys.platform}\n"
                      'Type "help", "copyright", "credits" or "license" '
                      "for more information.\n"
                      "(peo SDK preloaded — try: peo.vl53.front_mm)\n")
            _write(banner)
            while True:
                line = self._shell_q.get()
                if line is None:
                    break
                # 输入回显（带当前提示符）
                _write(("... " if self._shell_more else ">>> ") + line + "\n")
                try:
                    more = console.push(line)   # 真 REPL 语义
                except SystemExit:
                    more = False
                finally:
                    self._shell_more_flag = locals().get("more", False)
                    self._shell_busy.clear()

        self._shell_thread = threading.Thread(target=_loop, daemon=True,
                                              name="peo-shell")
        self._shell_thread.start()

    def _drain_shell_out(self):
        """主线程周期 drain shell 输出队列 → 统一控制台（Qt 控件限主线程操作）。
        同时把 shell 线程的续行标志同步到提示符标签。"""
        for _ in range(64):
            try:
                data = self._shell_out_q.get_nowait()
            except queue.Empty:
                break
            self._append_ide(self.script_output, data)
        # 脚本流式输出也走同一 drain（脚本运行中 print 实时显示）
        q = getattr(self, "_script_out_q", None)
        if q is not None:
            chunks = []
            for _ in range(64):
                try:
                    chunks.append(q.get_nowait())
                except queue.Empty:
                    break
            if chunks:
                self._append_ide(self.script_output, "".join(chunks))
        if self._shell_more_flag != self._shell_more:
            self._shell_more = self._shell_more_flag
            self.lbl_shell_prompt.setText("... " if self._shell_more else ">>> ")

    def _on_shell_enter(self):
        if self._script_running or self._shell_busy.is_set():
            self._append_ide(self.script_output, tr("shell_locked"))
            return
        text = self.shell_input.text()
        self.shell_input.clear()
        if not text.strip() and not self._shell_more_flag:
            return   # 非续行模式空输入忽略；续行模式空行 = 结束多行块，放行
        self._start_shell()
        # 历史：追加 + 重置游标（主线程维护，线程安全足够——deque 追加与 get 无并发写）
        self._shell_history.append(text)
        self._shell_hist_idx = None
        self._ensure_sdk_path()
        import peo
        peo._clear_stop_flag()
        self._shell_busy.set()
        self._shell_q.put(text)

    def _shell_hist_prev(self):
        if not self._shell_history:
            return
        if self._shell_hist_idx is None:
            self._shell_hist_idx = len(self._shell_history) - 1
        else:
            self._shell_hist_idx = max(0, self._shell_hist_idx - 1)
        self.shell_input.setText(self._shell_history[self._shell_hist_idx])

    def _shell_hist_next(self):
        if self._shell_hist_idx is None:
            return
        self._shell_hist_idx += 1
        if self._shell_hist_idx >= len(self._shell_history):
            self._shell_hist_idx = None
            self.shell_input.clear()
            return
        self.shell_input.setText(self._shell_history[self._shell_hist_idx])

    def _shell_complete(self):
        """Tab 补全：唯一匹配 → 补全；多个匹配 → 列出候选。"""
        if self._shell_completer is None:
            return
        text = self.shell_input.text()
        pos = self.shell_input.cursorPosition()
        if not text or pos == 0:
            return
        # 取光标前的"标识符.前缀"词
        i = pos - 1
        while i >= 0 and (text[i].isalnum() or text[i] in "_."):
            i -= 1
        prefix = text[i + 1:pos]
        if not prefix:
            return
        try:
            matches = []
            for state in range(100):
                match = self._shell_completer.complete(prefix, state)
                if match is None:
                    break
                if match != prefix:
                    matches.append(match)
        except Exception:
            return
        if not matches:
            return
        if len(matches) == 1:
            self.shell_input.setText(text[:i + 1] + matches[0] + text[pos:])
            self.shell_input.setCursorPosition(i + 1 + len(matches[0]))
        else:
            self._append_ide(self.script_output,
                             "    " + "  ".join(matches[:20]))

    @Slot()
    def _on_ide_frame(self):
        """主线程定时读取最新帧，跳过已显示/已知坏帧，限制解码频率。"""
        if self._last_ide_show is not None and \
                time.monotonic() - self._last_ide_show < _IDE_SHOW_MIN_S / 1.0:
            return                            # 节流：等下个通知
        try:
            self._ensure_sdk_path()
            import peo as _peo
            jpeg, seq = _peo.cam.read_raw(timeout_ms=0)
        except Exception:
            return
        if jpeg is None:
            if self.ide_video._frame_id != -1:
                self.ide_video.clear()      # 断连/off：回占位符
            return
        if seq == self._shown_seq or seq == self._bad_seq:
            return                            # 已显示 / 已知坏帧
        img = QImage.fromData(jpeg, "JPEG")
        if img.isNull():
            # QImage 严格模式拒绝 → cv2 宽容兜底：ESP32 偶发输出半截帧
            # （熵数据写到一半但 EOI 齐全），cv2 能解出近乎完整的图像。
            img = self._cv2_decode_fallback(jpeg)
        if img is None or img.isNull():
            self._bad_seq = seq               # 坏帧记 seq，只报一次
            self._dump_bad_frame(jpeg, seq)   # 取证落盘（字节级分析用）
            return                            # 保持上一帧（对齐 Tab1：坏帧无感）
        self._last_ide_show = time.monotonic()
        self._shown_seq = seq
        self.ide_video.show_image(seq, img)

    @staticmethod
    def _cv2_decode_fallback(jpeg: bytes):
        """cv2 宽容解码兜底：截断帧（premature end）仍能出完整图。失败返回 None。

        解码期间重定向 stderr：libjpeg 的 premature end 警告吞掉——
        Python 层警告进 sink 丢弃，C 层警告走 fd 2 归档 stderr.log，
        任何可见输出（控制台/屏幕）都不再出现。
        """
        try:
            import numpy as np
            import cv2
            import contextlib
            import io
            sink = io.StringIO()
            with contextlib.redirect_stderr(sink):
                bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8),
                                   cv2.IMREAD_COLOR)
            if bgr is None:
                return None
            h, w = bgr.shape[:2]
            return QImage(bgr.data, w, h, bgr.strides[0],
                          QImage.Format_BGR888).copy()
        except Exception:
            return None

    def _dump_bad_frame(self, jpeg: bytes, seq: int) -> None:
        """坏帧取证落盘（最多 8 个，最旧自动删）——字节级根因分析用。"""
        if getattr(self, "_bad_dump_count", 0) >= 8:
            return
        try:
            d = os.path.join(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))), "bad_frames")
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, f"f{seq}_{int(time.time() % 100000)}.jpg"),
                      "wb") as f:
                f.write(jpeg)
            self._bad_dump_count = getattr(self, "_bad_dump_count", 0) + 1
        except OSError:
            pass

    def _pull_ide_video(self):
        """Bounded 30Hz polling: render only the latest SDK frame, no Qt backlog."""
        try:
            self._ensure_sdk_path()
            import peo
        except ImportError:
            return
        self.btn_stop_script.setEnabled(self._script_running or self._shell_busy.is_set()
                                        or peo._bridge.client.connected)
        if not peo.cam._on:
            self.ide_video.clear()
            self._shown_seq = self._bad_seq = -1
            return
        self._on_ide_frame()

    # ---------------- 槽（主线程） ----------------

    def _on_shortcut_stop(self):
        """空格急停；文本框聚焦时不触发。"""
        fw = QApplication.focusWidget()
        if isinstance(fw, (QLineEdit, QPlainTextEdit, QTextEdit)):
            return
        if self.tabs.currentIndex() == 1:
            self._on_stop_script()
            return
        if self._send_json({"cmd": "STOP_ALL"}):
            self._log(tr("log_sc_stop"))

    def _on_cam_res_changed(self, idx: int):
        """相机分辨率热切换：{"cmd":"CAM_RES","res":...}（ESP32-S3 本地命令）。"""
        if not 0 <= idx < len(_CAM_RESOLUTIONS):
            return
        disp, res = _CAM_RESOLUTIONS[idx]
        if self._send_json({"cmd": "CAM_RES", "res": res}):
            self._log(tr("log_cam_res").format(disp))

    @Slot(bool)
    def _on_connect_clicked(self, _checked=False):
        self._ensure_sdk_path()
        import peo
        if self._script_running or self._shell_busy.is_set() or peo._bridge.client.connected:
            self._log(tr("shell_locked"))
            return
        if self._connecting or self.client.connected:
            return
        cancel = self._connect_cancel = threading.Event()
        self._connecting = True
        host = self.host_edit.text().strip() or "192.168.4.1"
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(False)
        self.video_btn.setEnabled(False)
        self.conn_status.setText(tr("conn_connecting"))

        def _worker():
            try:
                self._control_lease.acquire()
                if cancel.is_set():
                    self._control_lease.release()
                    self._sig_conn.emit(False, "Connection cancelled")
                    return
                self.client.connect(host, TCP_CMD_PORT, TCP_CMD_PORT + 1)
                self.client.wait_ready(1.0)
                if cancel.is_set():
                    self.client.close()
                self._sig_conn.emit(self.client.connected, host)
            except (OSError, ConnectionError, RuntimeError) as e:
                self._control_lease.release()
                self._sig_conn.emit(False, str(e))

        threading.Thread(target=_worker, daemon=True, name="peo-connect").start()

    @Slot(bool, str)
    def _on_connect_result(self, ok: bool, msg: str):
        self._connecting = False
        if self._closing:
            self.client.close()
            return
        self.connect_btn.setEnabled(True)
        if ok and self.client.connected and self.tabs.currentIndex() == 0:
            self.connect_btn.setEnabled(False)
            self.conn_status.setText(tr("conn_yes"))
            self.disconnect_btn.setEnabled(True)
            self.video_btn.setEnabled(True)
            self._log("✓ " + tr("log_conn_ok") + msg)
            # 断开后配置回默认：连接成功即下发默认分辨率，与重置后的 UI 一致
            self.cam_res_combo.setCurrentIndex(1)          # QVGA
            if not self._send_json({"cmd": "CAM_RES", "res": "QVGA"}):
                self._log(tr("log_cam_res").format("QVGA") + " (send failed)")
        else:
            if self.client.connected:
                self.client.close()
            self.conn_status.setText(tr("conn_no"))
            self._log("✗ " + tr("log_conn_fail") + msg)

    def _on_disconnect_clicked(self):
        self._connect_cancel.set()
        self.client.close()

    @Slot()
    def _on_disconnected(self):
        self.conn_status.setText(tr("conn_no"))
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.video_btn.setEnabled(False)
        self._video_on = False
        self.video_btn.setText(tr("btn_video_on"))
        with self._frame_mailbox_lock:
            self._frame_mailbox = None
        self.video.clear()                        # 视频 → 占位
        self._reset_sensor_placeholders()         # 仪表盘 → "—" 占位
        self._sensor_count = 0                    # 性能计数立即归零（不残留旧 Hz/fps）
        self._frame_count = 0
        self._update_perf()
        self._log("✗ " + tr("log_disconn"))

    def _on_video_clicked(self):
        if self._video_on:
            if not self._send_json({"cmd": "VIDEO_OFF"}):
                return
            self._video_on = False
        else:
            if not self._send_json({"cmd": "VIDEO_ON"}):
                return
            self._video_on = True
        self.video_btn.setText(tr("btn_video_off") if self._video_on else tr("btn_video_on"))

    def _reset_sensor_placeholders(self):
        """断连后传感器仪表盘全部复位为占位符，清除旧会话残留数据。"""
        for lbl in (self.lbl_dist_f, self.lbl_dist_l, self.lbl_dist_r):
            lbl.setText("—")
        for i in range(4):
            self.lbl_spd[i].setText("—")
            self.lbl_cur[i].setText("—")
        self.lbl_imu.setText("—")
        self.lbl_ack.setText("—")

    @Slot(object)
    def _on_sensor(self, s: SensorData):
        self._sensor_count += 1
        def fmt_mm(v, ok):
            return f"{v} mm" if ok else f"{v} mm({tr('invalid')})"
        self.lbl_dist_f.setText(fmt_mm(s.dist_front_mm, s.dist_front_status == 0 and not s.link_stale))
        self.lbl_dist_l.setText(fmt_mm(s.dist_left_mm, s.dist_left_status == 0 and not s.link_stale))
        self.lbl_dist_r.setText(fmt_mm(s.dist_right_mm, s.dist_right_status == 0 and not s.link_stale))
        for i in range(4):
            self.lbl_spd[i].setText(f"{s.speed_d100[i] / 100:+.1f} cm/s")
            self.lbl_cur[i].setText(f"{s.current_mA[i]} mA")
        self.lbl_imu.setText(
            f"quat=({s.quat_w:.2f},{s.quat_x:.2f},{s.quat_y:.2f},{s.quat_z:.2f}) "
            f"accel=({s.accel_x:.2f},{s.accel_y:.2f},{s.accel_z:.2f}) g")
        self.lbl_ack.setText(
            f"id={s.last_cmd_id} "
            + (tr("st_ok") if s.last_cmd_status == 0 else tr("st_err") + str(s.last_cmd_status)))

    @Slot(object)
    def _on_ack(self, ack: dict):
        """ESP32 命令回执（PROTOCOL.md §2.1 ACK）。"""
        cmd = ack.get("ack", "?")
        if cmd == "PING":
            return  # RTT is shown in the status bar; do not flood logs at 2 Hz.
        status = ack.get("status", -1)
        self._log(f"{tr('log_ack')} {cmd} → "
                  + (tr("st_ok") if status == 0 else f"{tr('st_err')}{status}"))
        self.lbl_ack.setText(f"{cmd} → "
                             + (tr("st_ok") if status == 0 else f"{tr('st_err')}{status}"))

    def _client_disconnected(self):
        self._control_lease.release()
        self._sig_disconn.emit()

    def _queue_frame(self, frame_id, jpeg):
        with self._frame_mailbox_lock:
            self._frame_mailbox = (frame_id, jpeg)
            if self._frame_notification:
                return
            self._frame_notification = True
        self._sig_frame.emit(0, b"")

    @Slot(int, bytes)
    def _on_frame(self, frame_id: int, jpeg: bytes):
        if not jpeg:
            with self._frame_mailbox_lock:
                frame = self._frame_mailbox
                self._frame_mailbox = None
                self._frame_notification = False
            if frame is None:
                return
            frame_id, jpeg = frame
        """真 fps：只统计能解码渲染的帧。QImage 解码做最终校验，坏帧不进视频。"""
        img = QImage.fromData(jpeg, "JPEG")
        if img.isNull():
            return
        self._frame_count += 1
        self.video.show_image(frame_id, img)

    @Slot(str)
    def _append_log(self, text: str):
        self.script_output.appendPlainText(text.rstrip("\n"))
        # 行数上限：超出自动裁剪最旧（滚动日志）
        doc = self.script_output.document()
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
        self.script_output.clear()
        self._log(tr("console_cleared"))

    def _on_lang_toggle(self):
        """顶部双语按钮：zh ⇄ en 一键切换。"""
        set_language("en" if current_language() == "zh" else "zh")
        self._retranslate()

    # ============================ 发送 ============================

    def _send_json(self, obj) -> bool:
        ok = self.client.send_json(obj)
        self._log((("✓ " if ok else "✗ ") +
                   (tr("log_sent") if ok else tr("log_send_fail"))) + str(obj)[:80])
        return ok

    # ============================ 性能 ============================

    def _update_perf(self):
        self._perf_tick = (self._perf_tick + 1) % 5
        if self.tabs.currentIndex() == 1:
            self._update_perf_ide()      # Tab2：SDK 遥测（GUI client 已断）
            return
        sys_info = self.client.sys_info or {}
        cpu0 = sys_info.get("cpu0")
        cpu1 = sys_info.get("cpu1")
        cpu0_txt = "-" if cpu0 is None else f"{cpu0}%"
        cpu1_txt = "-" if cpu1 is None else f"{cpu1}%"
        free, total = sys_info.get("ram_free"), sys_info.get("ram_total")
        ram_txt = "-"
        if isinstance(free, (int, float)) and isinstance(total, (int, float)) and total > 0:
            ram_txt = f"{round((total - free) * 100 / total)}%"
        rtt = self.client.last_rtt_ms
        rtt_txt = "-" if rtt is None else f"{rtt} ms"
        asm = getattr(self.client, "_video_asm", None)
        loss_txt = self._loss_text(asm)
        txt = (f"{tr('perf_sensor')} {self._sensor_count} Hz   "
               f"{tr('perf_video')} {self._frame_count} fps"
               f"{loss_txt}   "
               f"{tr('perf_cpu0')} {cpu0_txt}   {tr('perf_cpu1')} {cpu1_txt}   "
               f"{tr('perf_ram')} {ram_txt}   {tr('perf_rtt')} {rtt_txt}")
        self.lbl_perf.setText(txt)
        self._sensor_count = 0
        self._frame_count = 0     # 每秒归零：显示的是速率不是累计帧数

    @staticmethod
    def _loss_text(asm) -> str:
        """帧组装器丢包统计：坏帧（残帧+结构坏）/ 坏包（迟到+非法）。"""
        if asm is None:
            return ""
        lost_f = asm.frames_aborted + asm.frames_bad_jpeg
        lost_p = asm.packets_late + asm.packets_bad
        return f"   {tr('perf_loss')} {lost_f}f/{lost_p}p"

    def _update_perf_ide(self):
        """Tab2 状态栏：读 SDK 遥测（shell/脚本连接的 _bridge 数据）。

        GUI 自己的 client 在切 Tab2 时已断开（单客户端规范），
        状态栏改读 SDK：传感器计数做差算 Hz、cam 帧计数做差算 fps、
        CPU/RAM/RTT 取 peo.sys（SDK 后台线程 1s 刷新）。
        """
        try:
            self._ensure_sdk_path()
            import peo as _peo
            bridge = _peo._bridge
            cam = _peo.cam
            ok = True
        except Exception:
            ok = False
        if ok:
            n = bridge.sensor_count
            hz = n - self._sdk_sensor_base
            self._sdk_sensor_base = n
            f = cam.frames_total
            fps = f - self._sdk_frame_base
            self._sdk_frame_base = f
            cpu0_txt = "-" if _peo.sys.cpu0 is None else f"{_peo.sys.cpu0}%"
            cpu1_txt = "-" if _peo.sys.cpu1 is None else f"{_peo.sys.cpu1}%"
            ram_txt = "-" if _peo.sys.ram is None else f"{_peo.sys.ram}%"
            rtt = _peo.sys.rtt_ms
            rtt_txt = "-" if rtt is None else f"{rtt} ms"
            asm = getattr(bridge.client, "_video_asm", None)
            loss_txt = self._loss_text(asm)
        else:
            hz = fps = 0
            cpu0_txt = cpu1_txt = ram_txt = rtt_txt = "-"
            loss_txt = ""
        txt = (f"{tr('perf_sensor')} {hz} Hz   "
               f"{tr('perf_video')} {fps} fps"
               f"{loss_txt}   "
               f"{tr('perf_cpu0')} {cpu0_txt}   {tr('perf_cpu1')} {cpu1_txt}   "
               f"{tr('perf_ram')} {ram_txt}   {tr('perf_rtt')} {rtt_txt}")
        self.lbl_perf.setText(txt)

    def closeEvent(self, event):
        self._closing = True
        self._connect_cancel.set()
        self._ensure_sdk_path()
        import peo
        peo._request_stop()
        peo.disconnect()
        self.client.close()
        self._control_lease.release()
        try:
            self._shell_q.put_nowait(None)
        except queue.Full:
            pass
        self._perf_timer.stop()
        self._ide_video_timer.stop()
        self._shell_drain_timer.stop()
        event.accept()
