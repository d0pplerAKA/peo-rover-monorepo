"""
app.gui - main window v2 (rebuilt after the 2026-11-16 team programming model).

Tab 1 "Monitor": connection + video + read-only parameter dashboard + E-STOP.
Tab 2 "Team IDE": script editor / run / stop + shell REPL + video window +
output console (including global logs).
    - Scripts run on a separate thread (a blocking script never freezes the
      GUI); stopping is cooperative (peo.delay checks a flag).
    - Switching to Tab 2 automatically disconnects the GUI connection
      (single-client rule, PROTOCOL §2.3).
    - The shell and a running script are mutually exclusive (single control
      source).

Thread model: RoverClient receive-thread callbacks -> cross-thread Qt signal
emit -> main-thread slots update the UI.
Bilingual: every UI string goes through app.i18n.tr(); _retranslate() refreshes
on language switch.

Copyright (c) 2026 Danny
SPDX-License-Identifier: MIT
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
    QSizePolicy,
)

from rover_comm.client import RoverClient
from rover_comm.control import ControlLock
from rover_comm.protocol import SensorData, TCP_CMD_PORT, VIDEO_TCP_PORT
from app.i18n import tr, set_language, current_language
from app.theme import apply_theme
from app.video import VideoWidget


def _enqueue_output(target, text):
    """Push text into a bounded queue in chunks, dropping the oldest chunk when
    full (bounds both the item count and each item's size)."""
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
    """Streaming output forwarder: a script thread's write() enqueues
    immediately (a main-thread 50 ms timer drains it to the console).

    Replaces io.StringIO buffering so print() output during a script appears
    live in the console instead of all at once at the end.
    """

    def __init__(self, q: "queue.Queue"):
        super().__init__()
        self._q = q

    def write(self, s: str) -> int:
        if s:
            _enqueue_output(self._q, s)
        return len(s)

    def flush(self) -> None:
        pass


class LineNumberArea(QWidget):
    """The left-hand line-number gutter of the code editor (ported from the
    official Qt LineNumberArea example)."""

    def __init__(self, editor: "CodeEditor"):
        super().__init__(editor)
        self._editor = editor

    def sizeHint(self):
        return QSize(self._editor.line_number_area_width(), 0)

    def paintEvent(self, event):
        self._editor.line_number_area_paint(event)


class CodeEditor(QPlainTextEdit):
    """Plain-text editor with a line-number gutter and current-line highlight
    (used by the team IDE)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        # Tab indents 4 spaces (Python convention; the Qt default of 8 is too wide)
        self.setTabStopDistance(4 * self.fontMetrics().horizontalAdvance(" "))
        self._line_area = LineNumberArea(self)
        self.blockCountChanged.connect(self._update_area_width)
        self.updateRequest.connect(self._update_area)
        self.cursorPositionChanged.connect(self._highlight_current)
        self._update_area_width(0)
        self._highlight_current()

    def setFont(self, font):
        """Recompute the Tab width after a font change so indentation stays at
        4 spaces under any font."""
        super().setFont(font)
        self.setTabStopDistance(4 * self.fontMetrics().horizontalAdvance(" "))

    def line_number_area_width(self) -> int:
        """Width (px) needed to render the largest line number plus padding."""
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
        """Highlight the whole current line (cursor-line indicator while
        typing)."""
        sel = QTextEdit.ExtraSelection()
        sel.format.setBackground(QColor("#242c3a"))   # clearly brighter than the editor bg
        sel.format.setProperty(QTextFormat.FullWidthSelection, True)
        sel.cursor = self.textCursor()
        sel.cursor.clearSelection()
        self.setExtraSelections([sel])

_MAX_LOG_BLOCKS = 500      # max console log lines; older lines scroll off
_PERF_INTERVAL_MS = 1000   # status-bar refresh period (CPU/RAM)
_IDE_VIDEO_MS = 33         # main-thread latest-frame polling; bounded render cadence
_IDE_SHOW_MIN_S = 0.030    # Tab 2 minimum display interval: smooth cadence
                           # (24 fps = 42 ms, so no frame is dropped)

_CAM_RESOLUTIONS = (("QQVGA (160x120)", "QQVGA"),
                    ("QVGA (320x240)", "QVGA"),
                    ("VGA (640x480)", "VGA"),
                    ("SVGA (800x600)", "SVGA"))

# IDE default code (user decision 2026-11-17): pre-filled when a team member
# first opens the IDE. Indentation is 4 spaces (Python convention, matching the
# IDE Tab setting).
_DEFAULT_SCRIPT = (
    "import peo\n"
    "peo.connect()\n"
    "peo.delay(500)\n"
    "peo.cam.set_resolution(\"SVGA\")\n"
    "peo.cam.on()\n"
    "peo.sys_reset()\n"
    "\n"
    "\n"
    "while True:\n"
    "    peo.led(True)\n"
    "    peo.delay(1000)\n"
    "    peo.led(False)\n"
    "    peo.delay(1000)\n"
)


class _PeoConsole(code.InteractiveConsole):
    """A true REPL: stdlib InteractiveConsole with output forwarded to the GUI
    console.

    Provides: multi-line continuation (the ... prompt), expression echo,
    persistent variables across statements, and exception capture.
    """

    def __init__(self, env: dict, write_cb):
        super().__init__(env)
        self._write_cb = write_cb

    def write(self, data: str):
        self._write_cb(data)

    def flush(self):
        pass

    def runcode(self, code_):
        """Converge displayhook (expression echo) / print / stderr into write.

        The default implementation writes to the process sys.stdout (the
        terminal, not the GUI console); this redirects it. The shell and a
        script are mutually exclusive (the shell locks while a script runs), so
        global stdout has no concurrent writers."""
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
    """The shell input line: ↑/↓ history and Tab completion (rlcompleter)."""

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
    # Cross-thread signals (RoverClient receive threads emit -> main-thread
    # slots; Qt auto-queues them).
    _sig_sensor = Signal(object)      # SensorData
    _sig_frame = Signal(int, bytes)   # frame_id, jpeg
    _sig_ack = Signal(object)         # dict {"ack":cmd,"status":n}
    _sig_log = Signal(str)
    _sig_disconn = Signal()
    _sig_conn = Signal(bool, str)       # (success?, host or error message)
    # IDE output (script/shell threads -> queue -> main-thread QTimer drain)
    _sig_shell_out = Signal(str)
    _sig_ide_status = Signal(str)
    _sig_script_finished = Signal()     # script thread done -> restore buttons
    _sig_ide_frame = Signal()          # Tab 2 frame-arrived note (no payload:
                                       # the main thread pulls the latest frame)

    def __init__(self):
        super().__init__()
        self.resize(1280, 800)        # initial window size (a sensible workbench)
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
        self.client.on_sensor = self._sig_sensor.emit          # receive thread
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

        # IDE runtime state
        self._script_thread = None
        self._shell_thread = None
        self._script_running = False
        self._current_file = None      # path of the current script (None = default, unsaved)
        self._bad_seq = -1             # Tab 2 last-known bad-frame seq (decode once)
        self._shown_seq = -1           # Tab 2 last-shown frame seq (dedupe backlog)
        self._last_ide_show = None     # Tab 2 last display time (throttling)
        self._bad_dump_count = 0       # bad-frame forensics dump count (max 8)
        # shell REPL state
        self._shell_q = queue.Queue(maxsize=4)      # main thread -> shell thread input lines
        self._shell_out_q = queue.Queue(maxsize=256)  # shell thread -> main thread output
        self._shell_more = False           # continuation mode (... prompt, UI state)
        self._shell_more_flag = False      # continuation flag written by the shell thread
        self._shell_history = deque(maxlen=200)
        self._shell_hist_idx = None
        self._shell_completer = None       # rlcompleter.Completer (shell thread fills it)
        self._shell_started = False

        self._build_ui()          # build widgets and layouts only
        self._retranslate()       # fill text per the current language

        # Fixed dark theme
        apply_theme(QApplication.instance())

        # E-STOP shortcut: Space (ignored while a text box has focus)
        self._stop_sc = QShortcut(QKeySequence(Qt.Key_Space), self)
        self._stop_sc.activated.connect(self._on_shortcut_stop)

        # Perf counters: refresh the status bar once a second
        self._perf_timer = QTimer(self)
        self._perf_timer.timeout.connect(self._update_perf)
        self._perf_timer.start(_PERF_INTERVAL_MS)
        self._update_perf()     # show the initial "0 Hz / 0 fps" immediately

        # Tab 2 latest-frame mailbox polling (~30 Hz; capture target is 24 fps)
        self._ide_video_timer = QTimer(self)
        self._ide_video_timer.timeout.connect(self._pull_ide_video)
        self._ide_video_timer.start(_IDE_VIDEO_MS)

        # Shell output drain (shell thread -> queue -> main-thread console, 50 ms)
        self._shell_drain_timer = QTimer(self)
        self._shell_drain_timer.timeout.connect(self._drain_shell_out)
        self._shell_drain_timer.start(50)

        self._log(tr("log_ready"))

    # ============================ UI building ============================
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 6, 8, 4)
        root.setSpacing(4)

        # Top bar: app title + language toggle button (fixed right, does not
        # squeeze the tab titles)
        topbar = QHBoxLayout()
        self.lbl_app = QLabel()
        self.lbl_app.setStyleSheet("color: #9aa0ac; font-size: 12px;")
        topbar.addWidget(self.lbl_app)
        topbar.addStretch(1)
        self.lbl_file_path = QLabel()          # current script file path (top right)
        self.lbl_file_path.setStyleSheet("color: #9aa0ac; font-size: 11px;")
        topbar.addWidget(self.lbl_file_path)
        topbar.addSpacing(12)
        self.lang_btn = QPushButton()
        self.lang_btn.setMinimumWidth(64)      # fit its own content, avoid truncation
        self.lang_btn.setMaximumWidth(80)
        self.lang_btn.setToolTip("Switch language / 切换语言")
        self.lang_btn.clicked.connect(self._on_lang_toggle)
        topbar.addWidget(self.lang_btn)
        root.addLayout(topbar)

        self.tabs = QTabWidget()
        self.tabs.currentChanged.connect(self._on_tab_changed)
        root.addWidget(self.tabs)
        self.tabs.addTab(self._build_tab1(), "")   # text filled by _retranslate
        self.tabs.addTab(self._build_tab2(), "")

        # Status bar: perf/frame rate + lock indicator (lock at the far right)
        self.lbl_perf = QLabel()
        self.lbl_perf.setContentsMargins(0, 0, 8, 0)
        self.statusBar().addPermanentWidget(self.lbl_perf)

        self._lock_val = None
        self.lbl_lock = QLabel()
        self.lbl_lock.setContentsMargins(0, 0, 12, 0)
        self.statusBar().addPermanentWidget(self.lbl_lock)
        self._set_lock_indicator(None)   # initial: unknown

    # ---------------- Tab 1: manual monitor ----------------

    def _build_tab1(self):
        w = QWidget()
        lay = QHBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        splitter = QSplitter(Qt.Horizontal)
        lay.addWidget(splitter)

        # Left column: resolution + video
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)
        camrow = QHBoxLayout()
        self.lbl_cam_res = QLabel()
        camrow.addWidget(self.lbl_cam_res)
        self.cam_res_combo = QComboBox()
        self.cam_res_combo.addItems([d for d, _ in _CAM_RESOLUTIONS])
        self.cam_res_combo.setCurrentIndex(1)   # default QVGA (320x240)
        self.cam_res_combo.activated.connect(self._on_cam_res_changed)
        camrow.addWidget(self.cam_res_combo)
        camrow.addStretch(1)
        lv.addLayout(camrow)
        self.video = VideoWidget()
        lv.addWidget(self.video, 1)
        splitter.addWidget(left)

        # Right column: connection + read-only dashboard + E-STOP
        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(0, 0, 0, 0)
        rv.addWidget(self._build_conn_group())
        rv.addWidget(self._build_sensor_group())
        # E-STOP (safety feature: the only command button on Tab 1)
        self.btn_stop = QPushButton()
        self.btn_stop.setMinimumHeight(44)
        self.btn_stop.setObjectName("stopBtn")
        self.btn_stop.clicked.connect(lambda: self._send_json({"cmd": "STOP_ALL"}))
        rv.addWidget(self.btn_stop)
        rv.addStretch(1)
        splitter.addWidget(right)
        # Give the control column the spare width: the dashboard values need
        # stable, full display space; the video column keeps its minimum.
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([400, 880])
        return w

    def _build_conn_group(self):
        g = QGroupBox()
        self.grp_conn = g
        f = QFormLayout(g)
        f.setContentsMargins(6, 2, 6, 4)
        f.setVerticalSpacing(3)

        row = QHBoxLayout()
        self.host_edit = QLineEdit("peo_rover.local")
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

        # Layout-stable dashboard: value-label text length must never drive
        # the layout (and thus the window) width. Ignored horizontal policy
        # detaches sizeHint from the layout; stretched columns then share the
        # window width so every field keeps stable, full display space.
        _ign = QSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)

        self.lbl_dist_f = QLabel("—")
        self.lbl_dist_f.setSizePolicy(_ign)
        self.lbl_dist_l = QLabel("—")
        self.lbl_dist_l.setSizePolicy(_ign)
        self.lbl_dist_r = QLabel("—")
        self.lbl_dist_r.setSizePolicy(_ign)
        self.lbl_spd = [QLabel("—") for _ in range(4)]
        self.lbl_cur = [QLabel("—") for _ in range(4)]
        for lbl in self.lbl_spd + self.lbl_cur:
            lbl.setSizePolicy(_ign)
        self.lbl_imu = QLabel("—")
        self.lbl_imu.setSizePolicy(_ign)
        self.lbl_ack = QLabel("—")
        self.lbl_ack.setSizePolicy(_ign)

        self._sensor_labels = []          # (QLabel title, tr key); _retranslate refreshes
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
        # ACK echo row
        t = QLabel(); grid.addWidget(t, r, 0); grid.addWidget(self.lbl_ack, r, 1, 1, 5)
        self._sensor_labels.append((t, "cmd_ack"))

        # All columns share spare width equally (stretch): at a normal window
        # size every field gets full display space, while the layout minimum
        # stays small (title-driven) so the window never gets forced wider.
        for c in range(grid.columnCount()):
            grid.setColumnStretch(c, 1)
        return g

    # ---------------- Tab 2: team IDE ----------------

    def _build_tab2(self):
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)

        # Toolbar: open / save / save-as / run / stop + status
        toolbar = QHBoxLayout()
        self.btn_open = QPushButton()
        self.btn_open.clicked.connect(self._on_open_script)
        self.btn_save = QPushButton()
        self.btn_save.clicked.connect(self._on_save_script)
        self.btn_save_as = QPushButton()
        self.btn_save_as.clicked.connect(self._on_save_as_script)
        self.btn_run = QPushButton()
        self.btn_run.clicked.connect(self._on_run_script)
        self.btn_stop_script = QPushButton()
        self.btn_stop_script.clicked.connect(self._on_stop_script)
        self.btn_stop_script.setEnabled(False)
        self.lbl_ide_status = QLabel()
        toolbar.addWidget(self.btn_open)
        toolbar.addWidget(self.btn_save)
        toolbar.addWidget(self.btn_save_as)
        toolbar.addWidget(self.btn_run)
        toolbar.addWidget(self.btn_stop_script)
        toolbar.addWidget(self.lbl_ide_status)
        toolbar.addStretch(1)
        lay.addLayout(toolbar)

        # Top: script editor (line numbers + current-line highlight), pre-filled
        # with the default code
        self.ide_edit = CodeEditor()
        self.ide_edit.setFont(QFont("Consolas", 10))
        self.ide_edit.setPlainText(_DEFAULT_SCRIPT)
        lay.addWidget(self.ide_edit, 3)

        # Bottom: left video window / right shell + console (with global logs)
        lower = QSplitter(Qt.Horizontal)
        self.ide_video = VideoWidget()
        lower.addWidget(self.ide_video)

        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(0, 0, 0, 0)
        # Unified console (shell Q&A + script stdout + system logs) + clear button
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
        # Shell input line (Enter to run; echo + result go to the console)
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

    # ---------------- Text (bilingual refresh) ----------------

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
        # Top language button: shows the TARGET language (current zh -> "EN",
        # current en -> "中文")
        self.lang_btn.setText("EN" if current_language() == "zh" else "中文")
        self.lbl_app.setText(tr("app_brand"))

        self.grp_sensor.setTitle(tr("grp_sensor"))
        for t, key in self._sensor_labels:
            t.setText(tr(key))
        for i in range(4):
            self._spd_title[i].setText(tr("spd_col", i + 1))
            self._cur_title[i].setText(tr("cur_col", i + 1))

        # Tab 2 text
        self.btn_open.setText(tr("btn_open"))
        self.btn_save.setText(tr("btn_save"))
        self.btn_save_as.setText(tr("btn_save_as"))
        self.btn_run.setText(tr("btn_run"))
        self.btn_stop_script.setText(tr("btn_stop_script"))
        self.lbl_script_console.setText(tr("grp_console"))
        self.ide_edit.setPlaceholderText(tr("ide_placeholder"))
        self.shell_input.setPlaceholderText(tr("shell_placeholder"))
        self.btn_console_clear.setText(tr("console_clear"))
        self.video.update()    # redraw the video placeholder in the new language
        self.ide_video.update()
        self._update_file_label()   # top-right file path (default-code state)
        if hasattr(self, "lbl_lock"):   # _retranslate runs before lbl_lock is created
            self._set_lock_indicator(self._lock_val)

    # ---------------- Tab switching ----------------

    def _on_tab_changed(self, idx: int):
        if idx == 1 and (self.client.connected or self._connecting):
            self._connect_cancel.set()
            # Switching to the IDE auto-disconnects the GUI connection
            # (single-client rule), releasing control to the team script.
            self.client.close()
            self._log(tr("log_tab2_disconn"))
            self._set_ide_status(tr("ide_ready"))

    # ---------------- IDE execution ----------------

    def _ensure_sdk_path(self):
        """Add sdk/ to sys.path (both team scripts and the GUI's own peo
        imports depend on it)."""
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        sdk = os.path.join(root, "sdk")
        if sdk not in sys.path:
            sys.path.insert(0, sdk)
        return sdk

    def _peo_env(self):
        """Build the script execution environment: inject the peo module (sdk
        path) so both styles work: `peo.drive_all(30)` (import peo) and
        `drive_all(30)` (from peo import *)."""
        self._ensure_sdk_path()
        import peo
        env = {"__name__": "__main__", "peo": peo}
        env.update({k: getattr(peo, k) for k in peo.__all__})
        return env

    def _run_code_in_thread(self, code: str, out_q: queue.Queue):
        """exec the script code on a separate thread, streaming stdout/stderr to
        the queue (each write enqueues immediately)."""
        env = self._peo_env()
        relay = _StreamRelay(out_q)
        try:
            with contextlib.redirect_stdout(relay), contextlib.redirect_stderr(relay):
                try:
                    exec(compile(code, "<script>", "exec"), env, env)
                except SystemExit:
                    pass
                except Exception as e:
                    # Exceptions also go to the console (visible in the IDE
                    # console, not the terminal).
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
            self._current_file = path
            self._set_ide_status(os.path.basename(path))
            self._update_file_label()
        except OSError as e:
            self._log("✗ " + tr("log_open_fail") + str(e))

    def _on_save_script(self):
        """Save: an associated path writes back in place; unsaved default code
        falls through to Save As."""
        if not self._current_file:
            self._on_save_as_script()
            return
        try:
            with open(self._current_file, "w", encoding="utf-8") as f:
                f.write(self.ide_edit.toPlainText())
            self._log("✓ " + tr("log_save_ok") + self._current_file)
            self._set_ide_status(os.path.basename(self._current_file))
            self._update_file_label()
        except OSError as e:
            self._log("✗ " + tr("log_save_fail") + str(e))

    def _on_save_as_script(self):
        """Save As: choose a path, write, and associate it."""
        path, _ = QFileDialog.getSaveFileName(
            self, tr("dlg_save_as"), "script.py",
            "Python (*.py);;All Files (*)")
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.ide_edit.toPlainText())
            self._current_file = path
            self._log("✓ " + tr("log_save_ok") + path)
            self._set_ide_status(os.path.basename(path))
            self._update_file_label()
        except OSError as e:
            self._log("✗ " + tr("log_save_fail") + str(e))

    def _update_file_label(self):
        """Top-right area: show the current script file path; when unassociated,
        show the default-code status."""
        self.lbl_file_path.setText(
            self._current_file if self._current_file else tr("file_default"))

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
            self.client.close()   # safety: GUI must disconnect before the script connects

        self._script_running = True
        self.btn_run.setEnabled(False)
        self.btn_stop_script.setEnabled(True)
        self._set_ide_status(tr("ide_running"))
        self.script_output.clear()
        # Drain leftover output from the previous run (streaming queue)
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

        self._script_out_q = queue.Queue(maxsize=256)   # script thread output -> 50 ms drain

        def _runner():
            try:
                self._run_code_in_thread(code, self._script_out_q)
            finally:
                # Script end (normal/stop/exception) -> disconnect + release the
                # single-instance lease (single-client rule: the script does not
                # hold the connection, the next script/GUI can connect at once).
                self._ensure_sdk_path()
                import peo as _peo
                try:
                    _peo._bridge.disconnect()
                except Exception:
                    pass
                self._sig_ide_status.emit(tr("ide_done"))
                self._sig_script_finished.emit()   # restore button state on main thread

        self._script_thread = threading.Thread(target=_runner, daemon=True,
                                               name="peo-script")
        self._script_thread.start()

    @Slot()
    def _on_script_finished(self):
        """Script thread done (Qt widgets must be touched only on the main
        thread; button state is restored here)."""
        self._script_running = False
        self.btn_run.setEnabled(True)
        self.btn_stop_script.setEnabled(False)

    def _on_stop_script(self):
        self._ensure_sdk_path()
        import peo as _peo
        _peo._request_stop()      # queue STOP now; later SDK actions/waits raise ScriptStopped
        self._set_ide_status(tr("ide_stopping"))

    # ---------------- shell REPL (true REPL: InteractiveConsole) ----------------

    def _start_shell(self):
        """Long-lived shell thread: InteractiveConsole is stateful (variables
        persist), so it must stay on one thread."""
        if self._shell_started:
            return
        self._shell_started = True
        env = self._peo_env()

        def _write(data: str):
            _enqueue_output(self._shell_out_q, data)   # thread-safe queue; main thread drains

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
                # Input echo (with the current prompt)
                _write(("... " if self._shell_more else ">>> ") + line + "\n")
                try:
                    more = console.push(line)   # true REPL semantics
                except SystemExit:
                    more = False
                finally:
                    self._shell_more_flag = locals().get("more", False)
                    self._shell_busy.clear()

        self._shell_thread = threading.Thread(target=_loop, daemon=True,
                                              name="peo-shell")
        self._shell_thread.start()

    def _drain_shell_out(self):
        """Main thread periodically drains the shell output queue into the
        unified console (Qt widgets are main-thread only). Also syncs the shell
        thread's continuation flag to the prompt label."""
        for _ in range(64):
            try:
                data = self._shell_out_q.get_nowait()
            except queue.Empty:
                break
            self._append_ide(self.script_output, data)
        # Script streaming output uses the same drain (prints show live)
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
            return   # empty input in normal mode is ignored; in continuation
                     # mode an empty line ends the multi-line block
        self._start_shell()
        # History: append + reset cursor (main-thread only; deque append + get
        # have no concurrent writer)
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
        """Tab completion: a unique match completes; several matches are listed."""
        if self._shell_completer is None:
            return
        text = self.shell_input.text()
        pos = self.shell_input.cursorPosition()
        if not text or pos == 0:
            return
        # Take the "identifier.prefix" word before the cursor
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
        """Main thread: read the latest frame, skip already-shown/known-bad
        frames, and bound the decode rate."""
        if self._last_ide_show is not None and \
                time.monotonic() - self._last_ide_show < _IDE_SHOW_MIN_S / 1.0:
            return                            # throttled: wait for the next note
        try:
            self._ensure_sdk_path()
            import peo as _peo
            jpeg, seq = _peo.cam.read_raw(timeout_ms=0)
        except Exception:
            return
        if jpeg is None:
            if self.ide_video._frame_id != -1:
                self.ide_video.clear()      # disconnected/off: back to placeholder
            return
        if seq == self._shown_seq or seq == self._bad_seq:
            return                            # already shown / known bad
        img = QImage.fromData(jpeg, "JPEG")
        if img.isNull():
            # QImage strict mode rejected it -> cv2 lenient fallback: the ESP32
            # occasionally outputs a half frame (entropy cut short but EOI
            # present); cv2 can still decode a nearly-complete image.
            img = self._cv2_decode_fallback(jpeg)
        if img is None or img.isNull():
            self._bad_seq = seq               # record the bad seq, report once
            self._dump_bad_frame(jpeg, seq)   # dump to disk (byte-level analysis)
            return                            # keep the previous frame (like Tab 1)
        self._last_ide_show = time.monotonic()
        self._shown_seq = seq
        self.ide_video.show_image(seq, img)

    @staticmethod
    def _cv2_decode_fallback(jpeg: bytes):
        """cv2 lenient decode fallback: a truncated frame (premature end) can
        still yield a complete image; returns None on failure.

        stderr is redirected during decoding: libjpeg's premature-end warning is
        swallowed — Python-level warnings go to a sink, C-level warnings go to
        fd 2 (archived in stderr.log); nothing visible (console/screen) remains.
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
        """Dump a bad frame to disk for forensics (max 8, oldest deleted
        first) — for byte-level root-cause analysis."""
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
        """Bounded 30 Hz polling: render only the latest SDK frame, no Qt
        backlog."""
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

    # ---------------- Slots (main thread) ----------------

    def _on_shortcut_stop(self):
        """Space E-STOP; ignored while a text box has focus."""
        fw = QApplication.focusWidget()
        if isinstance(fw, (QLineEdit, QPlainTextEdit, QTextEdit)):
            return
        if self.tabs.currentIndex() == 1:
            self._on_stop_script()
            return
        if self._send_json({"cmd": "STOP_ALL"}):
            self._log(tr("log_sc_stop"))

    def _on_cam_res_changed(self, idx: int):
        """Hot-switch camera resolution: {"cmd":"CAM_RES","res":...} (an
        ESP32-S3 local command)."""
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
        host = self.host_edit.text().strip() or "peo_rover.local"
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
                # Only the video PORT is configured here; 8890 is dialed after
                # the user clicks Start Video.
                self.client.connect(host, tcp_port=TCP_CMD_PORT, video_port=VIDEO_TCP_PORT)
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
            # Reset config to default after disconnect: sending the default
            # resolution on connect keeps the UI consistent after a reset.
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
        self.video.clear()                        # video -> placeholder
        self._reset_sensor_placeholders()         # dashboard -> "—" placeholders
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
        """On disconnect, reset every sensor dashboard value to the placeholder
        and clear stale session data."""
        for lbl in (self.lbl_dist_f, self.lbl_dist_l, self.lbl_dist_r):
            lbl.setText("—")
        for i in range(4):
            self.lbl_spd[i].setText("—")
            self.lbl_cur[i].setText("—")
        self.lbl_imu.setText("—")
        self.lbl_ack.setText("—")
        self._set_lock_indicator(None)

    def _set_lock_indicator(self, lock_val):
        """Bottom-right lock badge: 0 = manual (green), 1 = autonomous (orange),
        None = unknown (gray)."""
        self._lock_val = lock_val
        qss = ("QLabel {{ background-color: {}; color: {}; "
               "padding: 2px 8px; border-radius: 9px; font-weight: bold; }}")
        if lock_val == 0:
            self.lbl_lock.setText("🔒 " + tr("lock_manual"))
            self.lbl_lock.setStyleSheet(qss.format("#2e7d32", "#ffffff"))
        elif lock_val == 1:
            self.lbl_lock.setText("🔓 " + tr("lock_auto"))
            self.lbl_lock.setStyleSheet(qss.format("#e65100", "#ffffff"))
        else:
            self.lbl_lock.setText("—")
            self.lbl_lock.setStyleSheet(qss.format("#3a3f4a", "#9aa0ac"))
        self.lbl_lock.setToolTip(tr("lock_tooltip"))

    @Slot(object)
    def _on_sensor(self, s: SensorData):
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
        self._set_lock_indicator(s.lock)

    @Slot(object)
    def _on_ack(self, ack: dict):
        """ESP32 command receipt (PROTOCOL.md §2.1 ACK)."""
        cmd = ack.get("ack", "?")
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
        """True fps: only frames that decode and render count. QImage decoding
        is the final check; bad frames never reach the video."""
        if not jpeg:
            with self._frame_mailbox_lock:
                frame = self._frame_mailbox
                self._frame_mailbox = None
                self._frame_notification = False
            if frame is None:
                return
            frame_id, jpeg = frame
        img = QImage.fromData(jpeg, "JPEG")
        if img.isNull():
            return
        self.video.show_image(frame_id, img)

    @Slot(str)
    def _append_log(self, text: str):
        self.script_output.appendPlainText(text.rstrip("\n"))
        # Line cap: trim the oldest block once over the limit (scrolling log)
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
        """Clear the console log."""
        self.script_output.clear()
        self._log(tr("console_cleared"))

    def _on_lang_toggle(self):
        """Top bilingual button: one-click zh ⇄ en switch."""
        set_language("en" if current_language() == "zh" else "zh")
        self._retranslate()

    # ============================ Send ============================

    def _send_json(self, obj) -> bool:
        ok = self.client.send_json(obj)
        self._log((("✓ " if ok else "✗ ") +
                   (tr("log_sent") if ok else tr("log_send_fail"))) + str(obj)[:80])
        return ok

    # ============================ Performance ============================

    def _update_perf(self):
        """Refresh CPU0/CPU1/RAM every second from the active connection's
        sensor.sys."""
        if self.tabs.currentIndex() == 1:
            self._update_perf_ide()
            return
        self._show_sys_info(self.client.sys_info)

    def _show_sys_info(self, sys_info):
        """Shared formatting for both tabs; placeholders when no telemetry,
        RAM shown as a usage percent."""
        sys_info = sys_info or {}
        cpu0, cpu1 = sys_info.get("cpu0"), sys_info.get("cpu1")
        cpu0_txt = "-" if cpu0 is None else f"{cpu0}%"
        cpu1_txt = "-" if cpu1 is None else f"{cpu1}%"
        free, total = sys_info.get("ram_free"), sys_info.get("ram_total")
        ram_txt = "-"
        if isinstance(free, (int, float)) and isinstance(total, (int, float)) and total > 0:
            ram_txt = f"{round((total - free) * 100 / total)}%"
        self.lbl_perf.setText(
            f"{tr('perf_cpu0')} {cpu0_txt}   {tr('perf_cpu1')} {cpu1_txt}   "
            f"{tr('perf_ram')} {ram_txt}")

    def _update_perf_ide(self):
        """Tab 2 reads the SDK connection's sensor.sys (the GUI connection is
        already closed after the tab switch)."""
        try:
            self._ensure_sdk_path()
            import peo
            sys_info = peo._bridge.client.sys_info
        except ImportError:
            sys_info = None
        self._show_sys_info(sys_info)

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
