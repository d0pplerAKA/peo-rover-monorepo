"""
app.theme — 全局暗色 QSS（固定暗色主题，不再切换）

用法：
    from app.theme import apply_theme
    apply_theme(app)
"""
from PySide6.QtWidgets import QApplication

DARK_QSS = """
QWidget {
    background: #1e1f24;
    color: #d8dae0;
    font-size: 13px;
}
QGroupBox {
    border: 1px solid #3a3d46;
    border-radius: 6px;
    margin-top: 6px;
    padding-top: 12px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 8px;
    padding: 0 4px;
    color: #9aa0ac;
}
QPushButton {
    background: #2c2f37;
    border: 1px solid #454a55;
    border-radius: 5px;
    padding: 3px 8px;
}
QPushButton:hover { background: #383c46; }
QPushButton:pressed { background: #24272e; }
QPushButton:disabled { color: #6a6f7a; background: #26282f; }
/* 急停按钮（安全设施：红色突出，Tab1 唯一命令按钮） */
QPushButton#stopBtn {
    background: #8b2c2c;
    border: 1px solid #c04040;
    color: #ffd9d9;
    font-weight: bold;
    font-size: 14px;
}
QPushButton#stopBtn:hover { background: #a33333; }
QPushButton#stopBtn:pressed { background: #6f2323; }
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QPlainTextEdit, QTextEdit {
    background: #16171b;
    border: 1px solid #454a55;
    border-radius: 4px;
    padding: 2px;
    selection-background-color: #3a5f8a;
}
QComboBox::drop-down { border: none; width: 18px; }
QComboBox QAbstractItemView {
    background: #22242b;
    border: 1px solid #454a55;
    selection-background-color: #3a5f8a;
}
QSplitter::handle { background: #2a2c33; }
QScrollArea { border: none; background: transparent; }
QScrollBar:vertical {
    background: #1a1b20;
    width: 10px;
    border-radius: 5px;
}
QScrollBar::handle:vertical {
    background: #454a55;
    border-radius: 4px;
    min-height: 30px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QStatusBar { background: #17181c; color: #9aa0ac; }
QToolTip {
    background: #2c2f37;
    color: #d8dae0;
    border: 1px solid #454a55;
}
"""


def apply_theme(app: QApplication):
    """应用全局暗色主题。"""
    app.setStyleSheet(DARK_QSS)
