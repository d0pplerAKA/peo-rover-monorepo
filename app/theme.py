"""
app.theme - fixed dark QSS stylesheet for the whole application.

The theme is dark-only (light/dark switching was removed by decision).

Usage:
    from app.theme import apply_theme
    apply_theme(app)

Copyright (c) 2026 Danny
SPDX-License-Identifier: MIT
"""
from PySide6.QtWidgets import QApplication

# Global dark stylesheet. Each hex value is the background / text / border
# color of a widget role; dark colors keep contrast high on a workbench.
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
/* Emergency stop button (safety feature: highlighted red, the only command button on Tab 1) */
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
    """Apply the fixed dark theme to the whole application.

    Args:
        app: the QApplication instance whose stylesheet is replaced.
    """
    app.setStyleSheet(DARK_QSS)
