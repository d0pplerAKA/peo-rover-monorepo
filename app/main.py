"""
app.main - PEO Rover ground-station entry point.

Run either way:
    python -m app.main
    python app/main.py

Copyright (c) 2026 Danny
SPDX-License-Identifier: MIT
"""
import os
import sys

# Support running this file directly (VS Code ▶ button / python app/main.py):
# in that case __package__ is None and sys.path[0] is the app/ directory, not
# the project root, so we prepend the project root to make `from app.gui ...`
# resolvable.
if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PySide6.QtCore import qInstallMessageHandler
from PySide6.QtWidgets import QApplication

from app.gui import MainWindow


def _qt_message_filter(mode, ctx, msg: str):
    """Silence Qt image-decoding noise as a last resort: bad frames are already
    blocked at the assembly layer (SOI/EOI guard) and the GUI layer (byte pre-
    check + isNull); this filters any remaining reports to avoid stderr spam."""
    if msg.startswith("qt.gui.imageio."):
        return
    sys.stderr.write(msg + "\n")


_STDERR_LOG = None   # held at module level: prevents the GC from closing the
                     # file object and leaving fd 2 dangling (EBADF)


def _redirect_c_stderr() -> None:
    """Redirect fd 2 to stderr.log at the project root.

    libjpeg writes its decode warnings with fprintf(stderr) (a C runtime
    handle), which replacing sys.stderr in Python cannot intercept. Redirecting
    at the OS level keeps the user's screen clean while the errors remain
    traceable; GUI logging through the Qt message system is unaffected.

    The file object must be held at module level, otherwise the GC closes it
    and fd 2 dangles (spamming EBADF).
    """
    global _STDERR_LOG
    log_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "stderr.log")
    try:
        _STDERR_LOG = open(log_path, "a", encoding="utf-8", buffering=1)
        os.dup2(_STDERR_LOG.fileno(), 2)
    except OSError:
        pass   # a redirect failure must not prevent startup


def main():
    qInstallMessageHandler(_qt_message_filter)
    _redirect_c_stderr()
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()


# Kept example snippet (a plain string expression at module level — inert):
'''
import peo
peo.connect()
peo.delay(500)
peo.cam.set_resolution("QVGA")
peo.cam.on()

while True:
	peo.delay(100)
'''
