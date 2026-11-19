"""
app.video - video display widget (decodes JPEG directly with QImage, no
OpenCV dependency).

Copyright (c) 2026 Danny
SPDX-License-Identifier: MIT
"""
import time

from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap, QPainter
from PySide6.QtWidgets import QWidget

from app.i18n import tr


class VideoWidget(QWidget):
    """Display the JPEG frames received over TCP 8890, scaled to fit, centered.

    Attributes:
        _pixmap: the currently displayed QPixmap (None = placeholder).
        _frame_id: id of the last shown frame (-1 = none yet).
        _frame_times: recent frame arrival timestamps (rolling window) used to
            compute the true fps.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._pixmap = None
        self._frame_id = -1
        self._frame_times = []   # recent arrival timestamps (rolling window)
        self.setMinimumSize(280, 140)

    def _current_fps(self) -> float:
        """Rolling fps over the last 30 frame arrivals (0 until 2+ frames)."""
        if not self._frame_times or time.monotonic() - self._frame_times[-1] > 1.0:
            return 0.0
        if len(self._frame_times) >= 2:
            span = self._frame_times[-1] - self._frame_times[0]
            if span > 0:
                return (len(self._frame_times) - 1) / span
        return 0.0

    def show_image(self, frame_id: int, img: QImage):
        """Display an already-decoded frame (decoding is done once on the GUI
        main thread; bad frames are blocked before reaching here)."""
        self._frame_times.append(time.monotonic())
        if len(self._frame_times) > 30:
            self._frame_times.pop(0)
        self._pixmap = QPixmap.fromImage(img)
        self._frame_id = frame_id
        self.update()

    def show_jpeg(self, frame_id: int, jpeg: bytes):
        """Legacy helper: decodes internally (Qt prints corrupt-frame errors to
        stderr); new code should use show_image instead."""
        img = QImage.fromData(jpeg, "JPEG")
        if img.isNull():
            return
        self.show_image(frame_id, img)

    def clear(self):
        """Reset to the placeholder (no frame, id -1, empty timing window)."""
        self._pixmap = None
        self._frame_id = -1
        self._frame_times.clear()
        self.update()

    def paintEvent(self, _event):
        """Paint the current frame scaled to fit, centered; or the "waiting"
        placeholder text when no frame is present."""
        p = QPainter(self)
        p.fillRect(self.rect(), Qt.black)
        if self._pixmap is not None:
            scaled = self._pixmap.scaled(
                self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            x = (self.width() - scaled.width()) // 2
            y = (self.height() - scaled.height()) // 2
            p.drawPixmap(x, y, scaled)
            p.setPen(Qt.white)
            p.drawText(8, 20, f"{self._current_fps():.1f} fps")
        else:
            p.setPen(Qt.gray)
            p.drawText(self.rect(), Qt.AlignCenter, tr("video_wait"))
        p.end()
