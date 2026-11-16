"""
app.video — 视频显示 widget（QImage 直接解码 JPEG，零 OpenCV 依赖）
"""
import time

from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPixmap, QPainter
from PySide6.QtWidgets import QWidget

from app.i18n import tr


class VideoWidget(QWidget):
    """显示 UDP 8889 收到的 JPEG 帧（等比缩放，居中）。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._pixmap = None
        self._frame_id = -1
        self._frame_times = []   # 最近帧时间戳（滚动窗口，算真实 fps）
        self.setMinimumSize(280, 140)

    def _current_fps(self) -> float:
        if not self._frame_times or time.monotonic() - self._frame_times[-1] > 1.0:
            return 0.0
        """按最近 30 帧的到达时间算滚动 fps（首帧后才有值）。"""
        if len(self._frame_times) >= 2:
            span = self._frame_times[-1] - self._frame_times[0]
            if span > 0:
                return (len(self._frame_times) - 1) / span
        return 0.0

    def show_image(self, frame_id: int, img: QImage):
        """显示已解码帧（解码由 GUI 主线程统一做，坏帧在此之前已被拦截）。"""
        self._frame_times.append(time.monotonic())
        if len(self._frame_times) > 30:
            self._frame_times.pop(0)
        self._pixmap = QPixmap.fromImage(img)
        self._frame_id = frame_id
        self.update()

    def show_jpeg(self, frame_id: int, jpeg: bytes):
        """兼容旧调用：内部解码（Qt 会把损坏帧错误打到 stderr；新代码走 show_image）。"""
        img = QImage.fromData(jpeg, "JPEG")
        if img.isNull():
            return
        self.show_image(frame_id, img)

    def clear(self):
        self._pixmap = None
        self._frame_id = -1
        self._frame_times.clear()
        self.update()

    def paintEvent(self, _event):
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
