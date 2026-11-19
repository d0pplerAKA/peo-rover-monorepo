"""Latest-frame camera mailbox for the team SDK.

Raw reads return JPEG bytes (no decoder needed); BGR reads decode lazily with
cv2. Only the newest frame is kept, so a slow consumer never lags on stale
imagery.

Copyright (c) 2026 Danny
SPDX-License-Identifier: MIT
"""
import math
import threading
import time

from ._bridge import _bridge, CamOff, ScriptStopped, _stop_requested
from ._i18n import tr

_FRAME_FRESH_MS = 1000        # a frame is "fresh" if received within 1 s
_RESOLUTIONS = ('QQVGA', 'QVGA', 'VGA', 'SVGA')


class Cam:
    """Single-slot mailbox that keeps only the latest decoded video frame.

    Receiving threads write raw JPEG bytes; readers can either grab the raw
    bytes (read_raw) or decode to a BGR ndarray (read, needs cv2).

    Attributes:
        _latest: (jpeg_bytes, frame_id) of the newest frame, or (None, -1).
        _last_t: monotonic time the newest frame arrived (seconds).
        _lock: condition guarding _latest/_last_t/_on.
        _decode_lock: serializes cv2 decoding (decode only once per frame).
        _decoded_raw: the raw bytes the cached _decoded image came from.
        _decoded: cached decoded BGR ndarray (or None).
        _on: whether video streaming is currently enabled.
        on_new_frame: optional callback called (frame_id, jpeg) on arrival.
    """

    def __init__(self):
        self._latest = (None, -1)
        self._last_t = 0.0
        self._lock = threading.Condition()
        self._decode_lock = threading.Lock()
        self._decoded_raw = self._decoded = None
        self._on = False
        self.on_new_frame = None
        _bridge.client.on_frame = self._on_frame

    def _on_frame(self, frame_id, jpeg):
        """Receiving-thread hook: store the newest frame, wake waiters, and fire
        the on_new_frame callback (its errors are swallowed so they can never
        break the receive chain)."""
        with self._lock:
            if not self._on:
                return
            self._latest = (jpeg, frame_id)
            self._last_t = time.monotonic()
            self._lock.notify_all()
        callback = self.on_new_frame
        if callback:
            try:
                callback(frame_id, jpeg)
            except Exception:
                pass

    def _reset(self):
        """Clear all frame state (called on disconnect) and un-set video."""
        with self._lock:
            self._on = False
            self._latest = (None, -1)
            self._last_t = 0.0
            self._lock.notify_all()
        with self._decode_lock:
            self._decoded_raw = self._decoded = None

    def _latest_raw(self, timeout_ms, *, cancel=True):
        """Wait up to timeout_ms for a fresh raw frame; return (jpeg, fid) or
        (None, -1). If cancel is True, a requested script stop raises
        ScriptStopped instead of blocking."""
        if not math.isfinite(timeout_ms) or timeout_ms < 0:
            raise ValueError('timeout_ms must be finite and non-negative')
        deadline = time.monotonic() + timeout_ms / 1000
        with self._lock:
            while True:
                if cancel and _stop_requested.is_set():
                    raise ScriptStopped(tr('script stopped', '脚本已停止'))
                if not self._on:
                    return None, -1
                # Inspect once even for timeout=0; never recycle indefinitely
                # stale imagery.
                if self._latest[0] is not None and time.monotonic() - self._last_t < _FRAME_FRESH_MS / 1000:
                    return self._latest
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None, -1
                self._lock.wait(min(remaining, 0.05))

    def on(self):
        """Ask the ESP32 to start streaming video and mark the mailbox on."""
        _bridge.send({'cmd': 'VIDEO_ON'})
        with self._lock:
            self._on = True

    def off(self):
        """Ask the ESP32 to stop streaming and reset the mailbox."""
        try:
            _bridge.send({'cmd': 'VIDEO_OFF'})
        finally:
            self._reset()

    def set_resolution(self, res):
        """Switch camera resolution (QQVGA/QVGA/VGA/SVGA) and cache it."""
        if res not in _RESOLUTIONS:
            raise ValueError('resolution must be QQVGA, QVGA, VGA or SVGA')
        _bridge.send({'cmd': 'CAM_RES', 'res': res})
        with self._lock:
            self._latest, self._last_t = (None, -1), 0.0
        _bridge._config['cam_res'] = res
        _bridge._save_config()

    def read(self, timeout_ms=500.0):
        """Newest fresh BGR array and wire frame ID; timeout gives (None, -1).

        Repeated reads cache decoding but return independent arrays. The ID can
        wrap or restart; it is not a monotonic count across video sessions.
        """
        if not self._on:
            raise CamOff(tr('camera off: call peo.cam.on() first', '摄像头未开启：先 peo.cam.on()'))
        raw, seq = self._latest_raw(timeout_ms)
        if raw is None:
            return None, -1
        try:
            import cv2
            import numpy as np
        except ImportError as exc:
            raise RuntimeError('cam.read requires opencv-python and numpy') from exc
        with self._decode_lock:
            if raw is not self._decoded_raw:
                try:
                    decoded = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
                except cv2.error:
                    decoded = None
                self._decoded_raw, self._decoded = raw, decoded
            return (self._decoded.copy(), seq) if self._decoded is not None else (None, -1)

    def read_raw(self, timeout_ms=0.0):
        """Fresh raw JPEG and wire ID; off/no frame returns (None, -1)."""
        return self._latest_raw(timeout_ms, cancel=False)

    @property
    def fresh(self):
        """True if video is on and a frame arrived within the freshness window."""
        with self._lock:
            return bool(self._on and self._latest[0] is not None
                        and time.monotonic() - self._last_t < _FRAME_FRESH_MS / 1000)

    @property
    def seq(self):
        """The wire frame id of the newest frame (-1 if none)."""
        with self._lock:
            return self._latest[1]


cam = Cam()
