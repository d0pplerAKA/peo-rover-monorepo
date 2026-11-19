"""Minimal video-stream viewer: connect, stream, show frames with FPS.

The simplest possible client for the PEO Rover video stream:
connects to the ESP32-S3 (mDNS or IP), turns the camera on, and shows the
frames with a rolling FPS counter. No GUI, no SDK — just rover_comm + cv2.

Video transport: TCP 8890 (2026-11-16 redesign; UDP retired), frames arrive
complete — the receiver thread decodes JPEG to BGR and keeps only the latest.

Usage:
    python tests/video_stream_minimal.py [host] [resolution]

    host        ESP32 address (default "peo_rover.local"; pass the IP if
                mDNS is unavailable on your machine)
    resolution  QQVGA / QVGA / VGA / SVGA (default VGA)

Controls (type in the CONSOLE — no need to click the video window):
    s            save current frame's raw JPEG to bad_frames/ (forensics)
    q            quit (also: ESC when the video window has focus, or Ctrl+C)

Rules:
    - The ESP32 must be powered on (AP "PEO_Rover").
    - The GUI must NOT be connected at the same time
      (single-client rule: one controller per vehicle).

Copyright (c) 2026 Danny
SPDX-License-Identifier: MIT
"""
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import cv2
import numpy as np

try:
    import msvcrt   # Windows console key polling (works without window focus)
except ImportError:
    msvcrt = None   # POSIX: rely on cv2.waitKey (window focus required)

from rover_comm.client import RoverClient
from collections import deque

_FPS_WINDOW = 30   # rolling window (frames) for FPS calculation
_RING_FRAMES = 96  # forensics ring: last ~4 s of raw JPEGs @24fps (~6 MB)


def _console_key() -> str:
    """Non-blocking console key read. Returns '' when no key is pending."""
    if msvcrt is None or not msvcrt.kbhit():
        return ""
    ch = msvcrt.getch()
    if ch in (b"\x00", b"\xe0"):        # arrow/function key prefix
        msvcrt.getch()
        return ""
    try:
        return ch.decode("ascii").lower()
    except UnicodeDecodeError:
        return ""


def main(host: str = "peo_rover.local", res: str = "QQVGA") -> None:
    client = RoverClient()
    latest = [None]   # latest decoded frame (BGR ndarray or None)
    latest_fid = [-1]  # fid of the latest frame (fps counts only NEW frames)
    ring = deque(maxlen=_RING_FRAMES)   # rolling raw-JPEG forensics ring

    def on_frame(frame_id: int, jpeg: bytes) -> None:
        """Receiving thread: decode asynchronously, keep only the latest."""
        buf = np.frombuffer(jpeg, np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if img is not None:
            latest[0] = img
            latest_fid[0] = frame_id
            ring.append((frame_id, jpeg))

    client.on_frame = on_frame

    try:
        client.connect(host)   # TCP 8888 first; TCP 8890 only after video_on().
        client.wait_ready(1.0)
    except (OSError, ConnectionError, RuntimeError) as e:
        print(f"connect failed: {e}")
        print("check: ESP32 powered on? laptop on PEO_Rover WiFi? "
              "GUI disconnected? mDNS working (or pass the IP)?")
        sys.exit(1)

    client.send_json({"cmd": "CAM_RES", "res": res})
    client.video_on()
    print(f"streaming {res} @ {host} — ESC or Ctrl+C to stop")

    times = []
    last_shown = -1      # fid already counted/shown (fps = NEW-frame rate)
    try:
        while True:
            img = latest[0]
            if img is not None and latest_fid[0] != last_shown:
                last_shown = latest_fid[0]
                now = time.monotonic()
                times.append(now)
                if len(times) > _FPS_WINDOW:
                    times.pop(0)
                fps = ((len(times) - 1) / (times[-1] - times[0])
                       if len(times) >= 2 else 0.0)
                disp = img.copy()
                cv2.putText(disp, f"{fps:.1f} fps", (8, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("PEO Rover video", disp)
                if (cv2.waitKey(1) & 0xFF) == 27:   # ESC (only when window focused)
                    break
            else:
                cv2.waitKey(10)   # keep the window responsive before 1st frame
            key = _console_key()  # console keys work regardless of window focus
            if key in ("q", "\x1b"):
                break
            if key == "s":
                # Post-hoc forensics: even after a glitch you can still save the
                # last ~4 s of raw JPEGs from the ring (including frames that
                # already scrolled past).
                snap = list(ring)
                if not snap:
                    print("ring empty — no frames received yet")
                    continue
                d = os.path.join(ROOT, "bad_frames")
                os.makedirs(d, exist_ok=True)
                stamp = int(time.time() % 100000)
                for i, (fid, jpeg) in enumerate(snap):
                    with open(os.path.join(d, f"f{fid}_{stamp}_{i:03d}.jpg"),
                              "wb") as fh:
                        fh.write(jpeg)
                print(f"saved {len(snap)} frames (f{snap[0][0]}..f{snap[-1][0]}) "
                      f"to bad_frames/")
                ring.clear()   # avoid dumping the same batch twice
    except KeyboardInterrupt:
        pass
    finally:
        client.video_off()
        client.close()
        cv2.destroyAllWindows()
        print("stopped")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "peo_rover.local",
         sys.argv[2] if len(sys.argv) > 2 else "VGA")
