"""vision_line - vision demonstration: follow a dark line using the camera.

Stops on missing/stale data instead of reusing an old frame.

Copyright (c) 2026 Danny
SPDX-License-Identifier: MIT
"""
import os
import sys

# Make the project root and sdk/ importable so `import peo` works.
_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [_root, os.path.join(_root, "sdk")]
import peo

peo.connect()
try:
    import cv2
    peo.cam.on()
    previous = -1
    while True:
        img, seq = peo.cam.read()   # newest BGR frame + frame id
        # Stop on a missing frame or stale camera/link data.
        if img is None or not peo.cam.fresh or not peo.fresh:
            peo.stop()
            peo.delay(100)
            continue
        # Skip a repeated frame id (no new frame yet).
        if seq == previous:
            peo.delay(20)
            continue
        previous = seq
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, threshold = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        moments = cv2.moments(threshold)
        if moments["m00"] == 0:
            peo.stop()
            continue
        # Horizontal error: line centroid vs image center (pixels).
        error = moments["m10"] / moments["m00"] - img.shape[1] / 2
        # Steer by commanding the two outer wheels; calibrate for your chassis.
        peo.drive(0, 25 + error * 0.3, 2, 25 - error * 0.3)
        peo.delay(20)
except KeyboardInterrupt:
    pass
finally:
    try:
        peo.stop()
    except (peo.NotConnected, TimeoutError):
        pass
    peo.disconnect()
