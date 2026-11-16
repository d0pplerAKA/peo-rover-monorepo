"""Vision demonstration: stop on missing/stale data instead of reusing an old frame."""
import os
import sys

_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [_root, os.path.join(_root, "sdk")]
import peo

peo.connect()
try:
    import cv2
    peo.cam.on()
    previous = -1
    while True:
        img, seq = peo.cam.read()
        if img is None or not peo.cam.fresh or not peo.fresh:
            peo.stop()
            peo.delay(100)
            continue
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
        error = moments["m10"] / moments["m00"] - img.shape[1] / 2
        # Wheels 1/3 are explicitly sent zero by drive(); calibrate for your chassis.
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
