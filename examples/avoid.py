"""avoid - basic distance demonstration: drive forward and stop when the front
sensor gets close. Validate wheel direction on your chassis before driving.

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
    while True:
        # Stop whenever data is missing/stale or the front sensor is invalid.
        if not peo.fresh or not peo.vl53.front_ok:
            peo.stop()
            peo.delay(100)
            continue
        # Stop within 200 mm of an obstacle.
        if peo.vl53.front_mm <= 200:
            peo.stop()
            peo.delay(100)
            continue
        peo.drive_all(30)   # all four wheels forward at 30 cm/s (closed loop)
        peo.delay(50)
except KeyboardInterrupt:
    pass
finally:
    try:
        peo.stop()          # try to halt even after Ctrl+C
    except (peo.NotConnected, TimeoutError):
        pass
    peo.disconnect()
