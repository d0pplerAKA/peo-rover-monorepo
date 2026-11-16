"""Basic distance demonstration; validate wheel direction before driving."""
import os
import sys

_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [_root, os.path.join(_root, "sdk")]
import peo

peo.connect()
try:
    while True:
        if not peo.fresh or not peo.vl53.front_ok:
            peo.stop()
            peo.delay(100)
            continue
        if peo.vl53.front_mm <= 200:
            peo.stop()
            peo.delay(100)
            continue
        peo.drive_all(30)
        peo.delay(50)
except KeyboardInterrupt:
    pass
finally:
    try:
        peo.stop()
    except (peo.NotConnected, TimeoutError):
        pass
    peo.disconnect()
