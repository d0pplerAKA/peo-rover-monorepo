"""Toggle the F407 LED; LED_TOGGLE has no absolute on/off state."""
import os
import sys

_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [_root, os.path.join(_root, "sdk")]
import peo

peo.connect()
try:
    while True:
        peo.led()
        peo.delay(100)
except KeyboardInterrupt:
    pass
finally:
    peo.disconnect()
