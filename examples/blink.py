"""Blink the F407 LED by toggling. peo.led(True) / peo.led(False) set absolute on/off."""
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
