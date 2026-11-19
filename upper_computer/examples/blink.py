"""blink - blink the F407 LED by toggling it on and off.

peo.led(True) / peo.led(False) set the LED to an absolute on/off state;
peo.led() with no argument toggles it.

Copyright (c) 2026 Danny
SPDX-License-Identifier: MIT
"""
import os
import sys

# Make the project root and sdk/ importable so `import peo` works when this
# example is run directly from the examples/ folder.
_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [_root, os.path.join(_root, "sdk")]
import peo

# Connect to the rover (mDNS hostname by default; pass an IP if mDNS is off).
peo.connect()
try:
    while True:
        peo.led()       # toggle the LED
        peo.delay(100)  # wait 100 ms
except KeyboardInterrupt:
    pass               # Ctrl+C exits the loop
finally:
    peo.disconnect()   # always release the connection
