# PEO Rover Team SDK (`peo`)

Updated 2026-11-16. Scripts run on the PC; the S3 routes commands and enforces
its STOP barrier, while the F407 executes motor/sensor operations.

## Install

From the **repository root** (the old `sdk/pyproject.toml` has moved):

```powershell
python -m pip install .
python -m pip install '.[vision]'   # optional numpy/OpenCV for cam.read()
```

The wheel includes both `peo` and `rover_comm`. The GUI Team IDE injects the
source SDK automatically. For source-only scripts outside the GUI, add both the
repository root and its `sdk/` directory to `sys.path`, or install the wheel.

## Minimal control loop

```python
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
        else:
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
```

Calibrate wheel direction and speed before using a moving example. `drive()`
sets all unlisted wheels to zero. Four wheel commands enter the local queue as
one batch, but they are independent operations on the vehicle. Repeated targets
are deliberately resent: local queue acceptance is not execution confirmation.

## Data and camera

Read `peo.mpu`, `peo.vl53`, `peo.wheels`, and `peo.sys`. Sensor data is delivered
at about 25 Hz. `peo.fresh` is a dynamic boolean and includes S3 `link_stale`;
use `peo.fresh` through the module, not a cached `from peo import fresh` value.
Disconnected/no-snapshot data properties return `None`; stale connected values
remain available for inspection and must not be assumed current.

```python
peo.cam.on()
img, seq = peo.cam.read(timeout_ms=100)  # latest fresh BGR array; may repeat a frame
raw, seq = peo.cam.read_raw()           # raw JPEG, no OpenCV needed
peo.cam.set_resolution("VGA")
peo.cam.off()
```

Frames older than one second are not returned as fresh images. `seq` is the
16-bit wire ID; it may wrap or restart. Video OFF/disconnect clears the mailbox.

## Stop and lifecycle

`peo.stop()` waits for a newly received successful S3 STOP ACK and retries once;
if unconfirmed it raises `TimeoutError`. It remains callable after IDE
cancellation. An ACK confirms S3 acceptance, not F407 execution.

IDE Stop immediately requests a priority STOP and marks SDK actions/waits as
cancelled. Pure Python/extension code that never checks the SDK may continue
running; it cannot send ordinary SDK commands after cancellation. The automatic
500 ms PING heartbeat is independent of script progress.

Monitor and SDK share a local control lease. Failed/passive connections release
it, and reconnects discard old queues and snapshots. Config is stored at
`~/.peo_rover_config.json` (override with `PEO_CONFIG_PATH`). Cached PID/resolution
are reapplied after fresh telemetry arrives or the F407 tick resets. Motor
commands and video ON are never automatically restored.

The full current API is [API_REFERENCE.md](API_REFERENCE.md). The nearby PDF is
an older static export and is not the current reference. See [project README](../README.md)
for tests, GUI behavior, and hardware validation limits.
