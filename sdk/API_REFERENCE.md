# peo SDK — Current API Reference

Updated 2026-11-17. This document describes the checked-in implementation.
Install from the repository root; see [README.md](README.md) and
[wire protocol](../PROTOCOL.md). The old PDF is not synchronized with this revision.

## Lifecycle

### `connect(host="peo_rover.local") -> None`

Acquire the local control lease (loopback TCP 8892) and open TCP 8888.
TCP 8890 is opened only after cam.on(); cam.off() closes it and stops retries.
The client sends a one-way safety heartbeat every 500 ms (no ACK or RTT).
While video is enabled, read timeout is 5 s and reconnect backoff is 2 s;
video failure leaves the command channel up. Repeated connect to the same active host is a no-op;
changing host closes the old session first. Failure releases sockets and the
lease. A pending IDE stop raises `ScriptStopped`.

The bridge waits for fresh telemetry before applying cached PID/resolution.
Ordinary action calls wait up to 500 ms for a valid generation before sending.
They can fail if the first telemetry has not arrived. The connection itself does
not guarantee a usable sensor snapshot; inspect `peo.fresh`.

### `disconnect() -> None`

Close TCP/UDP and wake workers; discard queued commands, snapshots and camera
frames; stop the session monitor and release the lease. The S3 handles TCP
connection loss as a STOP trigger. This is not an execution acknowledgment.

### `delay(ms) -> None`

Wait for a finite, non-negative number of milliseconds, interruptibly through a
threading event. Scheduler jitter applies; no 1 ms timing guarantee is made.
Raises `ScriptStopped` when the IDE cancellation flag is set. It does not modify
Windows timer resolution.

## Read-only data

### `peo.fresh -> bool`

True only when connected, telemetry arrived within 250 ms, and S3 did not mark
`link_stale`. S3's current UART-age threshold is 500 ms; fresh is not a guarantee
that each individual sensor has produced a new measurement. Use per-sensor
validity too. Always read `peo.fresh` dynamically; importing a bool into a local
variable captures its value at that moment.

| Namespace / attribute | Type / unit |
| --- | --- |
| `mpu.quat` | tuple of four floats (w, x, y, z) |
| `mpu.accel` | tuple of three floats, g |
| `vl53.front_mm / left_mm / right_mm` | distance, mm |
| `vl53.front_ok / left_ok / right_ok` | per-distance validity bool |
| `wheels.speed_cm_s` | four-element list, cm/s |
| `wheels.odometer` | four encoder counts |
| `wheels.current_mA` | four signed currents, mA |
| `wheels.bus_voltage_mV` | four bus voltages, mV |
| `sys.cpu0 / cpu1` | S3 CPU utilization percent |
| `sys.ram` | internal RAM used percent |
| `control_mode` | 0 = closed loop (speed PI), 1 = open loop (direct PWM) |
| `fw_version` | F407 firmware version number |
| `s3_fw` | S3 firmware version number |
| `lock` | 0 = manual (gamepad), 1 = auto (upper computer) |

Sensor properties return `None` when disconnected or before the first snapshot.
A stale connected snapshot is retained for inspection; check `fresh` before
using it for control. Missing distance fields are invalid, not valid zero ranges.
S3 JSON arrives at approximately 20 Hz. Resource values are read on demand;
reconnect does not spawn accumulating system-poll threads.

## Actions

All ordinary actions check connection/cancellation and return after local queue
acceptance. They do not wait for execution ACKs. Invalid numeric arguments are
rejected before sending. STOP has the separate confirmation behavior below.

### `drive(*pairs) -> None`

Pass `(motor, speed_cm_s)` pairs. Motor must be an integer 0–3 (not bool), cannot
repeat, and arguments must be paired. **Only the specified wheels are sent (data
update); unspecified wheels keep their current F407 target.**

```python
peo.drive(1, 30)                    # only M1 updated; others keep current
peo.drive(0, 30, 1, 30, 2, -30, 3, -30)
peo.drive()                        # ValueError: at least one pair required
```

Speeds must be finite (bool rejected) and fit signed int32 after multiplying by
100. Conversion uses Python round-to-nearest then integer conversion. All
specified wheels are validated before one local queue entry is published. On
local queue rejection the bridge requests STOP and raises `LinkLost`. The four
F407 operations are not atomic (each is a separate MOTOR_CLOSED). Halt all wheels
with `peo.stop()`.
Repeated targets are resent, so a previous STOP, reset or rejected command cannot
leave a deduplication cache suppressing an intentional new drive request.

### `drive_all(speed_cm_s) -> None`

Equivalent to specifying the same closed-loop target for all four wheels.

### `motor(*pairs) -> None`

Open-loop PWM drive that bypasses the speed PI loop — the fallback for when the
closed-loop PID parameters are not working. Pass `(motor, speed)` pairs; motor
must be an integer 0–3 (not bool), cannot repeat, and arguments must be paired.
**Only the specified wheels are sent (data update); unspecified wheels keep their
current F407 PWM.**

`speed` is **per-mille** (-1000 = full reverse, +1000 = full forward, 0 = coast),
**not cm/s** (that is `drive()`). Positive = forward, negative = reverse.

```python
peo.motor(0, 500, 1, 500, 2, 500, 3, 500)      # four wheels half speed, open loop
peo.motor(0, -800, 1, -800, 2, -800, 3, -800)  # reverse
peo.motor(1, 800)                              # only M1 updated; others keep
```

Sending `MOTOR` switches the F407 into open-loop mode; `MOTOR_CLOSED`
(`drive`/`drive_all`) switches back to closed loop, and `STOP_ALL` returns to
closed loop. Encoder speed feedback keeps reporting in open-loop mode. Invalid
speed (non-int, bool, or outside -1000..1000) raises `ValueError`.

### `set_pid(*args, kp=None, ki=None) -> None`

Use `set_pid(kp=0.3)`, `set_pid(ki=0.01)` or `set_pid(0.3, 0.01)`. A single
positional argument, too many arguments, mixed positional/keyword values, or no
values are errors. Values must fit finite float32. Omitted gain uses the cached
value, then defaults kp=0.25 / ki=0.005. Accepted requests update the config cache;
this does not prove the MCU applied the gains.

### `stop() -> None`

Clear locally pending ordinary work and enqueue priority STOP even when the
normal queue is full. Wait up to 300 ms for a **newly received successful
`{"ack":"STOP_ALL","status":0}`**, then retry once. Return on confirmation;
raise `TimeoutError` if neither attempt is confirmed or the connection closes
while waiting. If initially disconnected, raise `NotConnected`.

This method deliberately remains usable from `except ScriptStopped` cleanup.
An old `last_cmd_status=0` sensor value is not a STOP acknowledgment. The wire
protocol has no request identifier, so ACK correlation is limited to receipt
after the request in this TCP session. S3 acceptance is not F407 execution. One
already committed batch may precede STOP. Physical cutoff/watchdogs are separate.

### `led(on=None) -> None`

`on=True` sends `LED_ON` (absolute on), `on=False` sends `LED_OFF` (absolute off),
`on=None` (default) sends `LED_TOGGLE`. Non-bool/non-None arguments are rejected.

### `ws2812(r, g, b, brightness=100) -> None`

S3-local RGB request. RGB values must be integers 0–255 and brightness an integer
0–100, excluding bool. Out-of-range values raise `ValueError`; they are not
silently clamped or narrowed. The S3 converts RGB to GRB/MSB-first RMT output.

### `reset_odometer() -> None`

Request zeroing all four encoder odometers.

### `sys_reset() -> None`

Request an F407 reset; it does not reset the S3. Freshness recovery and backward
F407 tick changes trigger cached configuration restoration. Normal uint32 tick
rollover is not treated as a reset. No motor target is restored automatically.

## Camera

### `cam.on()` / `cam.off()`

Request VIDEO_ON/OFF. ON marks the local camera enabled after queue acceptance;
it does not guarantee a frame has arrived. OFF clears local raw/decoded frames
and wakes waiters even if sending fails. Disconnect also clears frames. Actual
initialization, exposure warmup and network delivery determine first-frame time.

### `cam.set_resolution(res)`

Accepts `QQVGA`, `QVGA`, `VGA`, `SVGA` only. Queues CAM_RES, clears the old raw
frame, and persists the requested resolution. Queue acceptance is not evidence
of successful hardware switching; firmware can retry or revert.

### `cam.read(timeout_ms=500.0) -> (image, seq)`

Return the latest frame younger than one second as a BGR numpy ndarray and its
16-bit wire ID. Returns `(None, -1)` on timeout or decode failure. It may return
the same frame again while still fresh; it does not wait specifically for a
sequence change. `timeout_ms=0` inspects the mailbox once. Repeated reads reuse a
decode cache and return a copy so caller mutation cannot contaminate other reads.
Requires the `vision` extra. Raises `CamOff` before ON, `ScriptStopped` on
cancellation, and `ValueError` for negative/non-finite timeout.

### `cam.read_raw(timeout_ms=0.0) -> (bytes, seq)`

Return latest fresh raw JPEG without OpenCV. OFF, no frame or timeout returns
`(None, -1)`. GUI polling uses this non-cancelling path; invalid timeout still
raises `ValueError`.

### `cam.fresh` / `cam.seq`

`fresh` means ON and a frame arrived within one second. `seq` is the latest wire
ID, or -1 after reset; it can wrap/restart and is not a global monotonic counter.
Marker validation is not full entropy validation; decoders may reject or conceal
corruption. Inspect frames and control-loop behavior on hardware.

## Errors, persistence and cancellation

| Exception | Meaning |
| --- | --- |
| `NotConnected` | no active control connection |
| `LinkLost` | missing usable generation, disconnected or full queue |
| `ScriptStopped` | cooperative cancellation; ordinary actions/waits rejected |
| `CamOff` | decoded camera read before ON |
| `TimeoutError` | STOP acceptance was not confirmed |
| `ConnectionError` / `RuntimeError` | connection/bind failure or busy local lease |
| `ValueError` / `TypeError` | invalid arguments |

Catch exception types rather than message text; legacy messages follow GUI
language where translated, while some validation errors are English.

Config defaults to `~/.peo_rover_config.json`; `PEO_CONFIG_PATH` overrides it.
Writes replace a temporary file atomically. Malformed configs are ignored;
only valid cached PID/resolution values are restored. Test scripts use temporary
configuration paths and do not delete personal configuration.

IDE Stop immediately queues STOP and sets cancellation. It does not kill a
Python thread. Scripts and active/multiline shell sessions cannot run together.
The 500 ms one-way safety heartbeat continues independently of user code, so heartbeat does
not detect a stalled algorithm. Always check sensor/frame freshness in algorithms.
