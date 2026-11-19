# PEO Rover — Upper Computer (PySide6 GUI + peo SDK)

> **One-line role**: The upper computer is the "console" between the team and the rover — see the rover (video), see its data (telemetry), send commands (GUI), and above all write Python scripts that drive it in real time (peo SDK).

---

## 1. Project Overview

The upper computer (Python ≥3.10, PySide6) is layered in three parts:

- **app/**: PySide6 desktop GUI — Tab 1 "Monitor" (video/dashboard/emergency stop), Tab 2 "Team IDE" (script editor + console);
- **rover_comm/**: communication library (importable by teammates) — command encoding, RoverClient (TCP 8888 command/telemetry + TCP 8890 video), control lock;
- **sdk/peo/**: the teammate SDK (single entry point `peo`) — action functions (drive/stop/set_pid…) + read-only sensor namespaces + on-disk config cache.

Connection model: the PC joins the S3 hotspot as WiFi STA; TCP 8888 carries commands/telemetry, TCP 8890 carries video; a heartbeat is sent one-way every 0.5 s; every normal command automatically carries the latest telemetry generation (gen).

## 2. Why This Project Exists

1. **Teammates have no embedded background**: driving a rover is abstracted into a set of Python action functions — teammates write scripts in a blocking style ("act → delay → act") that stream commands over WiFi in real time.
2. **Vision lives here**: video frames arrive in `peo.cam` as ndarrays (cv2), so vision algorithms (line following, obstacle avoidance) run on the PC with compute the MCU could never provide.
3. **Field debugging console**: the GUI dashboard (distance/speed/current/attitude/CPU), the emergency-stop button, and the IDE console make on-site troubleshooting visible.
4. **Configuration management**: PID/resolution are cached in `~/.peo_rover_config.json` and re-sent automatically after reconnect or F407 reset — tuned parameters are never lost.

## 3. What It Does

| Feature | Implementation |
|---|---|
| Monitor tab | connect, resolution select, video toggle, sensor dashboard, emergency-stop button (the only command button on Tab 1) |
| Team IDE tab | script edit/run/stop, persistent shell REPL, latest video, unified console |
| peo SDK | action functions + read-only sensor namespaces + blocking `delay()` + single-instance control lock |
| Video RX | TCP 8890, 8-byte header (0xCAFE+fid+len) + JPEG, structural guard + bad-frame drop |
| Heartbeat | one-way `{"type":"heartbeat"}` every 0.5 s (no ACK, no timestamp — only renews the watchdog) |
| Emergency stop | button + spacebar shortcut; SDK `peo.stop()` waits for a *new* S3 ACK (0.3 s × 2 retries) |
| Config cache | PID + resolution in `~/.peo_rover_config.json` (override via PEO_CONFIG_PATH) |
| i18n | GUI zh/en toggle (default English, international teammates), SDK error messages follow the GUI language |

## 4. Connections to the Other Three Nodes

```
        ┌──────────────────────────────────┐
        │   Upper Computer (PC, this) ★     │
        │   PySide6 GUI + peo SDK scripts   │
        └───────────────┬──────────────────┘
              WiFi STA  │ TCP 8888 cmd/telemetry
                         │ TCP 8890 video
        ┌───────────────▼──────────────────┐
        │     ESP32-S3 (WiFi bridge)        │
        └───────────────┬──────────────────┘
                         │ USART2
        ┌───────────────▼──────────────────┐
        │   STM32F407 (main controller)     │
        └───────────────┬──────────────────┘
                         │ USART3
        ┌───────────────▼──────────────────┐
        │   ESP32-C3 (gamepad BLE bridge)   │
        └──────────────────────────────────┘
```

- **↔ ESP32-S3**: the only exit — 8888 commands/telemetry, 8890 video; the heartbeat feeds the S3's watchdog.
- **↔ STM32F407 / ESP32-C3**: no direct link, everything relays through the S3; the on-vehicle `lock` field (manual/autonomous) is display-only here (the lock badge).

## 5. Main Implementation

### 5.1 Module layering

| Layer | File | Duty |
|---|---|---|
| GUI | app/gui.py | MainWindow v2: Tab 1 monitor + Tab 2 IDE + language toggle + emergency stop |
| GUI | app/video.py | VideoWidget decodes JPEG with QImage directly (no OpenCV dependency) + 30-frame fps |
| GUI | app/i18n.py | zh/en dictionaries + tr() |
| Comm | rover_comm/protocol.py | pure data layer: command IDs/CRC16/frame packing/SensorData |
| Comm | rover_comm/client.py | RoverClient: three threads (send/tcp/heartbeat) + video thread + gen machinery |
| Comm | rover_comm/control.py | ControlLock: local lease lock on 127.0.0.1:8892 |
| SDK | sdk/peo/__init__.py | action functions + read-only namespaces |
| SDK | sdk/peo/_bridge.py | connection bridge: RoverClient + 50 Hz snapshot + config cache + re-send monitor thread |
| SDK | sdk/peo/cam.py | single-slot latest-frame mailbox + cv2 lazy decode |

### 5.2 peo SDK action list

| Action | Semantics |
|---|---|
| `connect(host="peo_rover.local")` | connect 8888+8890+heartbeat+lease; auto re-sends cached PID/resolution |
| `disconnect()` | disconnect and release the lease |
| `delay(ms)` | blocking wait; raises ScriptStopped if the stop flag is set |
| `led(on=None)` | True→LED_ON / False→LED_OFF / None→LED_TOGGLE |
| `drive_all(speed_cm_s)` | all wheels at one closed-loop speed (**unit cm/s**, MOTOR_CLOSED) |
| `drive(*pairs)` | variadic (motor, speed_cm_s); sends only the named motors (data update); negative = reverse |
| `motor(*pairs)` | open-loop PWM **per-mille -1000..1000** (MOTOR); mutually toggles open/closed-loop mode with drive |
| `set_pid(kp=…, ki=…)` | PI shared by all wheels; merges cache defaults (kp 0.25 / ki 0.005); sends command + writes cache |
| `stop()` | priority STOP + waits for a **new** S3 ACK (0.3 s × 2), raises TimeoutError if unconfirmed |
| `reset_odometer()` | zero all four odometers |
| `ws2812(r,g,b,brightness=100)` | status LED color (S3-local command) |
| `sys_reset()` | F407 soft reset (S3 keeps running); cached config auto re-sent after recovery |

Read-only namespaces: `mpu.quat/accel`, `vl53.front_mm/left_mm/right_mm/*_ok`, `wheels.speed_cm_s/odometer/current_mA/bus_voltage_mV`, `sys.cpu0/cpu1/ram`, `cam`, and top-level `fresh/control_mode/fw_version/s3_fw/lock` (None when no data).

### 5.3 Connection & safety machinery

- **Generation (gen)**: normal commands automatically carry the latest telemetry generation; STOP_ALL is sent alone (clears the queue, invalidates the generation, clears ready); telemetry updates the generation only when it is valid and not blocked by a STOP.
- **Heartbeat**: 0.5 s one-way, renews the S3 watchdog only (S3 auto-stops after 2 s of silence) — **the heartbeat does not prove the algorithm is making progress**; the algorithm must check `fresh`/video freshness itself.
- **fresh semantics**: connected + snapshot present + no link_stale + within 250 ms of the latest snapshot (`FRESH_MS=250`).
- **Control lock**: `ControlLock` (127.0.0.1:8892) — GUI and SDK scripts are mutually exclusive on this machine; switching to Tab 2 auto-disconnects the GUI.
- **Config re-send monitor**: a background thread polls every 0.1 s for the fresh recovery edge (after connect / after F407 reset / after link_stale clears) and re-sends cached PID + CAM_RES.

### 5.4 Video reception

8-byte big-endian header (magic 0xCAFE + frame_id + JPEG length) + JPEG body; frame assembly lives in `RoverClient._video_loop` (read the header exactly, then the body exactly); bad magic or length outside 1..512 KB → drop the stream and reconnect (2 s backoff); frames pass `jpeg_structure_ok` (SOI/SOF/SOS/EOI, width/height ≤5M pixels) before decoding. The GUI Tab 1 takes the latest frame from a mailbox; Tab 2 goes through `peo.cam` (single-slot latest frame + 1 s freshness window + cv2 lazy decode).

## 6. What Each Feature Affects and Constrains

- **The two speed units must not be mixed**: `drive/drive_all` is **cm/s** (closed-loop, MOTOR_CLOSED); `motor` is **per-mille ±1000** (open-loop, MOTOR). Sending `motor` switches the F407 to open-loop mode; sending `drive` switches back — the two semantics are mutually exclusive.
- **A STOP ACK only means "S3 received it"**: the protocol has no request ID, so `peo.stop()` waits for the S3's confirmation, not for the F407 to have acted; the GUI button/spacebar does not wait at all. The real emergency stop is the S3 STOP latch plus the F407's independent deadman working together.
- **The heartbeat does not keep an algorithm alive**: a pure CPU busy-loop script cannot be force-killed (the IDE Stop is a cooperative cancel flag); the heartbeat only prevents the S3 from false-tripping — a moving rover still relies on the S3 heartbeat timeout or a physical power cutoff.
- **Single-client hard constraint**: local ControlLock + the S3's `PEO_NET_MAX_CLIENTS=1` double-lock the connection; concurrent scripts are refused (multi-script concurrency is explicitly forbidden by the project).
- **Video freshness window is 1 s**: frames older than 1 s are no longer returned by `cam.read()` (`_FRAME_FRESH_MS=1000`); vision algorithms must check `cam.fresh`.
- **Config-cache write failures are swallowed**: `_save_config` only logs on failure — a teammate script never aborts over a disk issue, but the parameter may silently fail to persist.
- **Telemetry cadence**: the F407 uploads at 50 Hz, the S3 telemetry JSON is 20 Hz in autonomous mode and 2 Hz in manual mode — `fresh` (250 ms window) still holds at 2 Hz, but in manual mode the effective sensor refresh rate is only 2 Hz.

## 7. Tech Stack & Run

- Python ≥3.10 + PySide6 + numpy + opencv-python (optional extra `[vision]`).
- Run: `python -m pip install -r requirements.txt` → `python -m app.main` (`run.bat` holds the local absolute path).
- Tests: `python tests/regression_test.py` (32 offline cases), `python tests/sdk_test.py`, `python tests/smoke_test.py`.

## 8. License

Copyright (c) 2026 Danny — MIT License (see `LICENSE`).
