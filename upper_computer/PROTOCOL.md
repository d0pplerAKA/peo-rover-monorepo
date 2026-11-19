# PEO Rover Communication Protocol Definition (PROTOCOL v2.0)

> Implementation alignment revision: 2026-11-16. The TCP/UDP sections have been checked against the current S3 source and the host-side implementation.
> The UART layout keeps the original convention and has been checked against the local S3 headers; this round did not run F407 firmware or hardware acceptance.
> Source baseline: `peo_rover/COMPONENTS/ESP_LINK/include/esp_link.h` (v2.26) + `SENSOR_HUB/sensor_hub.h`.
> **v2.0 (2026-11-15)**: the C3 gamepad bridge, the macro engine, and the GUI algorithm plugins are all deprecated (noted in §2.2);
> The only path for team-member automation = the §2.4 team-member programming model (native Python scripts on the host PC).

---

## 0. System Architecture

```
                                                   WiFi AP
 [Host Python (PySide6 GUI / peo SDK scripts)] <─────────────> [ESP32-S3]
         ▲                                                      │  ▲
         └────────── TCP 8890: video stream (JPEG push) ────────┘  │
                                                                  │
                                                          USART2 @ 921600 (PD5=TX / PD6=RX)
                                                          esp_link frames (see §1)
                                                                  │
                                                              [F407 main controller] (sensor hub / motors / command queue)
                                                                  ▲
                                                          USART3 @ 115200 (PD8=TX / PD9=RX)
                                                          c3_gamepad frames (see §1.4)
                                                                  │
 [Xbox gamepad] ──── BLE ────> [ESP32-C3] (gamepad bridge, sends raw stick state, 100Hz)
```

> **2026-11-18 manual-control link (Plan A)**: Xbox gamepad via ESP32-C3 BLE bridge → USART3 wired directly to F407,
> sends raw stick state frames (100Hz); the F407 side parses + lock arbitration + stick mapping (§1.4). Physically isolated from the WiFi/autonomous link.
> **2026-11-15 system convergence**: the ESP32 macro engine (macro_engine) and the GUI algorithm plugins (algorithms/) are deprecated;
> Team-member automation = the §2.4 team-member programming model (native Python scripts on the host PC; the ESP32 handles bridging and the stop barrier).

| Link | Physical layer | Protocol | Direction |
|---|---|---|---|
| ESP32-S3 ↔ F407 | USART2 921600 | esp_link frames (§1) | Bidirectional |
| ESP32-S3 ↔ Host PC | WiFi (STA, connects to the laptop hotspot) | TCP 8888 JSON commands (§2) + TCP 8890 video (§3) | Bidirectional / one-way |
| ESP32-C3 → F407 | USART3 115200 | c3_gamepad frames (§1.4) | One-way (gamepad state, 100Hz) |

### 0.1 Team-Member Resource Inventory (all resources available to the host PC)

Resources accessible to host-side code (`peo` SDK scripts / GUI) are listed below; see the corresponding sections for the data paths:

| # | Resource | Real-time data stream | Control / operation |
|---|---|---|---|
| 1 | MPU6050 (attitude + acceleration) | ~20 Hz JSON `imu.quat[4]` / `imu.accel_g[3]` (§2.1) | None (read-only) |
| 2 | INA3221 (four-channel current + bus voltage) | ~20 Hz JSON `power.current_mA[4]` / `power.bus_voltage_mV[4]` (§2.1) | None (read-only) |
| 3 | Encoders (speed + odometer) | ~20 Hz JSON `wheels.speed_d100[4]` (0.01 cm/s) / `wheels.odometer[4]` (§2.1) | **Reset odometer**: `{"cmd": "ODOMETER_RESET"}` (§1.3 0x07) |
| 4 | Four-motor PWM | `wheels.speed_d100[4]` echo | Open-loop: `MOTOR`; closed-loop speed: `MOTOR_CLOSED`; e-stop: `STOP_ALL` (§2.1) |
| 5 | Motor PI parameters | None | `{"cmd": "PID", "kp": .., "ki": ..}` (shared gains across all four wheels, §1.3 0x04) |
| 6 | Camera | TCP 8890 JPEG video frames (§3) → `peo.cam.read()` / `RoverClient.on_frame` | None (read-only; resolution hot-switched by the host `CAM_RES`) |

Python-side counterparts: `SensorData` fields (§4), `rover_comm.client.RoverClient` (network layer),
`peo` SDK (§2.4 team-member programming model, the team members\' only interface).

---

## 1. Link 1: esp_link Frame Protocol (USART2, F407 ↔ ESP32-S3)

### 1.1 Frame Format (all multi-byte fields little-endian)

```
┌────────┬─────┬──────────────┬─────────┐
│ SOF    │ LEN │ PAYLOAD      │ CRC16   │
│ 0xAA 0x55 │ 1B  │ LEN bytes     │ 2B (LSB 1st)    │
└────────┴─────┴──────────────┴─────────┘
```

- **SOF**: `0xAA 0x55` (high byte first)
- **LEN**: 1 byte, payload length
- **CRC16-CCITT-FALSE**: polynomial `0x1021`, init `0xFFFF`, covers **all bytes from SOF through PAYLOAD** (excluding the CRC itself), sent **least-significant byte first** at the frame tail.

### 1.2 Direction 1 (F407 → ESP32-S3, ~50 Hz continuous reporting): PAYLOAD = SensorData_t (116 B)

F407-side serialization = **raw memory copy of the whole struct (native C layout, padding included)**.
Total frame length = 5 + 116 = 121 bytes.

**SensorData_t byte layout (Cortex-M4 ARM EABI, 4-byte alignment, byte-by-byte offset table):**

| Offset | Size | Field | Type | Description |
|---:|---:|---|---|---|
| 0 | 2 | dist_front_mm | uint16 LE | Front distance mm |
| 2 | 2 | dist_left_mm | uint16 LE | Left distance mm |
| 4 | 2 | dist_right_mm | uint16 LE | Right distance mm |
| 6 | 1 | dist_front_status | uint8 | 0=valid |
| 7 | 1 | dist_left_status | uint8 | 0=valid |
| 8 | 1 | dist_right_status | uint8 | 0=valid |
| 9 | 3 | **padding** | — | Alignment (skipped) |
| 12 | 4 | dist_timestamp_ms | uint32 LE | SysTick ms |
| 16 | 4 | quat_w | float32 LE | MPU quaternion |
| 20 | 4 | quat_x | float32 LE | |
| 24 | 4 | quat_y | float32 LE | |
| 28 | 4 | quat_z | float32 LE | |
| 32 | 4 | accel_x | float32 LE | Acceleration g |
| 36 | 4 | accel_y | float32 LE | |
| 40 | 4 | accel_z | float32 LE | |
| 44 | 4 | mpu_timestamp_ms | uint32 LE | |
| 48 | 16 | speed_d100[4] | int32 LE ×4 | Four-wheel speed, 0.01 cm/s, positive = forward |
| 64 | 16 | odometer[4] | int32 LE ×4 | Four-wheel odometer counts |
| 80 | 4 | ctrl_timestamp_ms | uint32 LE | |
| 84 | 8 | current_mA[4] | int16 LE ×4 | Four-wheel current mA (signed) |
| 92 | 8 | bus_voltage_mV[4] | uint16 LE ×4 | Four-channel bus voltage mV |
| 100 | 4 | ina_timestamp_ms | uint32 LE | |
| 104 | 4 | sys_tick_ms | uint32 LE | Whole-frame snapshot timestamp |
| 108 | 1 | last_cmd_id | uint8 | Last command ID (echo) |
| 109 | 1 | last_cmd_status | uint8 | 0=OK, non-zero = error code |
| 110 | 1 | control_mode | uint8 | Control mode: 0=closed-loop, 1=open-loop |
| 111 | 1 | fw_version | uint8 | F407 firmware version (incrementing integer, 28=v2.28) |
| 112 | 1 | lock | uint8 | Control lock: 0=manual, 1=autonomous |
| 113 | 3 | **padding** | — | Tail alignment (116 B) |

**Total: 116 bytes.**

### 1.3 Direction 2 (ESP32-S3 → F407): PAYLOAD = command frame (cmd_id + param, one per frame)

**Packing rules (the ESP32-S3 sender must obey):**
1. **Exactly one command per frame**: `cmd_id(1B) + param(variable length)`; the param length is defined by cmd_id (see the command set table).
2. The CRC is computed after the whole frame payload (covering SOF~PAYLOAD), least-significant byte first.
3. After sending a frame, **pause for ≥ 1 byte time** to trigger the F407 USART2 IDLE interrupt for frame assembly.

**Command set (cmd_id + param):**

| cmd_id | Name | Param layout | Description |
|---:|---|---|---|
| 0x01 | LED_TOGGLE | None | |
| 0x02 | ~~SENSOR_RESET~~ | — | Deprecated (removed 2026-11-18; sensor reset is covered by SYS_RESET 0x08) |
| 0x03 | MOTOR | motor(1B) + speed(2B, int16 LE) | Open-loop direct drive |
| 0x04 | PID | kp(4B, float32 LE) + ki(4B, float32 LE) | Shared gains |
| 0x05 | MOTOR_CLOSED | motor(1B) + speed_d100(4B, int32 LE) | Closed-loop target speed |
| 0x06 | STOP_ALL | None | Full-stop e-brake (disconnect protection / safety brake) |
| 0x07 | ODOMETER_RESET | None | Zero the four-wheel odometers |
| 0x08 | SYS_RESET | None | F407 soft reset (S3 does not reboot) |
| 0x09 | LED_ON | None | Onboard LED on (absolute state) |
| 0x0A | LED_OFF | None | Onboard LED off (absolute state) |

**Command result codes (echoed by F407 in SensorData_t.last_cmd_status):**

| Code | Meaning |
|---:|---|
| 0x00 | Success |
| 0x01 | Unknown command ID |
| 0x02 | Parameter length error |
| 0x03 | Motor index out of range |

---

### 1.4 Gamepad Link (ESP32-C3 → F407, 100 Hz state stream): PAYLOAD = type + gamepad body

The Xbox gamepad goes through the ESP32-C3 BLE bridge; C3 sends raw gamepad state frames (**Plan A**: raw stick values are kept all the way to F407,
mapping / dead zone / ramping are centralized on F407, so the feel is tunable without re-flashing C3). Physically isolated from the WiFi/autonomous link.

```
┌────────────┬─────┬──────────────────────────────┬─────────┐
│ SOF        │ LEN │ PAYLOAD (type + body)         │ CRC16   │
│ 0xAA 0x55  │ 1B  │ 27B (type 1B + body 26B)      │ 2B (LSB 1st)    │
└────────────┴─────┴──────────────────────────────┴─────────┘
```

- **LEN** = 27 (type 1 + body 26); **type** = `0x10` (GAMEPAD_STATE)
- **body** = `c3_gamepad_payload_t` (26 B, little-endian, byte-by-byte offset table):

| Offset | Size | Field | Type | Description |
|---:|---:|---|---|---|
| 0 | 4 | seq | uint32 LE | Notification sequence number |
| 4 | 4 | disc_gen | uint32 LE | Disconnect generation |
| 8 | 1 | connected | uint8 | Whether the gamepad is connected |
| 9 | 1 | battery | uint8 | Battery level |
| 10 | 1 | input_fresh | uint8 | HID notification within 5 s |
| 11 | 2 | joy_lx | uint16 LE | Left stick horizontal |
| 13 | 2 | joy_ly | uint16 LE | Left stick vertical (forward/backward throttle) |
| 15 | 2 | joy_rx | uint16 LE | Right stick horizontal (single-side differential steering) |
| 17 | 2 | joy_ry | uint16 LE | Right stick vertical |
| 19 | 2 | trigger_lt | uint16 LE | Left trigger |
| 21 | 2 | trigger_rt | uint16 LE | Right trigger |
| 23 | 2 | buttons | uint16 LE | Button bitmap (table below) |
| 25 | 1 | lock | uint8 | Control lock: 0=manual, 1=autonomous |

**Total: 26 bytes; full frame length 32 bytes.**

**Stick values**: uint16 0–65535, center 32768, dead zone ±500, full deflection maps to speed ±4424 (0.01 cm/s).
**Lock state**: a C3 long-press of SELECT (1.5 s) toggles it; lock=0 manual (gamepad control), lock=1 autonomous (host PC control).
**At the instant the lock toggles**, F407 runs `CONTROL_StopAll()` to stop; the e-stop key (Xbox) bypasses the lock from both sources.

**Button bitmap (buttons):**

| bit | Key | | bit | Key |
|---|---|---|---|---|
| 0 | A | | 8 | Share |
| 1 | B | | 9 | LS |
| 2 | X | | 10 | RS |
| 3 | Y | | 11 | DPAD Up |
| 4 | LB | | 12 | DPAD Down |
| 5 | RB | | 13 | DPAD Left |
| 6 | Start | | 14 | DPAD Right |
| 7 | Select (lock key) | | 15 | Xbox (e-stop key) |

**Loss-of-link protection**: with the lock off (manual), if C3 sends no frame for over 500 ms / the gamepad disconnects / input goes stale → F407 stops;
with the lock on (autonomous), a gamepad disconnect does not affect autonomous operation.

---

## 2. TCP JSON (Port 8888)

One JSON object per line, terminated by a newline. S3 accepts only one client; video, telemetry, and commands share that control session. Maximum command line 16384 B (newline excluded); S3 total byte budget for complete lines in queue: 65536 B. The Python receive side also caps a single line at 16384 B.

### 2.1 Commands, ACK, gen, and Telemetry

| cmd | Parameters |
| --- | --- |
| `LED_TOGGLE` / `LED_ON` / `LED_OFF` / `STOP_ALL` / `ODOMETER_RESET` / `SYS_RESET` | No parameters; LED_ON/LED_OFF set the absolute state, LED_TOGGLE toggles; SYS_RESET only requests an F407 reset |
| `MOTOR` | `motor` integer 0–3, `speed` an integer in int16 range |
| `MOTOR_CLOSED` | `motor` integer 0–3, `speed_d100` an integer in int32 range (0.01 cm/s) |
| `PID` | `kp`, `ki` finite and float32-representable |
| `CAM_RES` | `res`: QQVGA / QVGA / VGA / SVGA |
| `VIDEO_ON` / `VIDEO_OFF` | S3 local video switch |
| `WS2812` | `r/g/b` required integers 0–255; `brightness` optional, default 100, and must be an integer 0–100 when given |

> **WS2812 status-light semantics (2026-11-16)**: not connected to the laptop hotspot = blue breathing (1.6 s period);
> connected = green fast-flash 3 times → solid green; disconnected = back to breathing. A manual `WS2812` command, while connected,
> overrides the solid color; after a connection-state change (disconnect/reconnect) it automatically returns to the status-light semantics.

```json
{"cmd":"MOTOR_CLOSED","motor":1,"speed_d100":4000,"gen":3}
{"cmd":"WS2812","r":128,"g":0,"b":0,"brightness":50,"gen":3}
{"cmd":"STOP_ALL"}
{"type":"heartbeat"}
```

Normal ACK: `{"ack":"command-name","status":0}`. 0 means accepted by the bridge layer, 1 means parameter error / unknown command / currently unacceptable, 2 means the explicit gen is stale. **ACK is neither an F407 execution confirmation nor a camera/LED hardware completion confirmation.** Unrecognized objects, invalid JSON, and some legacy messages without gen may be silently dropped.

Every time S3 latches a STOP it increments the uint32 `gen`, which is then sent in telemetry. When gen is provided it must equal the current uint32 integer; without gen the firmware compatibility path is kept. The current Python client — except for STOP / one-way safety heartbeats — must have obtained a usable telemetry generation before normal commands may be enqueued, and it attaches gen automatically. STOP clears commands not yet sent locally; new work is allowed only after telemetry with a different gen arrives. Callers may still provide gen explicitly; stale values are rejected by S3.

STOP / one-way safety heartbeats are exempt from gen. SDK four-wheel targets are queued as one local send batch (at most four JSON lines); F407 still executes them one at a time. A batch already committed before STOP may go out first, so an instant physical stop cannot be guaranteed. A normal action returning only means local enqueue succeeded; `peo.stop()` separately waits for a newly received successful STOP ACK, resends once after a 300 ms timeout, and raises if still unacknowledged. The protocol has no request ID, so strict per-request ACK correlation is impossible.

S3 reads snapshots and sends JSON on a fixed 50 ms beat, nominal **20 Hz**. The UART sensor source update rate differs from the TCP send rate.
**Lock-driven telemetry slowdown (2026-11-18)**: with the lock off (manual, lock=0) telemetry drops to **2 Hz** (one frame per 500 ms),
with the lock on (autonomous, lock=1) it returns to 20 Hz; the video stream is unaffected throughout.

```json
{"type":"sensor","tick":12345,"gen":3,
 "dist":{"front":{"mm":123,"ok":true},"left":{"mm":45,"ok":true},"right":{"mm":67,"ok":false}},
 "imu":{"quat":[1,0,0,0],"accel_g":[0,0,1]},
 "wheels":{"speed_d100":[0,0,0,0],"odometer":[0,0,0,0]},
 "power":{"current_mA":[100,110,-95,105],"bus_voltage_mV":[11800,11800,11800,11800]},
 "cmd_ack":{"id":3,"status":0},
 "control_mode":0,
 "lock":0,
 "fw_version":28,
 "s3_fw":28,
 "sys":{"cpu0":12,"cpu1":34,"ram_free":123456,"ram_total":200000}}
```

When there is no UART snapshot, or the received snapshot age exceeds 500 ms, S3 adds `link_stale:true` and can still send its own resource data. The host stores this flag in `SensorData.link_stale` (without changing the 116 B UART layout); SDK `fresh` additionally requires being connected and having received telemetry within the last 250 ms. Missing distance fields are treated as invalid. A historical sensor `cmd_ack` cannot confirm a current STOP.

Telemetry uses **latest-value mailbox** semantics (since 2026-11-18): S3 keeps only the most recent pending telemetry frame, and new frames overwrite older ones; under link congestion telemetry may be dropped (ACKs are unaffected), and congestion no longer causes disconnects. The host judges the link via the 250 ms fresh timeout and `link_stale`, and does not assume every telemetry frame must arrive. Numeric precision: `quat` 4 decimal places, `accel_g` 3 decimal places, non-finite floats output as `null`.

`sys.cpu0/cpu1` are per-core S3 utilization; `ram_free/ram_total` are internal heap bytes. The status bars on the GUI two tabs show only CPU0, CPU1, RAM; PING/RTT and video frame statistics have been removed.

### 2.2 Removed Features

The C3 gamepad bridge, the S3 local macro engine, and the host macro panel have all been removed. `{"type":"macro",...}` is no longer accepted, and no macro step / variable / jump protocol is provided.

### 2.3 Connection, Resources, and Video State

- **S3 = WiFi STA (finalized 2026-11-16)**: connects to the laptop mobile hotspot (SSID/password filled into
  `PEO_NET_STA_SSID/PASS` before flashing); the host connects via `peo_rover.local` (mDNS) or the hotspot-assigned IP;
  the S3 soft AP is no longer used (the old `PEO_Rover`/`192.168.4.1` has been removed).
- S3 `PEO_NET_MAX_CLIENTS=1`: the TCP accept layer rejects a second connection (in STA mode only the laptop hotspot
  is an AP, so other devices naturally cannot get in).
- Monitor and SDK share a local loopback TCP 8892 control lease; this is not a cross-device lock.
- After a TCP disconnect, S3 requests STOP, turns video off, and cleans the queues; the camera flushes/deinits after exiting its active loop.
- After VIDEO_ON the camera re-initializes; the first 48 grabbed frames after a successful recovery or switch are used for warm-up. First-frame latency depends on initialization/acquisition; 0.5 s is not guaranteed.
- On VIDEO_OFF the camera exits the grab loop and de-initializes; it no longer keeps capturing the "skipped" stream described in the old documentation.
- Resolution requests are latest-wins; a failed switch is retried a limited number of times and a fallback is attempted. ACK only means the request was enqueued.

### 2.4 Python Programming and Heartbeat

Scripts run on the PC, reading properties, sending actions, and waiting through the SDK; S3/F407 do not execute user Python.

**The actual heartbeat request:**

```json
{"type":"heartbeat"}
```

Once every 500 ms; S3 only refreshes this client safety-watchdog activity time — it does not reply, does not measure RTT, and does not resume motion. This message carries no `cmd` and is exempt from gen validation.

S3 supervises activity after successfully submitted MOTOR/MOTOR_CLOSED/PID; those successful commands or one-way safety heartbeats renew the lease, and after over 2000 ms S3 latches STOP and disconnects. Other local commands do not renew the lease. A background heartbeat does not prove the algorithm itself is making progress; the loop must check sensor and image freshness itself. On link loss / software crash the hardware side still needs an independent watchdog. **F407 loss-of-link self-stop (deadman, implemented 2026-11-18)**: S3 sends an empty LEN=0 frame heartbeat to F407 every 200 ms; if F407 gets no valid frame (CRC passed) for over 500 ms it automatically runs `CONTROL_StopAll()` — so even if S3 loses power or hangs, F407 can stop on its own.

SDK `drive` no longer deduplicates against unacknowledged old targets, nor claims a fixed 5 ms throttle. Callers should control the rate explicitly with `delay`. A GUI Stop immediately requests a priority STOP, then the SDK checks cooperative cancellation; an uncooperative Python thread cannot be force-killed.

## 3. TCP Video (Port 8890)

Finalized 2026-11-16: the video stream is now entirely TCP (the old UDP 8889 has been retired). S3 listens on 8890; the host connects and receives one-way. The video channel and the command channel (8888) are two independent TCP connections that do not block each other. S3-side it is 1:1: a new video connection displaces the old one and invalidates in-flight frames.

Frame delimiting: `magic(2) + frame_id(2) + length(4) + JPEG body`, big-endian header, magic is `0xCAFE`, length is the JPEG body byte count (limit 512 KiB; if the receiver exceeds it, the stream is treated as misaligned). frame_id is 16-bit and may wrap; it resets when S3 cleans up after a full disconnect. There is no fragmentation concept — TCP guarantees ordering and reliability, and lost frames appear as whole-frame skips (S3 depth-1 queue sends only the newest frame).

Send pacing: S3 sends each frame with non-blocking sends, advancing part of a frame per select-loop round (an in-flight frame must finish sending, otherwise the peer delimiting breaks); per-round send cap = 3 calls + an 8 KiB byte budget (header included); an EAGAIN from send (send buffer full) triggers a 50 ms cooldown, during which no new frame is started. The camera runs on an application-layer 15 fps beat. The network task is event-driven (eventfd wakeup + writability monitoring, since 2026-11-18): telemetry/ACK/new-frame enqueue wakes it immediately, a writable send buffer resumes sending immediately, and when idle it wakes once every 100 ms for heartbeat/statistics checks.

Receive: the host reads one full frame per the delimiting (a 5 s read timeout marks the link stalled). The JPEG body passes the SOI/SOF/SOS/segment-length/EOI structure guard before reaching the decoder (the guard does not verify entropy-data integrity). The SDK returns only complete frames from the last 1 s. On video disconnect/misalignment the host auto-reconnects (2 s backoff); the command channel is unaffected while connected.

## 4. Python Binary Reference

```python
SENSOR_DATA = struct.Struct('<HHHBBBxxxIfffffffI8iI4h4HIIBBBBBxxx')  # 116 B
VIDEO_REC = struct.Struct('>HHI')                              # 8 B: magic+fid+len
```

Implementation lives in `rover_comm/protocol.py`. CRC16-CCITT-FALSE polynomial 0x1021, init 0xFFFF, non-reflected; test vector `123456789` yields 0x29B1; not KERMIT.

## 5. Current Parameters and Verification Boundaries

| Item | Value |
| --- | --- |
| TCP | 8888 (commands/telemetry) + 8890 (video) |
| Local control lock | 127.0.0.1:8892 |
| TCP telemetry | ~20 Hz |
| S3 video frame rate | Sensor native frame rate (no artificial beat), in practice limited by hardware/network |
| Default resolution | QVGA 320×240 |
| Supported resolutions | QQVGA 160×120, QVGA 320×240, VGA 640×480, SVGA 800×600 |
| JSON line limit | 16384 B |
| Normal local send queue | 64 batches, at most four JSON lines per batch |

This round of offline testing provides no end-to-end latency budget or hardware-execution guarantees. See [REVIEW_AND_REPAIR.md](REVIEW_AND_REPAIR.md) for the latest test and repair records.

Video connection lifecycle: Python `connect(host, ..., video_port=8890)` only configures the video port, and connects only after `VIDEO_ON` has been sent successfully; after `VIDEO_OFF` is sent the socket is closed and reconnection stops. `video_port=None` disables video and `VIDEO_ON` returns False. With video enabled the 5 s read timeout and 2 s reconnect backoff are kept, and reconnection uses the port configured for the same session. The JPEG structure guard is kept (the application-layer CRC has been removed: TCP provides its own integrity check); frame counts are no longer accumulated.
