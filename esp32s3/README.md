# PEO Rover — ESP32-S3 WiFi Bridge Firmware

> **One-line role**: The S3 is the "router" between the upper computer and the main controller — it translates upper-computer JSON commands into frames the controller understands, translates the controller's sensor data into JSON, and relays camera video. It is deliberately "dumb": no business logic, only pass-through plus a few safety guardrails.

---

## 1. Project Overview

The S3 (ESP32-S3 N8R8, ESP-IDF v6.1 + arduino-esp32 as component) carries all wireless connectivity and data plumbing for the rover:

- **WiFi STA**: joins the upper computer's hotspot (SSID/password hard-coded in `esp_net.h` — fill in real values before deployment);
- **TCP 8888**: upper-computer JSON commands → esp_link command frames → F407 (USART2 921600); the reverse path parses the F407's 116-byte sensor frames into a snapshot that telemetry turns into JSON;
- **TCP 8890**: one-way OV2640 JPEG video relay to the upper computer;
- **Safety guardrails**: STOP lock + generation counter (old commands die after power cycle), heartbeat-timeout stop, PID stored in NVS (on-vehicle persistence).

Firmware = 7 components + 7 FreeRTOS tasks; components communicate only via queues.

## 2. Why This Project Exists

1. **The F407 has no WiFi**: the controller only has UARTs; the S3 is its only wireless channel to the upper computer.
2. **BLE and WiFi cannot share one chip** (measured early in the project): the gamepad BLE bridge was therefore split off to the C3; the S3 does WiFi only.
3. **Safety barrier**: the S3 owns "communication-layer safety" — upper-computer disconnect, heartbeat timeout, or video-link failure all latch STOP, and the generation counter invalidates all pre-reboot commands.
4. **On-vehicle PID persistence**: the PID lives in the S3's NVS (wear-leveled, million-cycle endurance), survives power loss, and is pushed back to the F407 automatically at boot / on client connect — the rover keeps its tuned PID even offline.

## 3. What It Does

| Feature | Implementation |
|---|---|
| Command relay | TCP 8888 JSON → cmd_parser validation → esp_link frames → F407 |
| Telemetry upload | F407 50 Hz sensor frames → telemetry JSON (20 Hz; 2 Hz in manual mode) → upper computer |
| Video relay | OV2640 JPEG at 24 fps → TCP 8890 (8-byte header + JPEG body) |
| STOP lock + generation | STOP latched at boot; generation +1 on every STOP; resume requires deliberate input carrying the new generation |
| Heartbeat supervision | upper computer `{"type":"heartbeat"}`, 2 s timeout stops the rover (only supervises clients that sent control commands) |
| Deadman heartbeat | S3→F407 empty-frame heartbeat every 200 ms (the F407 stops itself after >500 ms of silence) |
| PID persistence | NVS (namespace=`esp_link`, key=`pid`, 8-byte blob), auto-restored on first sensor frame / client connect |
| Status LED | WS2812 (GPIO48): blue breathing = no hotspot, solid green = connected, green flash ×3 = connect edge |
| Local commands | CAM_RES / VIDEO_ON/OFF / WS2812 color (not forwarded to the F407) |

## 4. Connections to the Other Three Nodes

```
        ┌──────────────────────────────────┐
        │     Upper Computer (PC, Python)   │
        └───────────────┬──────────────────┘
              WiFi STA  │ TCP 8888 cmd/telemetry
                         │ TCP 8890 video
        ┌───────────────▼──────────────────┐
        │   ESP32-S3 (WiFi bridge, this) ★ │
        └───────────────┬──────────────────┘
                         │ USART2 921600 (esp_link frames)
        ┌───────────────▼──────────────────┐
        │   STM32F407 (main controller)     │
        └───────────────┬──────────────────┘
                         │ USART3
        ┌───────────────▼──────────────────┐
        │   ESP32-C3 (gamepad BLE bridge)   │
        └──────────────────────────────────┘
```

- **↔ Upper computer**: the only exit. The S3 joins the PC hotspot as STA; 8888 carries commands/telemetry, 8890 carries video.
- **↔ STM32F407**: USART2 is the only channel — the S3 relays command frames, receives sensor frames, and sends the 200 ms empty-frame heartbeat that feeds the F407's deadman.
- **↔ ESP32-C3**: no direct electrical link (S3 owns WiFi, C3 owns BLE — physically isolated).

## 5. Main Implementation

### 5.1 Task structure (assembled by app_main, 7 tasks)

| Task | Prio | Stack | Core | Duty |
|---|---|---|---|---|
| esp_rx | 6 | 4096 | 0 | UART1 init + sensor frame RX (required) |
| esp_tx | 5 | 3072 | 0 | command queue → frames + empty-frame heartbeat (required) |
| esp_net | 5 | 10240 | 0 | WiFi STA + 8888/8890 select loop (required) |
| telemetry | 4 | 6144 | 1 | telemetry JSON assembly |
| cmd_parser | 4 | 6144 | 1 | JSON command parsing |
| camera | 3 | 8192 | 1 | OV2640 frame capture |
| ws2812 | 2 | 3072 | 1 | status LED |

Required-task creation failure → `esp_link_fatal_fault()` permanent dead loop (refuse to run half-alive). Queues: sensor snapshot 1-deep overwrite, command 8-deep, net rx 32-deep, ack 8-deep, camera request 1-deep overwrite.

### 5.2 Frame protocol (identical to the F407's)

`SOF(0xAA 0x55) + LEN(1B) + PAYLOAD + CRC16(2B)`, CRC16-CCITT (0x1021/0xFFFF). Direction 1 = F407 50 Hz sensor frame (PAYLOAD fixed 116 B, `static_assert(sizeof==116)`; the RX sliding window keeps the suffix on CRC failure to resync); direction 2 = command frame (one per frame, `cmd_id+param≤8B`, then a 2-tick pause to trigger the F407's USART2 IDLE framing).

### 5.3 Command table (cmd_parser)

Forwarded to F407 (JSON → esp_link frame):

| JSON | Notes |
|---|---|
| `{"cmd":"LED_TOGGLE"/"LED_ON"/"LED_OFF"}` | onboard LED |
| `{"cmd":"MOTOR","motor":0-3,"speed":i16}` | open-loop PWM per-mille |
| `{"cmd":"MOTOR_CLOSED","motor":0-3,"speed_d100":i32}` | closed-loop speed (0.01 cm/s) |
| `{"cmd":"PID","kp":f32,"ki":f32}` | PID gains; goes through send_pid_and_save (also writes NVS) |
| `{"cmd":"STOP_ALL"}` | unconditionally exempt from gen checks, goes straight to request_stop |
| `{"cmd":"ODOMETER_RESET"/"SYS_RESET"}` | zero odometry / F407 soft reset |

Local commands (not forwarded): `CAM_RES {"res":"QQVGA|QVGA|VGA|SVGA"}`, `VIDEO_ON/OFF`, `WS2812 {"r","g","b","brightness"}`. ACK format `{"ack":name,"status":N}` (0 ok / 1 bad param or busy / 2 stale generation). **Note: there is no PING command** — the heartbeat is the one-way `{"type":"heartbeat"}`.

### 5.4 STOP lock + generation (the safety core)

- `esp_link_request_stop()`: `++generation; inhibited=true; stop_pending=true`; the STOP frame is sent first (10 ms retry on failure).
- Resume: `esp_link_resume_commands(observed_gen)` requires `observed==generation && !stop_pending && !fatal`.
- Commit point: the TX loop checks `!inhibited && cmd.generation==generation` inside the safety lock; at most one already-committed old frame may precede STOP on the wire.
- Semantics = "safety epoch": every STOP bumps the generation, invalidating all old work and old client commands; only deliberate new input carrying the new generation can resume.
- STOP triggers: boot, heartbeat timeout, WiFi disconnect, client disconnect, video-link error, TX hard error, slow client / over-long line.

### 5.5 PID in NVS

namespace=`esp_link`, key=`pid`, 8-byte blob (kp f32 LE + ki f32 LE). Written after the PID command is successfully queued (`esp_link_send_pid_and_save`); `s_pid_mutex` serializes manual saves vs. restore reads (no stale-value overwrite). Auto-restore triggers = first RX sensor frame / TCP 8888 client connect → TX task resumes and queues, retrying every 50 ms on failure. Semantics: **the on-vehicle PID wins**.

### 5.6 Video (camera_service + esp_net)

OV2640 with its own 20 MHz XCLK crystal (`CAM_XCLK_HZ=20000000`, the ESP32 outputs no clock); resolutions QQVGA/QVGA/VGA/SVGA (initial QVGA); absolute 24 fps tick for all resolutions; JPEG quality QQVGA/QVGA=25, VGA=30, SVGA=35; buffers 16/32/64/96 KB; fb_count=3 + GRAB_LATEST. Sending: 8-byte big-endian header `0xCAFE+fid+len` + JPEG body, EAGAIN → 50 ms cooldown (an in-flight frame must finish), hard send error → STOP + close connection. Idle (no client / video OFF) fully deinits (returns PSRAM); hard-resets the sensor before resume.

## 6. What Each Feature Affects and Constrains

- **STOP latched at boot**: old motor targets die on power cycle; motion requires input carrying the new generation — safety over convenience.
- **Heartbeat supervises only "commanding" clients** (has_commanded): a connection that only sent harmless messages does not trigger the stop on timeout; a client that sent MOTOR/MOTOR_CLOSED/PID and goes silent for 2 s gets STOP latched and kicked.
- **Single client enforced**: `PEO_NET_MAX_CLIENTS=1`, the second TCP connection is refused — the upper computer pairs this with ControlLock (port 8892).
- **Video and command are two independent TCP links**: video loss never affects the command channel; but a video hard send error triggers STOP (conservative safety).
- **Telemetry rate follows the lock**: manual mode (lock=0) drops to 2 Hz, autonomous 20 Hz — manual mode leaves bandwidth to video.
- **PID authority is on the vehicle**: after the upper computer's set_pid, the S3 writes NVS immediately; both boot and client connect overwrite the F407 with the on-vehicle value.
- **Hotspot config is a hard-coded macro** (`PEO_NET_STA_SSID/PASS`): changing the hotspot requires a rebuild; there is no runtime configuration entry.
- **A dead S3 cannot send STOP**: that is why the F407 has its own deadman (fed by the S3's 200 ms empty-frame heartbeat); a physical power cutoff is the last line of defense.

## 7. Tech Stack & Build

- Chip: ESP32-S3 N8R8 (8 MB Quad Flash + 8 MB Octal PSRAM); ESP-IDF v6.1 + arduino-esp32 as component.
- Build: `./build.ps1` (activates the EIM v6.1 environment, runs `idf.py build`).
- Flash: `idf.py -p COMx flash monitor`.

## 8. License

Copyright (c) 2026 Danny — MIT License (see `LICENSE`).

Third-party components retain their own licenses — see
`THIRD_PARTY_NOTICES.md`: ESP-IDF (Apache-2.0), esp32-camera (Apache-2.0),
cJSON (MIT), managed components under their own licenses.

LGPL note: this firmware statically links arduino-esp32 (LGPL-2.1).
Rebuilding the firmware from this public source satisfies the LGPL
re-linking requirement (see the build instructions above).
