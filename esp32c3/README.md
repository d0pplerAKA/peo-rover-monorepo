# PEO Rover — ESP32-C3 Gamepad BLE Bridge Firmware

> **One-line role**: The C3 is the "translator" between the Xbox gamepad and the main controller — it connects to the gamepad over BLE and packs the stick/button state into frames the controller understands, sent over a direct wired UART to the F407. It sends raw values only; all mapping and arbitration live elsewhere.

---

## 1. Project Overview

The C3 (ESP32-C3 SuperMini, ESP-IDF v6.1) does exactly one job: **move gamepad state from wireless to wired**.

- **BLE** (NimBLE): connects to the Xbox gamepad filtered by target MAC, auto scan/connect/reconnect, 5 s first-notification watchdog;
- **Frame packing** (esp_link-identical): sticks/triggers/button bitmap/control lock packed into a 32-byte frame, sent at a fixed **100 Hz** over UART straight to the F407's USART3;
- **Heartbeat**: frames are sent regardless of gamepad connection — the frame itself is the C3's liveness heartbeat (the F407 stops if it sees no frame for 500 ms);
- **Local control lock**: Share long-press 1.5 s flips it (with rumble confirmation); the lock value rides in the frame, arbitration happens on the F407;
- **Status LED**: onboard blue LED breathing / flashing / solid states indicate connection status.

## 2. Why This Project Exists

1. **The F407 has no BLE**: the Xbox gamepad speaks BLE only; the controller cannot hear it, so a bridge is required.
2. **BLE and WiFi cannot share one chip** (measured early in the project): running WiFi AP and BLE together on the S3 made the link highly unstable, so the gamepad bridge lives on its own C3, physically isolated from WiFi.
3. **Lock logic stays local**: the timing logic of the Share long-press (counting, debounce, stray-press protection) lives on the C3 — closest to the gamepad — while the F407 only consumes the result; 100 Hz UART jitter cannot corrupt the long-press timing.
4. **Rock-solid 100 Hz heartbeat**: `vTaskDelayUntil` guarantees exact frame spacing, which is what makes the F407's 500 ms link-loss detection reliable.

## 3. What It Does

| Feature | Implementation |
|---|---|
| BLE link management | MAC whitelist (`68:6c:e6:42:57:89`), onLoop auto scan/connect/reconnect, 5 s first-notification watchdog |
| State reading | snapshot every 10 ms: sticks×4 (0-65535, center 32768), triggers×2 (0-1023), button bitmap, battery |
| Frame packing | 32-byte frame (SOF 0xAA55 + LEN 27 + type 0x10 + 26-byte body + CRC16), little-endian |
| 100 Hz heartbeat | sent regardless of gamepad connection, doubles as the C3 liveness signal |
| Control lock | Share long-press 1.5 s flip (plain timing, deliberately no HID-freshness gating), rumble feedback on flip |
| Status LED | GPIO8 blue LED (active LOW): breathing = disconnected, flashing = connect edge, solid = connected |

## 4. Connections to the Other Three Nodes

```
        ┌──────────────────────────────────┐
        │     Upper Computer (PC, Python)   │
        └───────────────┬──────────────────┘
                        │ (via S3; not this project's path)
        ┌───────────────▼──────────────────┐
        │       ESP32-S3 (WiFi bridge)      │
        └───────────────┬──────────────────┘
                        │ USART2
        ┌───────────────▼──────────────────┐
        │   STM32F407 (main controller)     │
        └───────────────▲──────────────────┘
                        │ USART3 115200 (100 Hz gamepad frames)
        ┌───────────────┴──────────────────┐
        │  ESP32-C3 (gamepad BLE, this) ★  │
        └───────────────▲──────────────────┘
                        │ BLE
                    Xbox gamepad
```

- **↔ STM32F407**: the C3's UART1 (GPIO4=TX/GPIO5=RX) wires directly to the F407's USART3 (115200), sending 100 Hz frames; the F407 owns deadzone/mapping/arbitration/emergency-stop.
- **↔ Upper computer / ESP32-S3**: no direct link — the C3 never touches WiFi; it is physically isolated from the S3, each owning one radio.

## 5. Main Implementation

### 5.1 Task structure (3 tasks)

| Task | Prio | Stack | Duty |
|---|---|---|---|
| xbox_send | 6 | 4096 | 100 Hz heartbeat: read snapshot → fill frame → maintain lock → UART send → notify LED (does not depend on BLE; keeps sending offline frames during connect/reconnect) |
| xbox_ble | 5 | 6144 | exclusive owner of the asukiaaa library: begin() + onLoop() + first-notification watchdog + rumble consumption, 20 ms tick |
| led_status | 1 | 2048 | drives the 3-state LED state machine on a 20 ms tick |

Task-split rules: lock state, fill_state, update_lock, and UART send are all owned by the sender task (no locking needed); rumble writes must happen in the BLE task (never touch the library's connection handle across tasks).

### 5.2 Frame format (esp_link-identical)

`SOF(0xAA 0x55) + LEN(1B=27) + type(0x10) + body(26B) + CRC16(2B)`, **32 B total**; CRC16-CCITT covers SOF..body, 30 bytes (standard vector 0x29B1). Body fields (`c3_gamepad_payload_t`, `static_assert(sizeof==26)`):

| Field | Type | Meaning |
|---|---|---|
| seq | u32 | valid-HID notification counter (survives reconnects; battery/short reports do not increment it) |
| disc_gen | u32 | disconnect generation (+1 on every link drop) |
| connected | u8 | connected and has data |
| battery | u8 | battery 0-100 (%) |
| input_fresh | u8 | C3-local "valid HID within 5 s" flag |
| joy_lx/ly/rx/ry | u16×4 | sticks 0-65535, center 32768 |
| trigger_lt/rt | u16×2 | triggers 0-1023 |
| buttons | u16 | bitmap (bit0=A, bit1=B, bit2=X, bit3=Y, bit4=LB, bit5=RB, bit6=START, bit7=SELECT, bit8=SHARE, bit9=LS, bit10=RS, bit11-14=d-pad, bit15=XBOX) |
| lock | u8 | control lock: 0=manual, 1=autonomous |

### 5.3 BLE details

- Target MAC filter: `PEO_XB_TARGET_MAC "68:6c:e6:42:57:89"` (empty string = auto-scan by appearance).
- `onLoop()` is called every 20 ms and runs the internal scan/connect/reconnect state machine; it **may block synchronously** (the heartbeat does not depend on it — frames keep flowing during the connect phase).
- First-notification watchdog: link up but no valid HID within 5 s → force disconnect to retrigger reconnect (judged by `isWaitingForFirstNotification()`, not `snapshot.connected`).
- Send cadence: `vTaskDelayUntil` absolute 10 ms period (`vTaskDelay` would absorb the 2.78 ms `uart_wait_tx_done` into the period, drifting to ~77 Hz — absolute ticks are mandatory).

### 5.4 Control lock (Share long-press)

- `XB_LOCK_BTN=XB_BTN_SHARE`, `XB_LOCK_HOLD_MS=1500U`.
- Plain long-press: press starts the timer → 1.5 s flips the lock → set triggered; release resets; a re-trigger requires observing a release first.
- **Deliberately no HID-freshness gating**: the Xbox gamepad only sends HID reports when the input *changes*; while held there are no new reports — freshness gating would zero the timer ~100 ms in and never reach 1.5 s (a real "long-press does nothing" bug that was reverted).
- On flip: rumble feedback (both motors 80% × 0.3 s, consumed and written by the BLE task) + log.
- **The B key (emergency stop) gets zero handling on the C3**: bit1 simply rides in the bitmap; the stop decision and arbitration live entirely on the F407.

## 6. What Each Feature Affects and Constrains

- **The 100 Hz frame is the basis of the F407's link-loss detection**: any C3 task hanging makes the F407 stop after 500 ms — so the xbox_send task does not depend on BLE (frames keep flowing while BLE blocks in connect), and its stack/cadence are conservatively designed.
- **Lock flips on the C3, arbitration on the F407**: the lock cannot flip while the C3 is offline (physically impossible); the F407 defaults to manual mode. The lock rides in each frame — if frames drop, the lock state follows the latest frame.
- **The C3 sends raw values only**: deadzone/curve/max-speed mapping all live on the F407 — tuning the feel never requires re-flashing the C3 (an explicit project decision).
- **GPIO constraints**: GPIO20/21 are taken by the onboard USB-UART and cannot be link pins; GPIO8 is a strapping pin, so the LED wiring must hold it high (LED off) at boot.
- **BLE begin() failure = dead loop requiring a reboot**: the sender keeps sending offline heartbeats (so the F407 does not false-trip), but a NimBLE stack init failure cannot self-heal.
- **Rumble must be written by the BLE task**: a sender-task HID write would race the library's internal connection handle — a hard constraint behind the task split.

## 7. Tech Stack & Build

- Chip: ESP32-C3 SuperMini (single-core 160 MHz, 4 MB flash); ESP-IDF v6.1 + NimBLE (esp-nimble-cpp ^2.6.0) + localized asukiaaa Xbox gamepad library.
- Build: `./build.ps1`; flash: `idf.py -p COMx flash monitor`.

## 8. License

Copyright (c) 2026 Danny — MIT License (see `LICENSE`).

Third-party components retain their own licenses — see
`THIRD_PARTY_NOTICES.md`: ESP-IDF (Apache-2.0), esp-nimble-cpp
(Apache-2.0), asukiaaa Xbox controller library (MIT).

LGPL note: this firmware statically links arduino-esp32 (LGPL-2.1).
Rebuilding the firmware from this public source satisfies the LGPL
re-linking requirement (see the build instructions above).
