# PEO Rover — STM32F407 Main Controller Firmware

> **One-line role**: The F407 is the "execution core" of the four-node system — it drives the four mecanum wheels, closes the wheel-speed loop, reads every sensor, and arbitrates the manual vs. autonomous control sources on board the vehicle.

---

## 1. Project Overview

The F407 (STM32F407VGT6, Keil MDK + ST Standard Peripheral Library, C) handles everything that touches the physical world. It has two upstream links: USART2 to the ESP32-S3 (upper-computer commands relayed over WiFi) and USART3 directly to the ESP32-C3 (Xbox gamepad state frames). The upstream nodes only decide *how the rover should move*; this firmware turns those decisions into motor motion and measures the physical world back.

The firmware = `main.c` + 17 components under `COMPONENTS/`, all feature-trimmed by macro switches centralized in `sensor_hub.h`.

## 2. Why This Project Exists

1. **1 kHz speed closed loop**: the four wheels are not "apply a voltage and hope" — a TIM6 1 kHz loop closes the speed loop per wheel (incremental PI + sliding-window speed measurement), which mecanum driving demands because all four wheel speeds must match precisely.
2. **On-board arbitration**: the two control sources — manual (Xbox gamepad) and autonomous (upper-computer script) — are arbitrated on the F407 via a control lock. Only one side can drive at a time; the emergency-stop key always bypasses the lock from either source.
3. **Sensor aggregation**: MPU6050 (attitude), INA3221×2 (current), VL53L1X×3 (distance), and quadrature encoders×4 are all read by the F407 and packed into one 50 Hz telemetry frame.
4. **Independent safety net**: the F407 has its own deadman stop — it does not depend on the S3 or C3 staying alive; if either upstream link dies, the rover stops itself.

## 3. What It Does

| Feature | Implementation |
|---|---|
| Mecanum drive | forward/backward + rotate + strafe (M1=front-left / M2=front-right / M3=rear-left / M4=rear-right) |
| Speed closed loop | TIM6 1 kHz incremental PI × 4 wheels, sliding-window measurement (N=10, quantization step 1.03 cm/s) |
| Open-loop drive | MOTOR command drives PWM per-mille directly (±1000), for encoder-less bring-up |
| Gamepad mapping | sticks/triggers → four-wheel synthesis (deadzone 500, LB/RB side boost, B = emergency stop) |
| Control lock | flipped by C3's Share long-press; F407 arbitrates: manual mode rejects upper-computer motion commands, lock flip stops instantly |
| Sensor acquisition | MPU6050 (100 Hz) / INA3221×2 (10 Hz) / VL53L1X×3 (~13 Hz), off at boot, toggled by gamepad Y/X/A |
| Telemetry | TIM14 50 Hz whole-frame upload of the 116-byte `SensorData_t` |
| Onboard LED | boot-complete indicator + upper-computer LED commands (0x01/0x09/0x0A) |
| Debug UART | USART1 115200 with printf redirection |

## 4. Connections to the Other Three Nodes

```
                       ┌──────────────────────────────────┐
                       │    Upper Computer (PC, Python)    │
                       │  PySide6 GUI + peo SDK scripts    │
                       └───────────────┬──────────────────┘
                             WiFi STA  │ TCP 8888 cmd/telemetry
                                        │ TCP 8890 video
                       ┌───────────────▼──────────────────┐
                       │   ESP32-S3 (WiFi bridge, relay)   │
                       └───────────────┬──────────────────┘
                                        │ USART2 921600 (esp_link frames)
                       ┌───────────────▼──────────────────┐
                       │  STM32F407 (controller, this) ★  │
                       │  motors + loop + sensors + lock   │
                       └───────────────┬──────────────────┘
                                        │ USART3 115200 (gamepad frames 100 Hz)
                       ┌───────────────▼──────────────────┐
                       │   ESP32-C3 (Xbox BLE bridge)      │
                       └───────────────┬──────────────────┘
                                        │ BLE
                                  Xbox gamepad
```

- **↔ ESP32-S3**: USART2 is the F407's only command input and telemetry output (50 Hz up / commands down). The S3 relays blindly.
- **↔ ESP32-C3**: USART3 receives 100 Hz gamepad state frames (32 B, sent regardless of gamepad connection — doubles as the C3 heartbeat). Mapping and arbitration happen on the F407; the C3 sends raw values only.
- **↔ Upper computer**: no direct link — everything goes through the S3.
- **Safety net**: the F407 deadman-stops each link independently — S3 silent >500 ms → stop; C3 silent >500 ms → stop.

## 5. Main Implementation

### 5.1 Component structure (main.c + 17 components)

| Component | Responsibility |
|---|---|
| main.c | init orchestration + main loop (STOP first → command consumer → gamepad processing, 5 ms tick) |
| CONTROL | TIM6 1 kHz wheel loop: encoder deltas → odometry → sliding-window speed → incremental PI×4 → PWM |
| ESP_LINK | USART2 frame protocol: CRC16/DMA RX-TX/command queue/independent STOP flag/deadman/TIM14 reporting |
| GAMEPAD | USART3 gamepad frames, lock arbitration, mecanum mapping, sensor toggles |
| SENSOR_HUB | telemetry data hub: `SensorData_t` layout, sensor push APIs, interrupt-safe consistent snapshot |
| MOTOR | 8 PWM channels (20 kHz) driving 4×DRV8871 IN1/IN2 |
| ENCODER | 4 quadrature encoders (TIM1/3/4/8, TI12 ×4), 16-bit delta reads |
| MPU6050 | eMPL DMP attitude glue (WHO_AM_I check, self-test, EXTI3 FIFO callback) |
| INA3221 | 2×INA3221 (0x40/0x41) continuous conversion + TIM7 10 Hz polling |
| TOF400C | 3×VL53L1X (sequential XSHUT release + re-addressing) + EXTI result reads |
| I2C / NVIC / RCC / TIMER / DELAY / USART / BUILTIN_LED | bus, interrupt, clock, timestamp, debug console, LED infrastructure |
| PEO_PINMAP.H | single source of board facts (clock tree/pins/timers/encoder constants/I2C addresses, 551-line macro header) |

### 5.2 Frame protocol (esp_link)

`SOF(0xAA 0x55) + LEN(1B) + PAYLOAD + CRC16(2B)`, CRC16-CCITT (poly 0x1021 / init 0xFFFF), CRC low byte first. Direction 1 (F407→S3) = 50 Hz sensor frame (triggered by TIM14, PAYLOAD = 116-byte `SensorData_t` memcpy'd whole, 121 B total on the wire); direction 2 (S3→F407) = command frame (`cmd_id(1B) + param(≤8B)`, exactly one command per frame), LEN=0 empty frame = heartbeat (only refreshes the link timestamp).

Command set (0x02 deprecated):

| ID | Command | Behavior |
|---|---|---|
| 0x01 | LED_TOGGLE | toggle onboard LED |
| 0x03 | MOTOR | open-loop PWM per-mille; switches to open-loop mode when CONTROL exists |
| 0x04 | PID | kp/ki (float), validated finite & ≥0, shared by all four wheels |
| 0x05 | MOTOR_CLOSED | closed-loop target speed (0.01 cm/s, clamped ±4424), switches to closed-loop mode |
| 0x06 | STOP_ALL | **bypasses the queue** — independent flag handled first by the main loop |
| 0x07 | ODOMETER_RESET | zero all four odometers |
| 0x08 | SYS_RESET | spins 200000 cycles to flush UART, then soft reset |
| 0x09/0x0A | LED_ON/LED_OFF | absolute LED state |

### 5.3 Speed closed loop (CONTROL)

- TIM6: PSC=83 → 1 MHz, ARR=999 → **1 kHz** (highest NVIC preemption priority).
- Incremental PI: `u[n]=u[n-1]+Kp·(e[n]-e[n-1])+Ki·e[n]`, u clamped to ±1000 (output clamping = built-in anti-windup); defaults Kp=0.25 / Ki=0.005.
- Speed measurement: sliding window N=10 (`PEO_CTRL_SPEED_WINDOW_TICKS`), `speed = sum×1031/N`, quantization step 1.03 cm/s.
- Units: 0.01 cm/s, max speed 4424 (≈44.24 cm/s); encoders 11 PPR × 4 (quadrature) × 45:1 gearbox = 1980 CPR.

### 5.4 Gamepad mapping and lock arbitration (GAMEPAD)

- Frame: 100 Hz, 32 B from the C3 (26 B body: seq/disc_gen/connected/battery/input_fresh/sticks×4/triggers×2/button bitmap/lock).
- Mapping: `throttle=map_axis(joy_ly)`, `steer=map_axis(joy_rx)`, `strafe=GAMEPAD_STRAFE_SIGN×(map_trigger(rt)−map_trigger(lt))`; mecanum synthesis `m1=t+s, m2=t+steer−s, m3=t−s, m4=t+steer+s`; deadzone ±500; LB/RB side boost +2212 (half speed).
- Buttons: B = emergency stop (**always bypasses the lock from either source**), Y/X/A = VL53 / INA3221 / MPU6050 toggles (rising edge, manual mode only).
- Lock: flipped by C3's Share long-press, reported in the frame's `lock` field; the F407 arbitrates — manual mode rejects upper-computer MOTOR/PID/MOTOR_CLOSED (returns INVALID), lock flip stops instantly, autonomous mode skips gamepad mapping.

### 5.5 Sensors

| Sensor | Bus/Address | Rate | Start/stop |
|---|---|---|---|
| MPU6050 | I2C1 (PB6/PB7), 0x68 | 100 Hz (EXTI3 FIFO) | A-key toggle |
| INA3221×2 | I2C2 (PB10/PB11), 0x40+0x41 | 10 Hz (TIM7) | X-key toggle |
| VL53L1X×3 | I2C3 (PA8/PC9), 0x52/0x54/0x56 | ~13 Hz each (EXTI0/1/2) | Y-key toggle |

All sensors are **init-only at boot and off by default**; the gamepad keys turn them on. Data converges in SENSOR_HUB and is uploaded as one 50 Hz frame.

## 6. What Each Feature Affects and Constrains

- **Macro matrix** (`sensor_hub.h`): `MOTOR=1 / CONTROL=0 / ENCODER=CONTROL / TOF=1 / INA=1 / MPU=1`. The current default is the **open-loop configuration** — with `CONTROL=0`: encoders are not initialized (ENCODER auto-follows), `MOTOR_CLOSED` is rejected at compile time, and both the upper-computer MOTOR command and the gamepad fall back to `MOTOR_SetSpeed` direct drive (no speed feedback). Switching open/closed loop only requires the MOTOR+CONTROL macros; ENCODER follows automatically.
- **Closed-loop dependencies**: the speed loop needs MOTOR + CONTROL + ENCODER all enabled; the `MOTOR` command switches to open-loop mode and `MOTOR_CLOSED` switches back — modes are toggled live by commands.
- **Lock arbitration constraints**: in manual mode the upper computer cannot move the rover (only STOP/LED/reset); the lock flip forces a stop to avoid a speed jump across the handover.
- **Sensor toggles are manual-mode-only**: autonomous mode skips toggle logic; all off at boot.
- **The stop path coasts, it does not brake**: `CONTROL_StopAll()` → `MOTOR_Coast` (IN1=IN2=0); every emergency path (STOP_ALL / gamepad B / link loss) coasts. Same in the open-loop configuration.
- **Dual deadman links**: S3 silent (>500 ms without a valid frame, including the 200 ms empty-frame heartbeat) → stop; C3 silent (>500 ms without a frame, or connected=0/input_fresh=0) → stop — the F407 never depends on upstream being alive.
- **PID is shared across all four wheels**: one Kp/Ki pair controls all wheels; per-wheel gains are not supported.
- **DRV8871 has no nFAULT routed**: stall/over-current is inferred indirectly from INA3221 current monitoring.

## 7. Tech Stack & Build

- MCU: STM32F407VGT6 (DevEBox board); Keil MDK + ST Standard Peripheral Library (StdPeriph), C.
- Debug: ST-Link (SWD) + USART1 printf.
- Build: open `peo_rover_f407.uvprojx` in Keil MDK; CLI: `UV4.exe -b peo_rover_f407.uvprojx -j0`.

## 8. License

Copyright (c) 2026 Danny — MIT License (see `LICENSE`).
