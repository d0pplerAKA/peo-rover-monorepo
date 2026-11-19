# PEO Rover — Monorepo

Four-node unmanned ground vehicle system for the PEO UGV Design Challenge.

## Structure

| Directory | Role | Stack |
|---|---|---|
| `f407/` | Main controller: motors + speed closed loop + sensors | C / Keil MDK |
| `esp32s3/` | WiFi bridge: commands / telemetry / video relay | C++ / ESP-IDF v6.1 |
| `esp32c3/` | Xbox gamepad BLE bridge | C++ / ESP-IDF v6.1 |
| `upper_computer/` | Upper computer: GUI + `peo` SDK for scripts | Python >=3.10 |

Each directory has its own `README.md` (English) and `README_zh.md` (Chinese) with full details.

## System Topology

```
Upper Computer (PC) ──WiFi── ESP32-S3 ──USART2 921600── STM32F407 ──USART3 115200── ESP32-C3 ──BLE── Xbox gamepad
TCP 8888 commands/telemetry · TCP 8890 video · 50 Hz sensor frames · 100 Hz gamepad frames
```

## Quick Start

The upper computer is the only part that needs no embedded toolchain:

```powershell
cd upper_computer
python -m pip install -r requirements.txt
python -m app.main
```

Building firmware requires ESP-IDF v6.1 (`esp32s3/`, `esp32c3/`) or Keil MDK (`f407/`) — see each directory's README.

## License

All four projects: Copyright (c) 2026 Danny, MIT License.
Each subdirectory contains its own `LICENSE` file.
