# PEO Rover — Monorepo / 四端总仓库

Four-node unmanned ground vehicle for the PEO UGV Design Challenge.
PEO UGV 无人车设计挑战赛的四端系统总仓库。

## Structure / 目录结构

| Directory / 目录 | Role / 定位 | Stack / 技术栈 |
|---|---|---|
| `f407/` | Main controller: motors + speed closed loop + sensors / 主控执行器（电机+速度闭环+传感器） | C / Keil MDK |
| `esp32s3/` | WiFi bridge: commands / telemetry / video relay / WiFi 桥（命令/遥测/视频透传） | C++ / ESP-IDF v6.1 |
| `esp32c3/` | Xbox gamepad BLE bridge / Xbox 手柄 BLE 桥 | C++ / ESP-IDF v6.1 |
| `upper_computer/` | Upper computer: GUI + `peo` SDK / 上位机（GUI + peo SDK） | Python ≥3.10 |

Each directory has its own `README.md` (English) and `README_zh.md` (Chinese) with full details.
每个子目录内有各自的 `README.md`（英文）与 `README_zh.md`（中文）详述。

## System Topology / 系统拓扑

```
   Upper Computer (PC) ──WiFi── ESP32-S3 ──USART2── STM32F407 ──USART3── ESP32-C3 ──BLE── Xbox gamepad
   （上位机）             （WiFi 桥）           （主控）             （手柄桥）
   TCP 8888 commands/telemetry · TCP 8890 video · 921600 · 115200 · 100 Hz gamepad frames
```

## Quick Start / 快速开始

The upper computer is the only part that needs no embedded toolchain:
上位机是唯一无需嵌入式环境的部分：

```powershell
cd upper_computer
python -m pip install -r requirements.txt
python -m app.main
```

Building firmware requires ESP-IDF v6.1 (`esp32s3/`, `esp32c3/`) or Keil MDK (`f407/`) — see each README.
编译固件需 ESP-IDF v6.1（esp32s3/、esp32c3/）或 Keil MDK（f407/），见各子目录 README。

## License / 版权

All four projects: Copyright (c) 2026 Danny, MIT License.
Each subdirectory contains its own `LICENSE` file.
四端工程：Copyright (c) 2026 Danny，MIT License。各子目录内含各自 `LICENSE`。
