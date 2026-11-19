# PEO Rover — ESP32-C3 手柄 BLE 桥 / Gamepad BLE Bridge

> **一句话定位 / One-line role**：C3 是 Xbox 手柄和主控之间唯一的「翻译官」——它通过 BLE 连上手柄，把摇杆/按键状态打包成主控能懂的帧，经串口直连发给 F407。
>
> The C3 is the sole "translator" between the Xbox gamepad and the main controller — it connects to the gamepad over BLE, packs the stick/button state into frames the main controller understands, and sends them over a direct UART link to the F407.

---

## 工程意义 / Why This Project Exists

**中文**：Xbox 手柄走的是 BLE（低功耗蓝牙），而 STM32F407 没有 BLE 能力。如果让 WiFi 桥（S3）同时处理 WiFi 和 BLE，两者抢同一颗芯片的射频资源会导致连接极不稳定（这是本工程立项前实测踩过的坑）。因此单独用一颗 ESP32-C3 专职做 BLE 桥，把手柄的模拟量（摇杆、扳机）和按键位图转换成数字帧，稳定地喂给主控。

**English**: The Xbox gamepad speaks BLE, but the STM32F407 has no BLE capability. If the WiFi bridge (S3) handled both WiFi and BLE at once, the two radios would fight over one chip's RF resources and the link became highly unstable (a real problem measured before this project was split out). So a dedicated ESP32-C3 acts as the BLE bridge, converting the gamepad's analog values (sticks, triggers) and button bitmap into digital frames fed reliably to the main controller.

---

## 功能与作用 / What It Does

| 功能 Function | 说明 Description |
|---|---|
| BLE 连接 | NimBLE 连 Xbox 手柄（指定目标 MAC 过滤） |
| 状态读取 | 摇杆/扳机模拟量 + 按键位图，高频采样 |
| 帧打包 | 转成 esp_link 同构帧（SOF `0xAA 0x55` + LEN + CRC16-CCITT） |
| 心跳 | 无论手柄连否都发帧，兼作 C3 存活信号 |
| 状态灯 | 连接/工作状态指示 |

---

## 与其它三端的联系 / Connections to the Other Three Nodes

```
        ┌──────────────────────────────────────┐
        │      上位机 Upper Computer (PC)       │
        └──────────────────┬───────────────────┘
                           │ (经 S3，与本工程无关)
        ┌──────────────────▼───────────────────┐
        │     ESP32-S3（WiFi 桥 Bridge）        │
        └──────────────────┬───────────────────┘
                           │ USART2
        ┌──────────────────▼───────────────────┐
        │  STM32F407（主控 Main Controller）    │
        └──────────────────▲───────────────────┘
                           │ USART3 (115200)   ← 本工程通过这条链路直连 F407
        ┌──────────────────┴───────────────────┐
        │   ESP32-C3（手柄 BLE 桥 Gamepad）  ★  │
        └──────────────────▲───────────────────┘
                           │ BLE
                     ┌─────┴─────┐
                     │ Xbox 手柄  │
                     └───────────┘
```

- **↔ STM32F407**：C3 的 UART 直连 F407 的 USART3，把手柄状态帧发过去；F407 端负责映射（死区/曲线/斜坡）和手操/自主锁仲裁。
- **↔ 上位机**：无直接联系——上位机走 WiFi 到 S3，C3 完全不碰 WiFi。
- **↔ ESP32-S3**：无直接电气连接——两者物理隔离，各管一条链路（S3 管 WiFi，C3 管 BLE）。

---

## 技术栈 / Tech Stack

- MCU：ESP32-C3 SuperMini（单核 160 MHz、4 MB flash）
- 工具链：ESP-IDF v6.1 + NimBLE（asukiaaa Xbox 手柄库本地封装）
- 语言：C++

## 构建 / Build

```powershell
Set-Location 'C:/Users/Danny/Documents/Espressif Files/peo_rover_esp32c3'
./build.ps1
```

---

Copyright (c) 2026 Danny — MIT License
