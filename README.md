# PEO Rover — STM32F407 主控固件 / Main Controller Firmware

> **一句话定位 / One-line role**：F407 是整台无人车的「大脑」，负责一切与物理世界直接相关的动作——驱动四个麦克纳姆轮、闭环控制轮速、采集传感器数据，并把所有状态上报给上层。
>
> The F407 is the "brain" of the rover — it drives the four mecanum wheels, closes the wheel-speed control loop, reads every sensor, and reports all state upward.

---

## 工程意义 / Why This Project Exists

**中文**：上层（上位机 / WiFi 桥 / 手柄桥）只负责「想往哪走、走多快」，真正把指令变成电机动作、把物理世界测回来的，是这块 STM32F407。它需要：
1. 以 1 kHz 的高频稳定地闭环控制四个轮子的转速，而不是「给个电压随它跑」；
2. 把分散的传感器（姿态、电流、测距、编码器）汇总成一致的遥测包；
3. 在手操（Xbox 手柄）和自主（上位机脚本）两种控制源之间做仲裁，保证同一时刻只有一方能驱车。

**English**: The upper layers (upper computer / WiFi bridge / gamepad bridge) only decide *where to go and how fast*. This STM32F407 is what actually turns those commands into motor motion and measures the physical world back. It must:
1. Close the speed loop on all four wheels at a steady 1 kHz — not just "apply a voltage and hope";
2. Aggregate scattered sensors (attitude, current, distance, encoders) into one consistent telemetry packet;
3. Arbitrate between the two control sources — manual (Xbox gamepad) and autonomous (upper-computer script) — so only one side can drive at a time.

---

## 功能与作用 / What It Does

| 功能 Function | 说明 Description |
|---|---|
| 四轮麦克纳姆控制 Mecanum drive | 前进/后退、原地旋转、左右平移（M1=左前 / M2=右前 / M3=左后 / M4=右后） |
| 速度闭环 PID | 增量式 PI 控制器 + 滑动窗口测速（TIM6 1 kHz 调度） |
| 传感器采集 | MPU6050（姿态/DMP）、INA3221（四路电流/电压）、VL53L1X（×3 测距）、正交编码器（×4） |
| 控制源仲裁 | 手操（手柄）与自主（上位机）的锁仲裁，急停键双源越过锁 |
| 双串口通讯 | USART2 ↔ ESP32-S3（921600）、USART3 ↔ ESP32-C3（115200） |
| 帧协议 | SOF `0xAA 0x55` + LEN + PAYLOAD + CRC16-CCITT |

---

## 与其它三端的联系 / Connections to the Other Three Nodes

```
                        ┌──────────────────────────────────────┐
                        │      上位机 Upper Computer (PC)       │
                        │   PySide6 GUI + peo SDK（组员脚本）   │
                        └──────────────────┬───────────────────┘
                                WiFi STA   │ TCP 8888 命令/遥测
                                            │ TCP 8890 视频
                        ┌──────────────────▼───────────────────┐
                        │     ESP32-S3（WiFi 桥 Bridge）        │
                        │  JSON↔esp_link 帧透传 + 视频 + 遥测    │
                        └──────────────────┬───────────────────┘
                                            │ USART2 (921600)   ← 本工程通过这条链路收命令、发遥测
                        ┌──────────────────▼───────────────────┐
                        │  STM32F407（主控 Main Controller）  ★ │
                        │  电机 + 速度闭环 + 传感器 + 锁仲裁      │
                        └──────────────────┬───────────────────┘
                                            │ USART3 (115200)   ← 本工程通过这条链路收手柄状态
                        ┌──────────────────▼───────────────────┐
                        │   ESP32-C3（手柄 BLE 桥 Gamepad）     │
                        │  NimBLE 连 Xbox → 摇杆/按键帧          │
                        └──────────────────────────────────────┘
```

- **↔ 上位机（经 S3）**：F407 收到上位机的运动/PID 命令并执行；同时把传感器快照（50 Hz）经 S3 转发给上位机。
- **↔ ESP32-S3**：USART2 是 F407 唯一的「命令入口 + 遥测出口」，S3 是纯透传路由器。
- **↔ ESP32-C3**：USART3 接收 Xbox 手柄的摇杆/按键状态帧（无论手柄连否 C3 都发心跳）。
- **安全兜底**：F407 自带 deadman（失联超时停车），不依赖 S3/C3 存活。

---

## 技术栈 / Tech Stack

- MCU：STM32F407VGT6（DevEBox 开发板）
- 工具链：Keil MDK + ST 标准外设库（StdPeriph，非 HAL/CubeMX），C 语言
- 调试：ST-Link（SWD）+ 外部 USB-TTL 串口 printf

## 构建 / Build

用 Keil MDK 打开 `peo_rover_f407.uvprojx` 编译，或用命令行：

```powershell
& 'C:/Users/Danny/AppData/Local/Keil_v5/UV4/UV4.exe' -b peo_rover_f407.uvprojx -j0 -o build_log.txt
```

引脚映射与定时器分配见 `PEO_ROVER_F407_PinMap.md`。

---

Copyright (c) 2026 Danny — MIT License
