# PEO Rover — STM32F407 主控固件

> **一句话定位**：F407 是四端系统的「执行中枢」——驱动四个麦克纳姆轮、闭环控制轮速、采集全部传感器，并把手操与自主两条控制链路的仲裁做在车上。

---

## 1. 工程总览

F407（STM32F407VGT6，Keil MDK + ST 标准库，C 语言）负责一切与物理世界直接相关的动作。它的上游有两路：USART2 连 ESP32-S3（上位机命令经 WiFi 桥透传），USART3 直连 ESP32-C3（Xbox 手柄状态帧）。上游只管「想怎么走」，真正把指令变成电机动作、把物理世界测回来的，是本工程。

固件结构 = `main.c` + 17 个组件（`COMPONENTS/`），全部通过宏开关（集中在 `sensor_hub.h`）裁剪功能。

## 2. 工程意义

1. **1 kHz 速度闭环**：四轮不是「给电压随它跑」，而是以 TIM6 1kHz 高频稳定闭环轮速（增量式 PI + 滑动窗口测速），满足麦克纳姆轮对四轮转速一致性的苛刻要求。
2. **双源仲裁在车上**：手操（Xbox 手柄）与自主（上位机脚本）两种控制源在 F407 上仲裁（控制锁），同一时刻只有一方能驱车，急停键双源永远越过锁。
3. **传感器汇聚**：MPU6050（姿态）、INA3221×2（电流）、VL53L1X×3（测距）、正交编码器×4 全部在 F407 采集，打包成统一的 50Hz 遥测帧上报。
4. **独立安全兜底**：F407 自带失联停车（deadman），不依赖 S3/C3 存活——S3 掉电或 C3 断线，车自己会停。

## 3. 功能与作用

| 功能 | 实现 |
|---|---|
| 四轮麦克纳姆控制 | 前进/后退 + 旋转 + 左右平移（M1=左前/M2=右前/M3=左后/M4=右后） |
| 速度闭环 | TIM6 1kHz 增量式 PI × 4 轮，滑动窗口测速（N=10，量化步长 1.03 cm/s） |
| 开环驱动 | MOTOR 命令直接 PWM 千分比（±1000），用于无编码器标定 |
| 手柄映射 | 摇杆/扳机 → 四轮速度合成（死区 500、LB/RB 单侧加速、B 急停） |
| 控制锁仲裁 | Share 长按在 C3 翻转，F407 执行仲裁：手操态拒上位机运动命令，锁切换瞬间停车 |
| 传感器采集 | MPU6050（100Hz）/ INA3221×2（10Hz）/ VL53L1X×3（~13Hz），上电默认关、手柄 Y/X/A 开启 |
| 遥测上报 | TIM14 50Hz 整帧上报 116B `SensorData_t` |
| 板载 LED | 启动完成指示 + 上位机 LED 命令（0x01/0x09/0x0A） |
| 调试串口 | USART1 115200 printf 重定向 |

## 4. 与其它三端的联系

```
                       ┌──────────────────────────────────┐
                       │       上位机（PC，Python）        │
                       │  PySide6 GUI + peo SDK 组员脚本  │
                       └───────────────┬──────────────────┘
                             WiFi STA  │ TCP 8888 命令/遥测
                                        │ TCP 8890 视频
                       ┌───────────────▼──────────────────┐
                       │    ESP32-S3（WiFi 桥，纯透传）    │
                       └───────────────┬──────────────────┘
                                        │ USART2 921600（esp_link 帧）
                       ┌───────────────▼──────────────────┐
                       │   STM32F407（主控，本工程） ★     │
                       │  电机 + 闭环 + 传感器 + 锁仲裁     │
                       └───────────────┬──────────────────┘
                                        │ USART3 115200（手柄帧 100Hz）
                       ┌───────────────▼──────────────────┐
                       │   ESP32-C3（Xbox 手柄 BLE 桥）    │
                       └───────────────┬──────────────────┘
                                        │ BLE
                                  Xbox 手柄
```

- **↔ ESP32-S3**：USART2 是 F407 唯一的命令入口 + 遥测出口（50Hz 上行 / 命令下行）。S3 纯透传，不处理逻辑。
- **↔ ESP32-C3**：USART3 接收 100Hz 手柄状态帧（32B，无论手柄连否 C3 都发，兼作存活心跳）。映射与仲裁在 F407，C3 只发原始值。
- **↔ 上位机**：无直接连接，一切经 S3 中转。
- **安全兜底**：F407 对两条链路分别 deadman——S3 失联 >500ms 停车、C3 失联 >500ms 停车。

## 5. 主要实现

### 5.1 组件结构（main.c + 17 组件）

| 组件 | 职责 |
|---|---|
| main.c | 初始化编排 + 主循环（STOP 优先 → 命令消费 → 手柄处理，5ms 节拍） |
| CONTROL | TIM6 1kHz 轮速闭环：编码器 delta → 里程 → 滑动窗口测速 → 增量式 PI×4 → PWM |
| ESP_LINK | USART2 帧协议：CRC16/DMA 收发/命令队列/STOP 独立标志/deadman/TIM14 上报 |
| GAMEPAD | USART3 手柄帧接收、锁仲裁、麦克纳姆映射、传感器 toggle |
| SENSOR_HUB | 遥测数据中心：`SensorData_t` 结构、各传感器 Push、关中断一致性快照 |
| MOTOR | 8 路 PWM（20kHz）驱动 4×DRV8871 IN1/IN2 |
| ENCODER | 4 路正交编码器（TIM1/3/4/8，TI12 四倍频），16 位差分读数 |
| MPU6050 | eMPL DMP 姿态胶水（WHO_AM_I 校验、自检、EXTI3 FIFO 回调） |
| INA3221 | 2 片 INA3221（0x40/0x41）连续转换 + TIM7 10Hz 轮询 |
| TOF400C | 3 片 VL53L1X（XSHUT 顺序释放重编址）+ EXTI 结果读取 |
| I2C / NVIC / RCC / TIMER / DELAY / USART / BUILTIN_LED | 总线、中断、时钟、时间戳、调试台、LED 基础设施 |
| PEO_PINMAP.H | 板级事实单点（时钟树/引脚/定时器/编码器常数/I2C 地址，551 行宏） |

### 5.2 帧协议（esp_link）

`SOF(0xAA 0x55) + LEN(1B) + PAYLOAD + CRC16(2B)`，CRC16-CCITT（poly 0x1021 / init 0xFFFF），CRC 低字节在前。方向1（F407→S3）= 50Hz 传感器帧（TIM14 触发，PAYLOAD=116B `SensorData_t` 整块 memcpy，帧总长 121B）；方向2（S3→F407）= 命令帧（`cmd_id(1B)+param(≤8B)`，每帧一条），LEN=0 空帧 = 心跳（只刷新链路时间戳）。

命令集（0x02 已废弃）：

| ID | 命令 | 行为 |
|---|---|---|
| 0x01 | LED_TOGGLE | 翻转板载 LED |
| 0x03 | MOTOR | 开环 PWM 千分比；有 CONTROL 时先切开环模式 |
| 0x04 | PID | kp/ki（float），校验 finite ≥0 后四轮共享 SetGains |
| 0x05 | MOTOR_CLOSED | 闭环目标速度（0.01cm/s，限幅 ±4424），切闭环模式 |
| 0x06 | STOP_ALL | **不进队列**，独立标志主循环优先处理 |
| 0x07 | ODOMETER_RESET | 四轮里程清零 |
| 0x08 | SYS_RESET | 空转 200000 拍发完 UART 后软复位 |
| 0x09/0x0A | LED_ON/LED_OFF | LED 绝对状态 |

### 5.3 速度闭环（CONTROL）

- TIM6：PSC=83 → 1MHz，ARR=999 → **1kHz**（NVIC 最高抢占优先级）。
- 增量式 PI：`u[n]=u[n-1]+Kp·(e[n]-e[n-1])+Ki·e[n]`，u 钳位 ±1000（对输出钳位天然抗积分饱和）；默认 Kp=0.25 / Ki=0.005。
- 测速：滑动窗口 N=10（`PEO_CTRL_SPEED_WINDOW_TICKS`），`speed = sum×1031/N`，量化步长 1.03 cm/s。
- 单位：0.01 cm/s，满速 4424（≈44.24 cm/s）；编码器 11 PPR × 4 倍频 × 减速 45:1 = CPR 1980。

### 5.4 手柄映射与锁仲裁（GAMEPAD）

- 帧：C3 100Hz 32B（body 26B：seq/disc_gen/connected/battery/input_fresh/摇杆×4/扳机×2/按键位图/lock）。
- 映射：`throttle=map_axis(joy_ly)`、`steer=map_axis(joy_rx)`、`strafe=GAMEPAD_STRAFE_SIGN×(map_trigger(rt)−map_trigger(lt))`；麦克纳姆合成 `m1=t+s、m2=t+steer−s、m3=t−s、m4=t+steer+s`；死区 ±500；LB/RB 单侧加速 +2212（半速）。
- 按键：B=急停（**双源永远越过锁**）、Y/X/A=VL53/INA3221/MPU6050 toggle（上升沿，仅手操态生效）。
- 锁：C3 Share 长按翻转，帧内 `lock` 字段上报；F407 执行仲裁——手操态拒上位机 MOTOR/PID/MOTOR_CLOSED（回 INVALID）、锁切换瞬间 StopAll、自主态跳过手柄映射。

### 5.5 传感器

| 传感器 | 总线/地址 | 节拍 | 启停 |
|---|---|---|---|
| MPU6050 | I2C1（PB6/PB7），0x68 | 100Hz（EXTI3 FIFO） | A 键 toggle |
| INA3221×2 | I2C2（PB10/PB11），0x40+0x41 | 10Hz（TIM7） | X 键 toggle |
| VL53L1X×3 | I2C3（PA8/PC9），0x52/0x54/0x56 | ~13Hz/片（EXTI0/1/2） | Y 键 toggle |

全部**上电 Init-only、默认关**，手柄按键开启；数据汇聚 SENSOR_HUB 后 50Hz 整帧上报。

## 6. 功能的影响与制约

- **宏开关矩阵**（`sensor_hub.h`）：`MOTOR=1 / CONTROL=0 / ENCODER=CONTROL / TOF=1 / INA=1 / MPU=1`。当前默认 = **开环配置**——`CONTROL=0` 时：编码器不初始化（ENCODER 自动跟随）、`MOTOR_CLOSED` 命令被编译期守卫拒绝、上位机 MOTOR 与手柄都走 `MOTOR_SetSpeed` 开环直驱（无速度反馈）。开闭环切换只需改 MOTOR+CONTROL 两个宏，ENCODER 自动跟随。
- **闭环依赖**：速度闭环 = MOTOR + CONTROL + ENCODER 三者同时开启；`MOTOR` 命令会切开环模式、`MOTOR_CLOSED` 切回闭环——两种模式由命令实时切换。
- **锁仲裁的制约**：手操态下上位机无法移动车（只能 STOP/LED/复位）；锁切换瞬间强制停车（防控制权交接时速度突变）。
- **传感器 toggle 只在手操态**：自主态跳过 toggle 逻辑，上电默认全关。
- **停车路径是滑行不是刹车**：`CONTROL_StopAll()` → `MOTOR_Coast`（IN1=IN2=0），所有急停路径（STOP_ALL/手柄 B/失联）都走滑行；开环配置下同样生效。
- **deadman 双链路**：S3 失联（>500ms 无有效帧，含 200ms 空帧心跳）停车；C3 失联（>500ms 无帧或 connected=0/input_fresh=0）停车——F407 不依赖任何上游存活。
- **PID 四轮共享**：一套 Kp/Ki 控制四轮，不支持单轮独立增益。
- **DRV8871 无 nFAULT 引出**：堵转/过流靠 INA3221 电流监测间接判断。

## 7. 技术栈与构建

- MCU：STM32F407VGT6（DevEBox 开发板）；Keil MDK + ST 标准库（StdPeriph），C 语言。
- 调试：ST-Link（SWD）+ USART1 串口 printf。
- 构建：Keil MDK 打开 `peo_rover_f407.uvprojx` 编译；命令行 `UV4.exe -b peo_rover_f407.uvprojx -j0`。

## 8. 版权

Copyright (c) 2026 Danny — MIT License（见 `LICENSE`）。

第三方组件保留各自许可——见 `THIRD_PARTY_NOTICES.md`：ST 标准外设库
（ST Liberty License）、ARM CMSIS（Apache-2.0/BSD）、VL53L1X Ultra Lite
Driver（ST SLA0103）、InvenSense Motion Driver（InvenSense 专有许可）。
