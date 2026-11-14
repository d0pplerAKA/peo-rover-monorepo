# PEO Rover — STM32F407VGT6 引脚分配总表（Pin Map）

> 版本 v2.20 · 2026-11-14（boot banner 迁入 SENSOR_HUB + 版本追平）
> 芯片：STM32F407VGT6（LQFP100）· 开发板：**DevEBox STM32F4XX-M**（板载 8MHz HSE → 168MHz）
> 依据：RM0090 Rev 21 + stm32-base.org 板卡资料（J4 TFT 座 = 8×1 排母：1=3V3 2=GND 3=SDI/PB15 4=SCL/PB13 5=CS/PB12 6=SDO/PB14 7=D/C/PC5 8=BLK/PB1，**无 RST 脚**）
> v1.3 变更：M2 PWM 从 TIM3_CH3/CH4(PB0/PB1) 挪至 TIM9_CH1/CH2(PA2/PA3)，PB1 归还 TFT BLK
> v1.4 变更：减速比 45:1 定案；ESP32 通讯定案 UART(USART2)；TFT 兼容性定案 ST7789/ST7735（RST 飞线 PE12）
> v1.5 变更：电流监测双路径预留——A: ACS712@PA4；B: INA3221@I2C2
> v1.7 变更：M4 电机从 TIM4 PB6-9 平移到 TIM4 PD12-15 → 腾出 I2C1(PB6/PB7) 专供 3×VL53L1X
> v1.8 变更：编码器与 PWM 全分离——4 编码器全部纯编码器（ARR=0xFFFF 免中断）；8 路 PWM 拆到 TIM2/5/9/12；DRV8871 为简化模组（无 nSLEEP/nFAULT）→ 删 nSLEEP×4、nFAULT×4、I2C2
> **v1.9 变更：① 恢复 TFT（J4 座 6 信号脚全部归还）——M4 PWM 从 TIM12(PB14/15) 挪到 TIM10_CH1(PB8)+TIM11_CH1(PB9)，把 PB14/15 还给 TFT 的 SPI2；② 移除 XSHUT×3(PC0/1/2)（VL53 寻址方案待定）**
> **v2.0 变更：① 恢复 XSHUT×3（PC0=前 / PC1=左 / PC2=右），XSHUT 分时上电改址到 0x52/54/56（VL53 寻址定案）；② 注释清理——修正 rcc.h 编码器 M 编号、PEO_PINMAP.H 过时 SD 卡 / TIM2/5/9/12 注释；③ 资源统计更新（I2C3 已释放可用、GPIO 30→33）**
> **v2.1 变更：I2C3（PA8=SCL / PC9=SDA，AF4）正式列为 MPU6050 + INA3221 预留总线。I2C2 确认不恢复——M1 PWM 占 TIM2 PB10/11，编码器定时器（SMS 从模式独占计数器）无法再挂恒定频率 PWM（RM0090 §18.3.12）。**
> **v2.2 变更：INA3221 从 I2C3 挪到 I2C1（与 3×VL53L1X 共享，0x40-43 vs 0x52/54/56 零冲突）；I2C3 改为 MPU6050 独占——IMU 采样率高需干净总线。**
> **v2.3 变更：交换两条 I2C 总线——I2C1(PB6/7) 改为 MPU6050 独占；I2C3(PA8/PC9) 承载 3×VL53L1X + INA3221。地址不变（0x68 vs 0x52/54/56 + 0x40-43）。**
> **v2.4 变更：测距芯片 VL53L0X → VL53L1X。XSHUT×3 保留——VL53L1X 多片同总线同样需 XSHUT 分时上电改址（默认 0x52，写寄存器 0x0001 改址，对应 L0X 的 0x8A）。地址方案不变（0x52/54/56）。**
> **v2.5 变更：VL53L1X 补上 INT×3（GPIO1 测距就绪中断）——前=PC3(EXTI3)、左=PC4(EXTI4)、右=PC8(EXTI9_5)；XSHUT×3 不变。功能脚 35→38。**
> **v2.6 变更：INT 引脚迁到 GPIOE——VL53L1X 前/左/右 = PE0/PE1/PE2（EXTI0/1/2）；MPU6050 INT 新增 = PE3（EXTI3）。4 个 INT 全走独立 EXTI 线（不再有共享 EXTI9_5 向量）。PC3/PC4/PC8 释放回空闲。**
> **v2.7 变更：移除 ESP32 通讯（USART2 PD5/PD6 + DMA1 Stream5/6，暂不接）；NVIC 改 Group2；DELAY/I2C/TOF400C/VL53L1X/TIMER 组件落地。**
> **v2.8–v2.15 变更：组件落地（MOTOR/ENCODER/CONTROL/SENSOR_HUB/eMPL DMP），无引脚/外设映射变化。**
> **v2.16 变更：BUILTIN_LED 新增 PA1（板载 LED D2）。**
> **v2.17 变更：I2C2 重新启用（备用脚 PF1=SCL / PF0=SDA，绕开被 M1 PWM 占用的 PB10/11），挂 2 片 INA3221（0x40/0x41）电流监测。I2C 最终分配：I2C1=MPU6050、I2C2=INA3221×2、I2C3=VL53L1X×3。**
> **v2.18 变更：ESP32 通讯重新接入（USART2 PD5/PD6 + TIM14 上报定时器）。**
> **v2.19 变更：main.c 模块启用开关（PEO_ENABLE_*）+ boot banner。**
> **v2.20 变更：boot banner/dbg_f3 迁入 SENSOR_HUB；PEO_ENABLE_* 宏迁到 sensor_hub.h；main.c 只做接线。**

---

## 0. 板载外设占用清单（本表所有分配已避让）

| 板载设备 | 占用引脚 | 处理 |
|---|---|---|
| K1 用户按键 | PA0（active high） | **保留不碰** |
| LED D2 | PA1（sink） | **保留不碰** |
| W25Q16BV SPI FLASH | PA15(/CS) PB3(CLK) PB4(DO) PB5(DI) | **保留不碰** |
| **TFT 屏接口 J4** | PB12(CS) PB13(SCL) PB14(SDO) PB15(SDI) PC5(D/C) PB1(BLK) | **全部归还（v1.9）**，驱动走 SPI2 或模拟 SPI |
| SD 卡槽 | PC8(DAT0) PC9(DAT1) PC10(DAT2) PC11(DAT3) PC12(CLK) PD2(CMD) | **SD 功能弃用**，这些脚现空闲 |
| USB | PA11(D-) PA12(D+) | 不用 USB，空闲备用 |
| SWD | PA13(SWDIO) PA14(SWCLK) | **保留（ST-Link）** |

**未引出引脚**：PC14、PC15、PB2、PH0/PH1（晶振）——不可分配。

---

## 1. 需求汇总（已确认硬件）

| 功能 | 数量 | 接口 | 硬件型号 |
|---|---|---|---|
| ACS712 总电流 | 1 | 模拟 → ADC | ACS712-05B（185mV/A） |
| VL53L1X 障碍物测距（前/左/右） | 3 | I2C3 | VL53L1X |
| 电机编码器 | 4 | 定时器正交（A/B） | 霍尔编码器 PPR 11 |
| 电机驱动 | 4 | DRV8871（每电机 1 片） | **简化模组：IN1/IN2 + OUT1/OUT2（无 nSLEEP/nFAULT）** |
| 电机 PWM（每电机 2 路） | 8 | 定时器输出 | — |
| 调试串口 | 1 | USART1（外部 USB-TTL） | 115200 |
| TFT 屏 | 1 | SPI2 引脚组 | 板载 J4 排座 |
| MPU6050 整车震动（预留） | 1 | I2C1（独占） | MPU6050（0x68） |
| INA3221 多路电流（预留，路径 B） | 1 | I2C3（与 VL53 共享） | INA3221（0x40-43，与 ACS712 二选一待定） |

> MPU6050 独占 I2C1（IMU 采样率高、需干净带宽）；INA3221 与 3×VL53L1X 共享 I2C3（0x40-43 vs 0x52/54/56 零冲突）。I2C2 因 M1 PWM 占用不恢复。

---

## 2. 外设 → 引脚分配总表（主表）

### 2.1 核心设计：编码器与 PWM 全分离（v1.8 → v1.9）

**4 纯编码器 + 5 PWM 定时器（M4 拆两个单路）**：

| 电机 | 编码器定时器（CH1/CH2，纯编码器 ARR=0xFFFF） | PWM 定时器（各 2 路，独立 ARR） |
|---|---|---|
| M1 左前 | TIM1（PE9 / PE11） | TIM2_CH3/CH4（PB10 / PB11） |
| M2 左后 | TIM3（PA6 / PA7） | TIM5_CH3/CH4（PA2 / PA3） |
| M3 右前 | TIM8（PC6 / PC7） | TIM9_CH1/CH2（PE5 / PE6） |
| M4 右后 | TIM4（PD12 / PD13） | **TIM10_CH1（PB8）+ TIM11_CH1（PB9）** |

- **4 编码器全部纯编码器**：ARR=0xFFFF，16 位差分，**零中断**。
- **8 路 PWM 拆到 5 个定时器**（v1.9）：M4 用 TIM10+11 两个单路定时器，因为 TIM12 的 PB14/15 让给了 TFT。其余 M1/M2/M3 各 1 个定时器 2 路相邻。
- TIM2 跳过死 CH2（PB3=FLASH）、TIM5 跳过被按键/LED 占的 CH1/2 —— 板载全保留。

### 2.2 调试串口 —— USART1（PA9/PA10）

| 引脚 | 功能 | AF |
|---|---|---|
| PA9 | USART1_TX → 外部 USB-TTL | AF7 |
| PA10 | USART1_RX ← 外部 USB-TTL | AF7 |

### 2.3 PWM → DRV8871 接线

| PWM | 电机 | 定时器/通道 | 引脚 | AF | 接 DRV8871 |
|---|---|---|---|---|---|
| PWM1 | M1 | TIM2_CH3 | PB10 | AF1 | M1-IN1 |
| PWM2 | M1 | TIM2_CH4 | PB11 | AF1 | M1-IN2 |
| PWM3 | M2 | TIM5_CH3 | PA2 | AF2 | M2-IN1 |
| PWM4 | M2 | TIM5_CH4 | PA3 | AF2 | M2-IN2 |
| PWM5 | M3 | TIM9_CH1 | PE5 | AF3 | M3-IN1 |
| PWM6 | M3 | TIM9_CH2 | PE6 | AF3 | M3-IN2 |
| PWM7 | M4 | TIM10_CH1 | PB8 | AF3 | M4-IN1 |
| PWM8 | M4 | TIM11_CH1 | PB9 | AF3 | M4-IN2 |

方向逻辑：IN1=PWM & IN2=0 正转；IN1=0 & IN2=PWM 反转；IN1=IN2=0 滑行；IN1=IN2=1 刹车。
**DRV8871 简化模组无 nSLEEP/nFAULT**——上电即工作，无逐电机故障检测。

### 2.4 I2C1 —— MPU6050 独占（PB6/PB7）

PB6 = I2C1_SCL、PB7 = I2C1_SDA（AF4，4.7kΩ 上拉），400kHz。
- **MPU6050**（整车震动）@ 0x68（AD0 可选 0x69）——**独占总线**：IMU 采样率高、需干净带宽，不与其它设备共享。
- **MPU6050 INT** = PE3（EXTI3，数据就绪中断，上升沿触发）。

### 2.5 电流监测路径 A：ACS712 —— ADC1_IN4（PA4）

PA4 = ADC1_IN4，12bit；前级 RC。ACS712-05B：电流 = (V − 2.5) / 0.185。3.3V ADC 有效 ±4.3A。

### 2.6 TFT 屏接口（板载 J4，已恢复 v1.9）

| 槽位引脚 | 信号 | STM32 引脚 | 说明 |
|---|---|---|---|
| 1 | 3V3 | — | 供电 |
| 2 | GND | — | — |
| 3 | SDI | PB15 | SPI2_MOSI（AF5） |
| 4 | SCL | PB13 | SPI2_SCK（AF5） |
| 5 | CS | PB12 | GPIO 片选 |
| 6 | SDO | PB14 | SPI2_MISO（AF5，多数屏不接） |
| 7 | D/C | PC5 | GPIO 数据/命令 |
| 8 | BLK | PB1 | GPIO 背光 |

驱动方式：硬件 SPI2（21Mbps）或模拟 SPI。**RST 无座脚，模块 RST 飞线接 PE12**。
屏兼容：ST7789/ST7735 可用；LCD12864 不兼容。

### 2.7 I2C3 —— VL53L1X ×3 + INA3221（PA8/PC9）

PA8 = I2C3_SCL、PC9 = I2C3_SDA（AF4，4.7kΩ 上拉），400kHz。
- **VL53L1X ×3**：地址 0x52/0x54/0x56，靠 **XSHUT 分时上电改址**——三片出厂默认 0x52，上电时 XSHUT 全部拉低，再逐个释放改址。
- **INT（GPIO1）×3**：测距就绪中断——前=PE0(EXTI0) / 左=PE1(EXTI1) / 右=PE2(EXTI2)，默认高有效、EXTI 上升沿触发，免轮询 CheckForDataReady。
- **INA3221**（路径 B）：地址 0x40-0x43，与 VL53 零冲突（低采样率，共享无压力）。
- I2C3 备用脚 PH7/PH8 在 LQFP100 未引出，只能用 PA8/PC9。

---

## 3. 端口完整清单

### 3.1 GPIOA

| 引脚 | 分配 | 备注 |
|---|---|---|
| PA0 | 板载 K1 按键 | 保留 |
| PA1 | 板载 LED D2 | 保留 |
| PA2 | TIM5_CH3（PWM3, M2-IN1） | AF2 |
| PA3 | TIM5_CH4（PWM4, M2-IN2） | AF2 |
| PA4 | ADC1_IN4（ACS712） | 模拟 |
| PA5 | 空闲 | — |
| PA6 | TIM3_CH1（编码器 M2） | AF2 |
| PA7 | TIM3_CH2（编码器 M2） | AF2 |
| PA8 | I2C3_SCL（VL53×3 + INA3221） | AF4 |
| PA9 | USART1_TX（调试） | AF7 |
| PA10 | USART1_RX（调试） | AF7 |
| PA11 | 空闲（USB_D-） | — |
| PA12 | 空闲（USB_D+） | — |
| PA13 | SWDIO | 保留 |
| PA14 | SWCLK | 保留 |
| PA15 | 板载 FLASH /CS | 保留 |

### 3.2 GPIOB

| 引脚 | 分配 | 备注 |
|---|---|---|
| PB0 | 空闲 | ADC_IN8 备用 |
| PB1 | TFT BLK（背光） | GPIO 输出 |
| PB2 | BOOT1 | 保留（未引出） |
| PB3 | 板载 FLASH CLK | 保留 |
| PB4 | 板载 FLASH DO | 保留 |
| PB5 | 板载 FLASH DI | 保留 |
| PB6 | I2C1_SCL（MPU6050） | AF4 |
| PB7 | I2C1_SDA（MPU6050） | AF4 |
| PB8 | TIM10_CH1（PWM7, M4-IN1） | AF3 |
| PB9 | TIM11_CH1（PWM8, M4-IN2） | AF3 |
| PB10 | TIM2_CH3（PWM1, M1-IN1） | AF1 |
| PB11 | TIM2_CH4（PWM2, M1-IN2） | AF1 |
| PB12 | TFT CS | GPIO 输出 |
| PB13 | TFT SCK（SPI2_SCK） | AF5 |
| PB14 | TFT SDO（SPI2_MISO） | AF5 |
| PB15 | TFT SDI（SPI2_MOSI） | AF5 |

### 3.3 GPIOC

| 引脚 | 分配 | 备注 |
|---|---|---|
| PC0 | XSHUT_前（VL53L1X） | GPIO 输出，低有效复位 |
| PC1 | XSHUT_左（VL53L1X） | GPIO 输出，低有效复位 |
| PC2 | XSHUT_右（VL53L1X） | GPIO 输出，低有效复位 |
| PC3 | 空闲 | — |
| PC4 | 空闲 | — |
| PC5 | TFT D/C | GPIO 输出 |
| PC6 | TIM8_CH1（编码器 M3） | AF3 |
| PC7 | TIM8_CH2（编码器 M3） | AF3 |
| PC8 | 空闲（原 INT_右） | — |
| PC9 | I2C3_SDA（VL53×3 + INA3221） | AF4 |
| PC10~PC13 | 空闲 | — |
| PC14~PC15 | 未引出 | — |

### 3.4 GPIOD

| 引脚 | 分配 | 备注 |
|---|---|---|
| PD0~PD6 | 空闲 | — |
| PD7~PD11 | 空闲 | — |
| PD12 | TIM4_CH1（编码器 M4） | AF2 |
| PD13 | TIM4_CH2（编码器 M4） | AF2 |
| PD14~PD15 | 空闲 | — |

### 3.5 GPIOE

| 引脚 | 分配 | 备注 |
|---|---|---|
| PE0 | INT_前（VL53L1X） | GPIO 输入，EXTI0 上升沿 |
| PE1 | INT_左（VL53L1X） | GPIO 输入，EXTI1 上升沿 |
| PE2 | INT_右（VL53L1X） | GPIO 输入，EXTI2 上升沿 |
| PE3 | INT（MPU6050） | GPIO 输入，EXTI3 上升沿 |
| PE4 | 空闲 | — |
| PE5 | TIM9_CH1（PWM5, M3-IN1） | AF3 |
| PE6 | TIM9_CH2（PWM6, M3-IN2） | AF3 |
| PE7~PE8 | 空闲 | — |
| PE9 | TIM1_CH1（编码器 M1） | AF1 |
| PE10 | 空闲 | — |
| PE11 | TIM1_CH2（编码器 M1） | AF1 |
| PE12 | TFT RST（飞线） | GPIO 输出 |
| PE13~PE15 | 空闲 | — |

---

## 4. 外设资源占用统计

| 外设 | 已用 | 剩余可用 |
|---|---|---|
| 定时器 | TIM1/3/4/8（编码器）+ TIM2/5/9/10/11（PWM）= **9 个** | TIM6/7/12/13/14（TIM12 引脚被 TFT 占） |
| USART | USART1 | USART2/UART4/5/6 |
| I2C | I2C1（MPU6050 独占）+ I2C3（VL53×3 + INA3221） | I2C2 不可用（M1 PWM 占 PB10/11） |
| SPI | SPI1 板载 FLASH 占；SPI2 → TFT | SPI3 空闲 |
| ADC | ADC1_IN4（1 路） | 大量 |
| GPIO | 39 个（含 XSHUT×3 + INT×4 + I2C3） | 30 个 |

---

## 5. 时钟与带宽约定

| 项 | 值 | 依据 |
|---|---|---|
| 系统时钟 | 168 MHz | 8MHz HSE，PLL M=8/N=336/P=2/Q=7 |
| TIM1/8/9/10/11 | 168 MHz | APB2 ×2 |
| TIM2/3/4/5 | 84 MHz | APB1 ×2 |
| PWM 频率 | 20 kHz | TIM2/5：ARR=4200；TIM9/10/11：ARR=8400 |
| 编码器 | ×4 倍频 | 44 计数/电机轴转；45:1 → 1980 计数/输出轴转 |
| 编码器溢出 | 全 ARR=0xFFFF | 16 位差分，无中断 |
| TFT SPI | 21 Mbps | SPI2 @APB1=42MHz/2 |

---

## 6. 使用约定

1. **板载避让**：PA0/PA1/PA15+PB3~5 绝不初始化。
2. **编码器全纯编码器**：ARR=0xFFFF，读 16 位差分，无中断。
3. **PWM 独立时基**：TIM2/5/9/10/11 各自 ARR 锁 20kHz。
4. **DRV8871 简化模组**：无 nSLEEP/nFAULT；堵转/过流靠 ACS712 整车电流兜底。
5. **编码器电源 5V**：PA6/7、PC6/7、PE9/11、PD12/13 均 FT，直连安全。
6. **TFT RST**：飞线 PE12，软件上电复位（低 10ms → 高）。
7. **SD 卡槽弃用**：PC8-12/PD2 现空闲。
8. **VL53L1X XSHUT**：PC0/1/2 上电先全拉低，逐个释放改址，改址完成前不发测距命令。
8b. **VL53L1X INT**：PE0/1/2 配 EXTI0/1/2 上升沿（GPIO1 默认高有效）；MPU6050 INT = PE3 配 EXTI3。4 个 INT 全独立 EXTI 线（无共享向量）。
9. **I2C 分配**：I2C1=MPU6050 独占（0x68）；I2C3=VL53×3 + INA3221（0x52/54/56 + 0x40-43）；I2C2 不恢复（M1 PWM 占）。

---

## 7. 待确认项

- [x] 开发板 / 晶振 / 调试串口 / DRV8871×4 / 编码器 PPR11 / 5V 供电 / 45:1
- [x] 编码器/PWM 架构 → 全分离（4 纯编码器 + 5 PWM 定时器，共 9 定时器）
- [x] TFT 屏 → 恢复 J4 座 6 信号脚（SPI2），RST 飞线 PE12
- [x] **VL53L1X 多片寻址方案**：恢复 XSHUT×3（PC0/1/2），XSHUT 分时上电改址到 0x52/54/56
- [x] **MPU6050 引脚**：I2C1（0x68，AD0 可选 0x69），独占；INT=PE3(EXTI3)
- [x] **INA3221 引脚**：I2C3（0x40-0x43），与 VL53 共享——是否采用待定
- [ ] 电流监测最终定案（ACS712 vs INA3221）
