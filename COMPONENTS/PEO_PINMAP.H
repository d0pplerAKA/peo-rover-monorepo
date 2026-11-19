/**
 * @file PEO_PINMAP.H
 * @brief STM32F407VGT6 / LQFP100 pinmap, revised 2026-11-16.
 * 3 hardware I2C buses, 3 UARTs, 4 quadrature encoders, 8 motor PWMs.
 * User assignment: LED PA1, button PA0; W25Q16BV on PA15/PB3/PB4/PB5 (SPI1).
 * Encoder numbering: 1=TIM1, 2=TIM3, 3=TIM4, 4=TIM8.
 * PWM numbering: 1/2=TIM5, 3/4=TIM9, 5/6=TIM12, 7=TIM10, 8=TIM11.
 * PB14/PB15 belong to motor PWM: SPI2 TFT cannot use these pins.
 * SD socket must remain unused (PC9 is I2C3 SDA).
 * Existing sensor addresses, geometry and USART2 DMA mapping retained.
 * Pinmap macros do not initialize RCC/GPIO/timers: update driver code
 * that hard-codes old timers, channels, GPIOF, or APB clock selection.
 * Reference: ST DS8626, LQFP100 pin definitions and alternate functions.
 */
#ifndef __PEO_PINMAP_H__
#define __PEO_PINMAP_H__

#include "stm32f4xx.h"

/* ============================================================================
   1. CLOCK TREE AND GLOBAL TIMING
   ----------------------------------------------------------------------------
   HSE 8 MHz --[PLL M=8, N=336, P=2, Q=7]--> SYSCLK 168 MHz
     APB1 prescaler = 4  -> 42 MHz bus, timer clock x2 =  84 MHz (TIM3/TIM4)
     APB2 prescaler = 2  -> 84 MHz bus, timer clock x2 = 168 MHz (TIM1/8/9)
   PWM runs at 20 kHz (above the audible band; DRV8871 needs no dead-time).
   ============================================================================ */
#define PEO_PM_SYSCLK_HZ        (168000000UL)   /* SYSCLK after PLL */
#define PEO_PM_HSE_HZ           (8000000UL)     /* board crystal; MUST match HSE_VALUE! */
#define PEO_PM_TIM_APB2_HZ      (168000000UL)   /* TIM1 / TIM8 / TIM9 / TIM10 / TIM11 */
#define PEO_PM_TIM_APB1_HZ      (84000000UL)    /* TIM2 / TIM3 / TIM4 / TIM5 / TIM12 */

#define PEO_PM_PWM_FREQ_HZ      (20000U)        /* motor PWM frequency */
#define PEO_PM_ARR_APB2         (PEO_PM_TIM_APB2_HZ / PEO_PM_PWM_FREQ_HZ)   /* period counts = 8400; hardware ARR = 8399 */
#define PEO_PM_ARR_APB1         (PEO_PM_TIM_APB1_HZ / PEO_PM_PWM_FREQ_HZ)   /* period counts = 4200; hardware ARR = 4199 */
#define PEO_PM_DUTY_MAX_APB2    (PEO_PM_ARR_APB2)   /* CCR value for 100% duty */
#define PEO_PM_DUTY_MAX_APB1    (PEO_PM_ARR_APB1)

/* ============================================================================
   2. ENCODER AND ODOMETRY CONSTANTS
   ----------------------------------------------------------------------------
   Hall encoder: 11 PPR on the MOTOR shaft.
   TI12 quadrature mode (both edges of both channels) multiplies by 4
   -> 44 counts per motor-shaft revolution.
   Gearbox 45:1 (fixed 2026-11-10) -> 1980 counts per output-shaft rev.
   65 mm wheel -> pi*65/1980 = 0.1031 mm per count; 130 rpm -> 0.44 m/s.
   Velocity formula (sample CNT delta every 1 ms):
     rpm_out = delta_counts * 1000 * 60 / 1980 = delta_counts * 30.3
   ============================================================================ */
#define PEO_PM_ENC_PPR          (11U)               /* pulses per motor-shaft rev */
#define PEO_PM_ENC_MULT         (4U)                /* TI12 both-edge multiplier */
#define PEO_PM_ENC_CNT_PER_REV  (PEO_PM_ENC_PPR * PEO_PM_ENC_MULT)             /* 44 counts / motor rev */
#define PEO_PM_GEAR_RATIO       (45U)               /* gearbox ratio (fixed) */
#define PEO_PM_CNT_PER_OUT_REV  (PEO_PM_ENC_CNT_PER_REV * PEO_PM_GEAR_RATIO)   /* 1980 counts / output rev */
#define PEO_PM_WHEEL_DIA_MM     (65U)               /* wheel diameter in mm */
#define PEO_PM_UM_PER_CNT       (103U)              /* ~0.1031 mm/count -> 103 um/count (approx) */
#define PEO_PM_CNT_TO_RPM       (30U)               /* x30.3: 1ms-sample delta -> output rpm (approx) */

/* ============================================================================
   3. DEBUG USART (USART1, PA9/PA10, AF7, 115200)
   ----------------------------------------------------------------------------
   Connected to an EXTERNAL USB-TTL adapter (3V3 logic).
   ============================================================================ */
/** DEBUG USART **/
#define PEO_PM_DBG_USART        USART1
#define PEO_PM_DBG_BAUDRATE     (115200U)

#define PEO_PM_DBG_AF           GPIO_AF_USART1
#define PEO_PM_DBG_TX_PORT      GPIOA
#define PEO_PM_DBG_TX_PIN       GPIO_Pin_9
#define PEO_PM_DBG_TX_SRC       GPIO_PinSource9
#define PEO_PM_DBG_RX_PORT      GPIOA
#define PEO_PM_DBG_RX_PIN       GPIO_Pin_10
#define PEO_PM_DBG_RX_SRC       GPIO_PinSource10

/* ============================================================================
   3b. ESP32 LINK USART (USART2, PD5/PD6, AF7)  [v1.4 定案 / v2.7 移除 / 重新接入]
   ----------------------------------------------------------------------------
   F407 <-> ESP32-S3 全双工通信链路（一套 RX/TX）。
     PD5 = USART2_TX -> ESP32 RX   （方向1：sensorhub 上报）
     PD6 = USART2_RX <- ESP32 TX   （方向2：命令帧）
   波特率见 PEO_PM_ESP_BAUDRATE。收发均走 DMA：方向1 上报 DMA 发送，方向2 接收 DMA 循环缓冲 + USART IDLE 中断组帧。
   ============================================================================ */
/** ESP32 LINK USART **/
#define PEO_PM_ESP_USART        USART2
#define PEO_PM_ESP_BAUDRATE     (921600U)

#define PEO_PM_ESP_AF           GPIO_AF_USART2
#define PEO_PM_ESP_TX_PORT      GPIOD
#define PEO_PM_ESP_TX_PIN       GPIO_Pin_5
#define PEO_PM_ESP_TX_SRC       GPIO_PinSource5
#define PEO_PM_ESP_RX_PORT      GPIOD
#define PEO_PM_ESP_RX_PIN       GPIO_Pin_6
#define PEO_PM_ESP_RX_SRC       GPIO_PinSource6

/** ESP32 link sensorhub 上报定时器（TIM14 Update 中断，频率 = SENSOR_RATE_REPORT_HZ） **/
#define PEO_PM_ESP_REPORT_TIM   TIM14

/* Third asynchronous UART: USART3 TX=PD8(55), RX=PD9(56), AF7.
 * Baud rate is an application setting; no device-specific rate imposed here. */
#define PEO_PM_UART3_USART USART3
#define PEO_PM_UART3_AF GPIO_AF_USART3
#define PEO_PM_UART3_TX_PORT GPIOD
#define PEO_PM_UART3_TX_PIN GPIO_Pin_8
#define PEO_PM_UART3_TX_SRC GPIO_PinSource8
#define PEO_PM_UART3_RX_PORT GPIOD
#define PEO_PM_UART3_RX_PIN GPIO_Pin_9
#define PEO_PM_UART3_RX_SRC GPIO_PinSource9

/* ============================================================================
   4. QUADRATURE ENCODERS x4  (TIMx CH1 = A, CH2 = B)
   ----------------------------------------------------------------------------
   All four encoder timers are PURE (v1.8): encoder interface mode
   (TIM_EncoderMode_TI12, rising/rising) with ARR = 0xFFFF full 16-bit range.
   Read as 16-bit DELTA (int16_t)(now - last) which self-handles wraparound
   -> NO overflow interrupt on ANY encoder timer.
   PWM was split onto its own timers in v1.8/v1.9 (TIM5/9/10/11/12, section 5).

   M1 (TIM1, PE9/PE11)  - pure encoder, ARR = 0xFFFF
   M2 (TIM3, PA6/PA7)   - pure encoder, ARR = 0xFFFF
   M3 (TIM4, PD12/PD13)   - pure encoder, ARR = 0xFFFF
   M4 (TIM8, PC6/PC7) - pure encoder, ARR = 0xFFFF

   GPIO init: Mode_AF, OType_PP, PuPd_NOPULL (encoder modules have their
   own bus pull-ups; enable MCU pull-ups only for long wiring).
   ============================================================================ */

/** ENCODER M1 - left front (TIM1 CH1/CH2 = PE9/PE11, AF1) **/
#define PEO_PM_ENC1_TIM         TIM1
#define PEO_PM_ENC1_ARR         (0xFFFFU)                  /* pure encoder, 16-bit delta */
#define PEO_PM_ENC1_AF          GPIO_AF_TIM1
#define PEO_PM_ENC1_CH1_PORT    GPIOE
#define PEO_PM_ENC1_CH1_PIN     GPIO_Pin_9
#define PEO_PM_ENC1_CH1_SRC     GPIO_PinSource9
#define PEO_PM_ENC1_CH2_PORT    GPIOE
#define PEO_PM_ENC1_CH2_PIN     GPIO_Pin_11
#define PEO_PM_ENC1_CH2_SRC     GPIO_PinSource11

/** ENCODER M2 - right rear (TIM3 CH1/CH2 = PA6/PA7, AF2, pure encoder) **/
#define PEO_PM_ENC2_TIM         TIM3
#define PEO_PM_ENC2_ARR         (0xFFFFU)                  /* full 16-bit range, delta reads */
#define PEO_PM_ENC2_AF          GPIO_AF_TIM3
#define PEO_PM_ENC2_CH1_PORT    GPIOA
#define PEO_PM_ENC2_CH1_PIN     GPIO_Pin_6
#define PEO_PM_ENC2_CH1_SRC     GPIO_PinSource6
#define PEO_PM_ENC2_CH2_PORT    GPIOA
#define PEO_PM_ENC2_CH2_PIN     GPIO_Pin_7
#define PEO_PM_ENC2_CH2_SRC     GPIO_PinSource7

/** ENCODER M3 - left (TIM4 CH1/CH2 = PD12(59)/PD13(60), AF2) **/
#define PEO_PM_ENC3_TIM                  TIM4
#define PEO_PM_ENC3_ARR         (0xFFFFU)                  /* pure encoder, 16-bit delta */
#define PEO_PM_ENC3_AF                   GPIO_AF_TIM4
#define PEO_PM_ENC3_CH1_PORT             GPIOD
#define PEO_PM_ENC3_CH1_PIN              GPIO_Pin_12
#define PEO_PM_ENC3_CH1_SRC              GPIO_PinSource12
#define PEO_PM_ENC3_CH2_PORT             GPIOD
#define PEO_PM_ENC3_CH2_PIN              GPIO_Pin_13
#define PEO_PM_ENC3_CH2_SRC              GPIO_PinSource13

/** ENCODER M4 - right (TIM8 CH1/CH2 = PC6(63)/PC7(64), AF3) **/
#define PEO_PM_ENC4_TIM                  TIM8
#define PEO_PM_ENC4_ARR         (0xFFFFU)                  /* pure encoder, 16-bit delta */
#define PEO_PM_ENC4_AF                   GPIO_AF_TIM8
#define PEO_PM_ENC4_CH1_PORT             GPIOC
#define PEO_PM_ENC4_CH1_PIN              GPIO_Pin_6
#define PEO_PM_ENC4_CH1_SRC              GPIO_PinSource6
#define PEO_PM_ENC4_CH2_PORT             GPIOC
#define PEO_PM_ENC4_CH2_PIN              GPIO_Pin_7
#define PEO_PM_ENC4_CH2_SRC              GPIO_PinSource7

/* ============================================================================
   5. MOTOR PWM x8 (two inputs per DRV8871 motor driver)
   M1: TIM5 PA2/PA3; M2: TIM9 PE5/PE6;
   M3: TIM12 PB14/PB15; M4: TIM10 PB8 + TIM11 PB9.
   PWM1 mode, PSC=0. Legacy PEO_PM_ARR_APBx means period COUNTS;
   write hardware ARR = PEO_PM_ARR_APBx - 1. Enable CCR preload.
   CCR=0 is 0%; CCR=period counts is 100% in edge-aligned PWM1.
   Set each timer to 20 kHz using its own APB timer clock.
   ============================================================================ */

/* PWM1: TIM5_CH3, PA2, LQFP100 pin 25. */
#define PEO_PM_PWM1_TIM                  TIM5
#define PEO_PM_PWM1_CH                   TIM_Channel_3
#define PEO_PM_PWM1_PORT                 GPIOA
#define PEO_PM_PWM1_PIN                  GPIO_Pin_2
#define PEO_PM_PWM1_SRC                  GPIO_PinSource2
#define PEO_PM_PWM1_AF                   GPIO_AF_TIM5
#define PEO_PM_PWM1_PHYS_PIN             (25U)
#define PEO_PM_PWM1_TIM_HZ               PEO_PM_TIM_APB1_HZ

/* PWM2: TIM5_CH4, PA3, LQFP100 pin 26. */
#define PEO_PM_PWM2_TIM                  TIM5
#define PEO_PM_PWM2_CH                   TIM_Channel_4
#define PEO_PM_PWM2_PORT                 GPIOA
#define PEO_PM_PWM2_PIN                  GPIO_Pin_3
#define PEO_PM_PWM2_SRC                  GPIO_PinSource3
#define PEO_PM_PWM2_AF                   GPIO_AF_TIM5
#define PEO_PM_PWM2_PHYS_PIN             (26U)
#define PEO_PM_PWM2_TIM_HZ               PEO_PM_TIM_APB1_HZ

/* PWM3: TIM9_CH1, PE5, LQFP100 pin 4. */
#define PEO_PM_PWM3_TIM                  TIM9
#define PEO_PM_PWM3_CH                   TIM_Channel_1
#define PEO_PM_PWM3_PORT                 GPIOE
#define PEO_PM_PWM3_PIN                  GPIO_Pin_5
#define PEO_PM_PWM3_SRC                  GPIO_PinSource5
#define PEO_PM_PWM3_AF                   GPIO_AF_TIM9
#define PEO_PM_PWM3_PHYS_PIN             (4U)
#define PEO_PM_PWM3_TIM_HZ               PEO_PM_TIM_APB2_HZ

/* PWM4: TIM9_CH2, PE6, LQFP100 pin 5. */
#define PEO_PM_PWM4_TIM                  TIM9
#define PEO_PM_PWM4_CH                   TIM_Channel_2
#define PEO_PM_PWM4_PORT                 GPIOE
#define PEO_PM_PWM4_PIN                  GPIO_Pin_6
#define PEO_PM_PWM4_SRC                  GPIO_PinSource6
#define PEO_PM_PWM4_AF                   GPIO_AF_TIM9
#define PEO_PM_PWM4_PHYS_PIN             (5U)
#define PEO_PM_PWM4_TIM_HZ               PEO_PM_TIM_APB2_HZ

/* PWM5: TIM12_CH1, PB14, LQFP100 pin 53. */
#define PEO_PM_PWM5_TIM                  TIM12
#define PEO_PM_PWM5_CH                   TIM_Channel_1
#define PEO_PM_PWM5_PORT                 GPIOB
#define PEO_PM_PWM5_PIN                  GPIO_Pin_14
#define PEO_PM_PWM5_SRC                  GPIO_PinSource14
#define PEO_PM_PWM5_AF                   GPIO_AF_TIM12
#define PEO_PM_PWM5_PHYS_PIN             (53U)
#define PEO_PM_PWM5_TIM_HZ               PEO_PM_TIM_APB1_HZ

/* PWM6: TIM12_CH2, PB15, LQFP100 pin 54. */
#define PEO_PM_PWM6_TIM                  TIM12
#define PEO_PM_PWM6_CH                   TIM_Channel_2
#define PEO_PM_PWM6_PORT                 GPIOB
#define PEO_PM_PWM6_PIN                  GPIO_Pin_15
#define PEO_PM_PWM6_SRC                  GPIO_PinSource15
#define PEO_PM_PWM6_AF                   GPIO_AF_TIM12
#define PEO_PM_PWM6_PHYS_PIN             (54U)
#define PEO_PM_PWM6_TIM_HZ               PEO_PM_TIM_APB1_HZ

/* PWM7: TIM10_CH1, PB8, LQFP100 pin 95. */
#define PEO_PM_PWM7_TIM                  TIM10
#define PEO_PM_PWM7_CH                   TIM_Channel_1
#define PEO_PM_PWM7_PORT                 GPIOB
#define PEO_PM_PWM7_PIN                  GPIO_Pin_8
#define PEO_PM_PWM7_SRC                  GPIO_PinSource8
#define PEO_PM_PWM7_AF                   GPIO_AF_TIM10
#define PEO_PM_PWM7_PHYS_PIN             (95U)
#define PEO_PM_PWM7_TIM_HZ               PEO_PM_TIM_APB2_HZ

/* PWM8: TIM11_CH1, PB9, LQFP100 pin 96. */
#define PEO_PM_PWM8_TIM                  TIM11
#define PEO_PM_PWM8_CH                   TIM_Channel_1
#define PEO_PM_PWM8_PORT                 GPIOB
#define PEO_PM_PWM8_PIN                  GPIO_Pin_9
#define PEO_PM_PWM8_SRC                  GPIO_PinSource9
#define PEO_PM_PWM8_AF                   GPIO_AF_TIM11
#define PEO_PM_PWM8_PHYS_PIN             (96U)
#define PEO_PM_PWM8_TIM_HZ               PEO_PM_TIM_APB2_HZ
#define PEO_PM_PWM1_2_AF GPIO_AF_TIM5
#define PEO_PM_PWM3_4_AF GPIO_AF_TIM9
#define PEO_PM_PWM5_6_AF GPIO_AF_TIM12
#define PEO_PM_PWM7_8_AF GPIO_AF_TIM10
/* TIM10 and TIM11 both use GPIO AF3; per-channel AF macros above preferred. */

/* ============================================================================
   6. DRV8871 MOTOR DRIVER MODULES (simplified, v1.8)
   ----------------------------------------------------------------------------
   The DRV8871 modules in use expose ONLY: IN1, IN2 (PWM inputs) and
   OUT1, OUT2 (to motor), plus VM/GND. They do NOT bring out nSLEEP or
   nFAULT - so this design has no sleep control and no per-motor fault pin.
   Consequence: stall/over-current detection relies on the per-motor
   INA3221 current monitor (I2C2, section 7c).
   ============================================================================ */

/* ============================================================================
   7. I2C BUS - I2C1 (PB6/PB7, AF4, 400 kHz) - MPU6050 DEDICATED (v2.3)
   ----------------------------------------------------------------------------
   DEDICATED to the MPU6050 alone (whole-vehicle vibration). The IMU may
   sample faster than the rangefinders/current monitor, so it gets a clean
   bus with no other traffic.
     MPU6050   0x68 (or 0x69 via AD0)
   I2C2 uses PB10/PB11 for INA3221 current monitors.
   ============================================================================ */

/** I2C1 - MPU6050 dedicated bus (PB6/PB7, AF4) **/
#define PEO_PM_MPU6050_I2C          I2C1
#define PEO_PM_MPU6050_I2C_CLKSPEED (400000U)
#define PEO_PM_MPU6050_SCL_PORT     GPIOB
#define PEO_PM_MPU6050_SCL_PIN      GPIO_Pin_6
#define PEO_PM_MPU6050_SCL_SRC      GPIO_PinSource6
#define PEO_PM_MPU6050_SDA_PORT     GPIOB
#define PEO_PM_MPU6050_SDA_PIN      GPIO_Pin_7
#define PEO_PM_MPU6050_SDA_SRC      GPIO_PinSource7
#define PEO_PM_MPU6050_I2C_AF       GPIO_AF_I2C1

/** I2C1 slave address (7-bit) **/
#define PEO_PM_MPU6050_ADDR     (0x68U)         /* whole-vehicle vibration */

/* ============================================================================
   7b. I2C BUS - I2C3 (PA8/PC9, AF4, 400 kHz) - VL53L1X x3 (INA3221 moved to I2C2)
   ----------------------------------------------------------------------------
   Dedicated bus for the 3x VL53L1X rangefinders only:
     VL53L1X front    0x52
     VL53L1X left     0x54
     VL53L1X right    0x56
   Re-addressing (XSHUT sequential): all three VL53L1X ship at factory default 0x52.
   XSHUT (section 9) is held LOW at power-up, then released one sensor at a
   time to assign the address above before the next sensor is released.
   NOTES:
     - I2C3 alternate pins PH7/PH8 are NOT broken out on LQFP100, so PA8/PC9
       are the only option. PA8 (also TIM1_CH1) and PC9 (also TIM8_CH4) are
       free because TIM1/TIM8 use their CH1/CH2 pins (PE9/11, PC6/7).
   ============================================================================ */

/** I2C3 - VL53L1X dedicated bus (PA8/PC9, AF4) **/
#define PEO_PM_VL53_I2C          I2C3
#define PEO_PM_VL53_I2C_CLKSPEED (400000U)
#define PEO_PM_VL53_SCL_PORT     GPIOA
#define PEO_PM_VL53_SCL_PIN      GPIO_Pin_8
#define PEO_PM_VL53_SCL_SRC      GPIO_PinSource8
#define PEO_PM_VL53_SDA_PORT     GPIOC
#define PEO_PM_VL53_SDA_PIN      GPIO_Pin_9
#define PEO_PM_VL53_SDA_SRC      GPIO_PinSource9
#define PEO_PM_VL53_I2C_AF       GPIO_AF_I2C3

/** VL53 driver address convention retained: 8-bit shifted addresses.
 * Equivalent 7-bit addresses: 0x29 / 0x2A / 0x2B. */
#define PEO_PM_VL53_DEFAULT_ADDR        (0x52U)
#define PEO_PM_VL53_FRONT_ADDR          (0x52U)          /* I2C3 */
#define PEO_PM_VL53_LEFT_ADDR           (0x54U)          /* I2C3 */
#define PEO_PM_VL53_RIGHT_ADDR          (0x56U)          /* I2C3 */
/* ============================================================================
   7c. I2C BUS - I2C2 (PB10=SCL/PB11=SDA, AF4, 400 kHz) - INA3221
   ----------------------------------------------------------------------------
   Dedicated bus for the INA3221 current monitor (moved OFF I2C3 so the
   rangefinder bus stays clean):
     INA3221          0x40  (3-channel current/voltage monitor, low rate)
   PB10/PB11 are dedicated to I2C2; GPIOF is not available on LQFP100.
   ============================================================================ */

/** I2C2 - INA3221 dedicated bus (PB10=SCL/PB11=SDA, AF4) **/
#define PEO_PM_INA_I2C          I2C2
#define PEO_PM_INA_I2C_CLKSPEED (400000U)
#define PEO_PM_INA_SCL_PORT              GPIOB
#define PEO_PM_INA_SCL_PIN               GPIO_Pin_10
#define PEO_PM_INA_SCL_SRC               GPIO_PinSource10
#define PEO_PM_INA_SDA_PORT              GPIOB
#define PEO_PM_INA_SDA_PIN               GPIO_Pin_11
#define PEO_PM_INA_SDA_SRC               GPIO_PinSource11
#define PEO_PM_INA_I2C_AF       GPIO_AF_I2C2

/** I2C2 slave addresses (7-bit) — two INA3221 share the bus **/
#define PEO_PM_INA3221_ADDR_A   (0x40U)   /* 片A：A0=GND，测 M1/M2 */
#define PEO_PM_INA3221_ADDR_B   (0x41U)   /* 片B：A0=VS，测 M3/M4 */

/* ============================================================================
   9. VL53L1X XSHUT x3  (restored v2.0)
   ----------------------------------------------------------------------------
   GPIO outputs, active-low reset per sensor. Held LOW at power-up to keep all
   three at the factory default 0x52, then released one at a time so
   each can be re-addressed (see section 7) before the next is released.
     XSHUT front = PC0    XSHUT left = PC1    XSHUT right = PC2
   Push-pull output, no pull-up needed (VL53L1X has an internal XSHUT pull-up).
   ============================================================================ */

/** VL53L1X XSHUT (active-low; low = held in reset) **/
#define PEO_PM_VL53_XSHUT_PORT              GPIOC
#define PEO_PM_VL53_FRONT_XSHUT_PIN         GPIO_Pin_0
#define PEO_PM_VL53_LEFT_XSHUT_PIN          GPIO_Pin_1
#define PEO_PM_VL53_RIGHT_XSHUT_PIN         GPIO_Pin_2

/* ============================================================================
   9b. VL53L1X INT x3 + MPU6050 INT  (data-ready interrupt outputs)  [v2.6]
   ----------------------------------------------------------------------------
   VL53L1X GPIO1 asserts when a ranging result is ready (active-high default).
   MPU6050 INT asserts on data-ready (or FIFO/motion, per MPU6050_Config).
   Each sensor INT is wired to a dedicated MCU input + EXTI line so the host
   reads results interrupt-driven instead of polling.
     VL53L1X INT front = PE0 (EXTI0)   left = PE1 (EXTI1)   right = PE2 (EXTI2)
     MPU6050 INT      = PE3 (EXTI3)
   MCU side: floating input (sensor drives push-pull); EXTI rising edge.
   All four on INDEPENDENT EXTI lines - no shared EXTI9_5 vector.
   ============================================================================ */

/** VL53L1X INT (GPIO1, active-high data-ready) **/
#define PEO_PM_VL53_INT_PORT                GPIOE
#define PEO_PM_VL53_FRONT_INT_PIN           GPIO_Pin_0
#define PEO_PM_VL53_LEFT_INT_PIN            GPIO_Pin_1
#define PEO_PM_VL53_RIGHT_INT_PIN           GPIO_Pin_2

/** MPU6050 INT (data-ready / FIFO / motion, per config) **/
#define PEO_PM_MPU_INT_PORT           GPIOE
#define PEO_PM_MPU_INT_PIN            GPIO_Pin_3

/** EXTI lines / pin sources (RM0090 sec.10.2) **/
#define PEO_PM_VL53_FRONT_INT_EXTI_LINE     EXTI_Line0
#define PEO_PM_VL53_FRONT_INT_EXTI_PSRC     EXTI_PortSourceGPIOE
#define PEO_PM_VL53_FRONT_INT_EXTI_SRC      EXTI_PinSource0

#define PEO_PM_VL53_LEFT_INT_EXTI_LINE      EXTI_Line1
#define PEO_PM_VL53_LEFT_INT_EXTI_PSRC      EXTI_PortSourceGPIOE
#define PEO_PM_VL53_LEFT_INT_EXTI_SRC       EXTI_PinSource1

#define PEO_PM_VL53_RIGHT_INT_EXTI_LINE     EXTI_Line2
#define PEO_PM_VL53_RIGHT_INT_EXTI_PSRC     EXTI_PortSourceGPIOE
#define PEO_PM_VL53_RIGHT_INT_EXTI_SRC      EXTI_PinSource2

#define PEO_PM_MPU_INT_EXTI_LINE            EXTI_Line3
#define PEO_PM_MPU_INT_EXTI_SRC             EXTI_PinSource3


/* ============================================================================
   10. EXTERNAL FLASH - W25Q16BV (SPI1, APB2)
   Flash pin   Signal     Board connection     MCU LQFP100 pin
       1       /CS        PA15                 77
       2       DO         PB4 / SPI1_MISO      90
       3       /WP        +3.3 V (no GPIO)
       4       GND        GND
       5       DI         PB5 / SPI1_MOSI      91
       6       CLK        PB3 / SPI1_SCK       89
       7       /HOLD      +3.3 V (no GPIO)
       8       VCC        +3.3 V

   Use SPI1 AF5 on PB3/PB4/PB5. PA15 is software-controlled GPIO CS,
   NOT an AF pin: push-pull output, idle HIGH, active LOW.
   Set the PA15 output latch HIGH before enabling output mode.
   Configure SPI master with software NSS management (SPI_NSS_Soft).
   Enable GPIOA/GPIOB on AHB1 and SPI1 on APB2 before initialization.
   Release JTAG functions on PA15/PB3/PB4 and do not enable SWO on PB3;
   retain SWD on PA13/PA14. This pinmap does not initialize the flash.
   ============================================================================ */
#define PEO_PM_FLASH_SPI                  SPI1
#define PEO_PM_FLASH_SPI_AF               GPIO_AF_SPI1
#define PEO_PM_FLASH_SPI_RCC              RCC_APB2Periph_SPI1
#define PEO_PM_FLASH_GPIO_RCC             (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB)
#define PEO_PM_FLASH_CS_PORT              GPIOA
#define PEO_PM_FLASH_CS_PIN               GPIO_Pin_15
#define PEO_PM_FLASH_CS_PHYS_PIN          (77U)
#define PEO_PM_FLASH_CS_ACTIVE_LEVEL      Bit_RESET
#define PEO_PM_FLASH_CS_IDLE_LEVEL        Bit_SET
#define PEO_PM_FLASH_SCK_PORT             GPIOB
#define PEO_PM_FLASH_SCK_PIN              GPIO_Pin_3
#define PEO_PM_FLASH_SCK_SRC              GPIO_PinSource3
#define PEO_PM_FLASH_SCK_PHYS_PIN         (89U)
#define PEO_PM_FLASH_MISO_PORT            GPIOB
#define PEO_PM_FLASH_MISO_PIN             GPIO_Pin_4
#define PEO_PM_FLASH_MISO_SRC             GPIO_PinSource4
#define PEO_PM_FLASH_MISO_PHYS_PIN        (90U)
#define PEO_PM_FLASH_MOSI_PORT            GPIOB
#define PEO_PM_FLASH_MOSI_PIN             GPIO_Pin_5
#define PEO_PM_FLASH_MOSI_SRC             GPIO_PinSource5
#define PEO_PM_FLASH_MOSI_PHYS_PIN        (91U)

/* ============================================================================
   11. USER-RESERVED GPIO AND DEBUG
   LED PA1(24), button PA0(23), per user assignment (not board-label inference).
   PA15(77), PB3(89), PB4(90), PB5(91): W25Q16BV SPI1, see section 10.
   SWD PA13(72)/PA14(76), NRST(14): retain debug/reset.
   USB PA11/PA12 retained. SD socket unused: PC9 belongs to I2C3.
   Button is polled: EXTI1 already belongs to VL53 left INT on PE1.
   ============================================================================ */
#define PEO_PM_LED_PORT GPIOA
#define PEO_PM_LED_PIN GPIO_Pin_1
#define PEO_PM_LED_ON_LEVEL Bit_RESET /* Existing active-low convention retained. */
#define PEO_PM_BUTTON_PORT GPIOA
#define PEO_PM_BUTTON_PIN GPIO_Pin_0

/* ============================================================================
   12. INTERRUPT HANDLER NAMES (StdPeriph, startup_stm32f40xx.s)
   ----------------------------------------------------------------------------
   ALL four encoders are pure (ARR = 0xFFFF), read as 16-bit DELTA - no
   encoder timer needs an Update interrupt. The PWM timers (TIM5/9/10/11/12)
   are edge-aligned with no interrupt either. Encoder/PWM operation needs no timer IRQ; other application timers
   (for example the ESP32 report timer) may still use interrupts.
   ============================================================================ */

/* ============================================================================
   13. DMA STREAM / CHANNEL MAP  (RM0090 Table 43 - DMA1 request mapping)
   ----------------------------------------------------------------------------
   Hardware routing facts - they live HERE (not in a component header) so
   every component can see them through the PEO_PINMAP.H include chain.
   ----------------------------------------------------------------------------
   | Peripheral | Direction | Stream / Channel | IRQ channel (see nvic.h) |
   |------------|-----------|------------------|--------------------------|
   SPI2 TFT is not allocated: PB14/PB15 are motor PWM outputs.
   ----------------------------------------------------------------------------
   ============================================================================ */


/** ESP32 link USART2 DMA (RM0090 Table 43 - DMA1 request mapping) **/
#define PEO_DMA_ESP_RX_STREAM   DMA1_Stream5
#define PEO_DMA_ESP_RX_CH       DMA_Channel_4
#define PEO_DMA_ESP_TX_STREAM   DMA1_Stream6
#define PEO_DMA_ESP_TX_CH       DMA_Channel_4

/** 手柄 USART3 DMA (RM0090 Table 43 - DMA1 request mapping) **/
#define PEO_DMA_GAMEPAD_RX_STREAM  DMA1_Stream1
#define PEO_DMA_GAMEPAD_RX_CH      DMA_Channel_4

/* ============================================================================
   14. EXTI LINES
   ----------------------------------------------------------------------------
   v2.6: INT x4 -> VL53L1X front/left/right = EXTI0/1/2 (PE0/1/2); MPU6050 =
   EXTI3 (PE3). All four on INDEPENDENT EXTI lines (EXTI0-3 each have their
   own IRQ vector). Pin/source macros live in section 9b.
   ============================================================================ */

#endif /* __PEO_PINMAP_H__ */
