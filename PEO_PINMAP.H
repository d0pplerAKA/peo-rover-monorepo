/**
 ******************************************************************************
 * @file    PEO_PINMAP.H
 * @brief   PEO Rover - STM32F407VGT6 pin assignment header
 *          (single source of truth for all pin/peripheral macros)
 *
 * @version v2.3.0  (2026-11-11)
 * @board   DevEBox STM32F4XX-M (MCUDev), LQFP100, on-board 8 MHz HSE
 * @basedon PEO_ROVER_F407_PinMap.md v2.2 (same folder, authoritative doc)
 *
 * ----------------------------------------------------------------------------
 * BOARD FACTS THAT SHAPED THIS FILE (from stm32-base.org + datasheet):
 *  1. On-board crystal is 8 MHz HSE. The StdPeriph library defaults
 *     HSE_VALUE to 25 MHz - it MUST be changed to 8000000 in
 *     stm32f4xx_conf.h (or project defines), otherwise the PLL settings
 *     below (M=8/N=336/P=2/Q=7) do NOT produce 168 MHz.
 *  2. The SD socket (PC8-PC12, PD2) is NOT used by this design. Since v1.8
 *     all these pins are FREE and may be repurposed. Leave the socket empty -
 *     do not insert a card while this design runs on the board.
 *  3. TFT socket J4 (8x1) has NO RST pin. ST7789/ST7735 modules need one:
 *     wire module RST to PE12 (flywire) - see section 11.
 *     LCD12864 is NOT compatible with J4 (parallel bus / no D/C concept /
 *     usually 5 V powered) - do not buy one.
 *  4. Hall encoder modules are 5 V powered with built-in bus pull-ups.
 *     All four encoder pin groups used here are 5 V-tolerant (FT), so the
 *     5 V A/B signals may be wired directly to the MCU.
 *  5. Debug UART is an EXTERNAL USB-TTL adapter on PA9/PA10 (the board's
 *     CH340 is not used).
 *
 * REVISION HISTORY:
 *  v1.0  Initial pinmap for STM32F411CEU6 (superseded - pin conflict).
 *  v1.2  First complete F407 pinmap (symmetric: encoder CH1/2 + PWM CH3/4
 *        on the same timer for each motor).
 *  v1.3  M2 PWM moved from TIM3_CH3/4 (PB0/PB1) to TIM9_CH1/2 (PA2/PA3);
 *        PB1 returned to TFT BLK -> display can be plugged in.
 *  v1.4  Gear ratio fixed at 45:1; ESP32 link fixed as UART; TFT
 *        compatibility decided (ST7789/ST7735 OK + RST flywire PE12;
 *        LCD12864 rejected).
 *  v1.5  Current-sensing dual path reserved:
 *          path A: ACS712 (analog)  on PA4  = ADC1_IN4
 *          path B: INA3221 (I2C)    on I2C2, addresses 0x40-0x43
 *        Board-level fact: a SECOND hardware I2C bus is impossible on
 *        LQFP100 (I2C1 pins PB6-PB9 are taken by M4 encoder/PWM; I2C3
 *        needs PC9 which is M3 PWM; no PF pins on 100-pin package).
 *        -> spare software-I2C pin pair reserved: PE0 (SCL) / PE1 (SDA).
 *  v1.6  DMA stream/channel map (section 15) and EXTI line map (section 16)
 *        moved HERE from the NVIC component header: hardware routing facts
 *        must be visible to every component (fixes usart_comm.c build).
 *  v1.7  Second HARDWARE I2C recovered (v1.5 "impossible" superseded):
 *        M4 motor moved from TIM4 PB6-PB9 to TIM4 PD12-PD15 (all FT, same
 *        AF2, same timer/time-base - a pure pin move). Frees PB6/PB7 for a
 *        DEDICATED I2C1 serving the 3x VL53L0X; I2C2 keeps MPU6050+INA3221.
 *        Software-I2C spare (PE0/PE1) removed.
 *  v1.8  Encoders and PWM fully separated onto 8 timers (4 enc + 4 PWM):
 *        - ALL encoders are PURE (ARR=0xFFFF, 16-bit delta, no Update IRQ).
 *        - PWM moved to TIM2 (PB10/11), TIM5 (PA2/3), TIM9 (PE5/6),
 *          TIM12 (PB14/15) - one timer per motor, 2 adjacent channels.
 *        - DRV8871 modules are SIMPLIFIED: only IN1/IN2 + OUT1/2, no nSLEEP
 *          and no nFAULT -> both pin groups removed (PE2-5, PC10-12, PD2).
 *        - I2C2 (PB10/11) and TFT SPI2 (PB14/15) pins taken by PWM; their
 *          macro blocks are removed (sensor/display TBD).
 *  v1.9  TFT restored + XSHUT removed:
 *        - M4 PWM moved off TIM12 (PB14/15) to TIM10_CH1(PB8) + TIM11_CH1(PB9)
 *          so PB14/PB15 return to the TFT SPI2 pins. PWM timers become 5
 *          (TIM2/5/9 + TIM10 + TIM11).
 *        - TFT (J4) macro block restored: CS=PB12 SCK=PB13 MISO=PB14
 *          MOSI=PB15 DC=PC5 BLK=PB1 RST=PE12.
 *        - XSHUT x3 (PC0/PC1/PC2) removed - VL53L0X addressing TBD.
 *  v2.0  XSHUT x3 RESTORED (VL53L0X re-addressing is mandatory - see 10):
 *        PC0 = front, PC1 = left, PC2 = right (AN4846 sequential power-up ->
 *        addresses 0x52/0x54/0x56).
 *        Comment clean-up: SD-socket "reused" board fact (section 2) and every
 *        stale "TIM2/5/9/12" PWM-timer reference corrected to TIM2/5/9/10/11.
 *  v2.1  I2C3 reserved for MPU6050 + INA3221 (PA8/SCL, PC9/SDA, AF4, 400 kHz).
 *        I2C2 stays unavailable: M1 PWM sits on TIM2 PB10/11 and the encoder
 *        timers (SMS slave-mode counter) cannot co-host a constant-freq PWM
 *        (RM0090 S18.3.12 "external clock with direction selection").
 *  v2.2  INA3221 moved OFF I2C3 onto I2C1 (shares with 3x VL53L0X, 0x40-43).
 *        I2C3 is now DEDICATED to MPU6050 alone - the IMU may sample faster
 *        than the current monitor, so it gets a clean bus with no other
 *        traffic.
 *  v2.3  SWAPPED the two I2C buses (per user): I2C1 (PB6/PB7) is now the
 *        MPU6050 DEDICATED bus; I2C3 (PA8/PC9) carries 3x VL53L0X + INA3221.
 *        Addresses unchanged (0x68 vs 0x52/54/56 + 0x40-43).
 ******************************************************************************
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
#define PEO_PM_TIM_APB1_HZ      (84000000UL)    /* TIM2 / TIM3 / TIM4 / TIM5 */

#define PEO_PM_PWM_FREQ_HZ      (20000U)        /* motor PWM frequency */
#define PEO_PM_ARR_APB2         (PEO_PM_TIM_APB2_HZ / PEO_PM_PWM_FREQ_HZ)   /* = 8400 */
#define PEO_PM_ARR_APB1         (PEO_PM_TIM_APB1_HZ / PEO_PM_PWM_FREQ_HZ)   /* = 4200 */
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
   Connected to an EXTERNAL USB-TTL adapter (3V3 logic). Do not confuse
   with the ESP32 link on USART2 (section 4).
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
   4. ESP32-S3 HIGH-SPEED LINK (USART2, PD5/PD6, AF7, 4.2 Mbps, DMA)
   ----------------------------------------------------------------------------
   Frame protocol, DMA TX/RX recommended. At 1 kHz control rate the
   payload is ~27 KB/s = 7% of the 4.2 Mbps line - large headroom.
   ============================================================================ */
/** ESP32 COMM USART **/
#define PEO_PM_COMM_USART       USART2
#define PEO_PM_COMM_BAUDRATE    (4200000U)

#define PEO_PM_COMM_AF          GPIO_AF_USART2
#define PEO_PM_COMM_TX_PORT     GPIOD
#define PEO_PM_COMM_TX_PIN      GPIO_Pin_5
#define PEO_PM_COMM_TX_SRC      GPIO_PinSource5
#define PEO_PM_COMM_RX_PORT     GPIOD
#define PEO_PM_COMM_RX_PIN      GPIO_Pin_6
#define PEO_PM_COMM_RX_SRC      GPIO_PinSource6

/* ============================================================================
   5. QUADRATURE ENCODERS x4  (TIMx CH1 = A, CH2 = B)
   ----------------------------------------------------------------------------
   All four encoder timers are PURE (v1.8): encoder interface mode
   (TIM_EncoderMode_TI12, rising/rising) with ARR = 0xFFFF full 16-bit range.
   Read as 16-bit DELTA (int16_t)(now - last) which self-handles wraparound
   -> NO overflow interrupt on ANY encoder timer.
   PWM was split onto its own timers in v1.8/v1.9 (TIM2/5/9/10/11, section 6).

   M1 (TIM1, PE9/PE11)  - pure encoder, ARR = 0xFFFF
   M2 (TIM3, PA6/PA7)   - pure encoder, ARR = 0xFFFF
   M3 (TIM8, PC6/PC7)   - pure encoder, ARR = 0xFFFF
   M4 (TIM4, PD12/PD13) - pure encoder, ARR = 0xFFFF

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

/** ENCODER M2 - left rear (TIM3 CH1/CH2 = PA6/PA7, AF2, pure encoder) **/
#define PEO_PM_ENC2_TIM         TIM3
#define PEO_PM_ENC2_ARR         (0xFFFFU)                  /* full 16-bit range, delta reads */
#define PEO_PM_ENC2_AF          GPIO_AF_TIM3
#define PEO_PM_ENC2_CH1_PORT    GPIOA
#define PEO_PM_ENC2_CH1_PIN     GPIO_Pin_6
#define PEO_PM_ENC2_CH1_SRC     GPIO_PinSource6
#define PEO_PM_ENC2_CH2_PORT    GPIOA
#define PEO_PM_ENC2_CH2_PIN     GPIO_Pin_7
#define PEO_PM_ENC2_CH2_SRC     GPIO_PinSource7

/** ENCODER M3 - right front (TIM8 CH1/CH2 = PC6/PC7, AF3) **/
#define PEO_PM_ENC3_TIM         TIM8
#define PEO_PM_ENC3_ARR         (0xFFFFU)                  /* pure encoder, 16-bit delta */
#define PEO_PM_ENC3_AF          GPIO_AF_TIM8
#define PEO_PM_ENC3_CH1_PORT    GPIOC
#define PEO_PM_ENC3_CH1_PIN     GPIO_Pin_6
#define PEO_PM_ENC3_CH1_SRC     GPIO_PinSource6
#define PEO_PM_ENC3_CH2_PORT    GPIOC
#define PEO_PM_ENC3_CH2_PIN     GPIO_Pin_7
#define PEO_PM_ENC3_CH2_SRC     GPIO_PinSource7

/** ENCODER M4 - right rear (TIM4 CH1/CH2 = PD12/PD13, AF2) **/
#define PEO_PM_ENC4_TIM         TIM4
#define PEO_PM_ENC4_ARR         (0xFFFFU)                  /* pure encoder, 16-bit delta */
#define PEO_PM_ENC4_AF          GPIO_AF_TIM4
#define PEO_PM_ENC4_CH1_PORT    GPIOD
#define PEO_PM_ENC4_CH1_PIN     GPIO_Pin_12
#define PEO_PM_ENC4_CH1_SRC     GPIO_PinSource12
#define PEO_PM_ENC4_CH2_PORT    GPIOD
#define PEO_PM_ENC4_CH2_PIN     GPIO_Pin_13
#define PEO_PM_ENC4_CH2_SRC     GPIO_PinSource13

/* ============================================================================
   6. MOTOR PWM x8  ->  4x DRV8871 (IN1/IN2 per motor)
   ----------------------------------------------------------------------------
   v1.8/v1.9: PWM fully split onto its OWN timers (M1/M2/M3 = one timer each
   with 2 adjacent channels; M4 = two single-channel timers TIM10 + TIM11),
   independent of the encoder timers. Each PWM timer's ARR is free
   (8400 @ 20 kHz) and no longer constrains the encoder counter.
     M1 -> TIM2  CH3/CH4 = PB10/PB11
     M2 -> TIM5  CH3/CH4 = PA2/PA3
     M3 -> TIM9  CH1/CH2 = PE5/PE6
     M4 -> TIM10 CH1 = PB8  +  TIM11 CH1 = PB9   (PB14/15 -> TFT)
   Direction logic (DRV8871):
     IN1 = PWM, IN2 = 0   ->  forward
     IN1 = 0,   IN2 = PWM ->  reverse
     IN1 = 0,   IN2 = 0   ->  coast
     IN1 = 1,   IN2 = 1   ->  brake (low side)
   20 kHz, PSC = 0, ARR = PEO_PM_ARR_APBx, PWM1 mode, preload enabled.
   (TIM1/TIM8 no longer output PWM, so TIM_CtrlPWMOutputs/MOE is not needed.)
   ============================================================================ */

/** PWM1/2 = M1 - TIM2_CH3/CH4 (PB10/PB11, AF1) **/
#define PEO_PM_PWM1_TIM         TIM2
#define PEO_PM_PWM1_CH          TIM_Channel_3
#define PEO_PM_PWM1_PORT        GPIOB
#define PEO_PM_PWM1_PIN         GPIO_Pin_10
#define PEO_PM_PWM1_SRC         GPIO_PinSource10
#define PEO_PM_PWM2_TIM         TIM2
#define PEO_PM_PWM2_CH          TIM_Channel_4
#define PEO_PM_PWM2_PORT        GPIOB
#define PEO_PM_PWM2_PIN         GPIO_Pin_11
#define PEO_PM_PWM2_SRC         GPIO_PinSource11
#define PEO_PM_PWM1_2_AF        GPIO_AF_TIM2

/** PWM3/4 = M2 - TIM5_CH3/CH4 (PA2/PA3, AF2) **/
#define PEO_PM_PWM3_TIM         TIM5
#define PEO_PM_PWM3_CH          TIM_Channel_3
#define PEO_PM_PWM3_PORT        GPIOA
#define PEO_PM_PWM3_PIN         GPIO_Pin_2
#define PEO_PM_PWM3_SRC         GPIO_PinSource2
#define PEO_PM_PWM4_TIM         TIM5
#define PEO_PM_PWM4_CH          TIM_Channel_4
#define PEO_PM_PWM4_PORT        GPIOA
#define PEO_PM_PWM4_PIN         GPIO_Pin_3
#define PEO_PM_PWM4_SRC         GPIO_PinSource3
#define PEO_PM_PWM3_4_AF        GPIO_AF_TIM5

/** PWM5/6 = M3 - TIM9_CH1/CH2 (PE5/PE6, AF3) **/
#define PEO_PM_PWM5_TIM         TIM9
#define PEO_PM_PWM5_CH          TIM_Channel_1
#define PEO_PM_PWM5_PORT        GPIOE
#define PEO_PM_PWM5_PIN         GPIO_Pin_5
#define PEO_PM_PWM5_SRC         GPIO_PinSource5
#define PEO_PM_PWM6_TIM         TIM9
#define PEO_PM_PWM6_CH          TIM_Channel_2
#define PEO_PM_PWM6_PORT        GPIOE
#define PEO_PM_PWM6_PIN         GPIO_Pin_6
#define PEO_PM_PWM6_SRC         GPIO_PinSource6
#define PEO_PM_PWM5_6_AF        GPIO_AF_TIM9

/** PWM7/8 = M4 - TIM10_CH1(PB8) + TIM11_CH1(PB9), AF3 (v1.9: PB14/15 -> TFT) **/
#define PEO_PM_PWM7_TIM         TIM10
#define PEO_PM_PWM7_CH          TIM_Channel_1
#define PEO_PM_PWM7_PORT        GPIOB
#define PEO_PM_PWM7_PIN         GPIO_Pin_8
#define PEO_PM_PWM7_SRC         GPIO_PinSource8
#define PEO_PM_PWM8_TIM         TIM11
#define PEO_PM_PWM8_CH          TIM_Channel_1
#define PEO_PM_PWM8_PORT        GPIOB
#define PEO_PM_PWM8_PIN         GPIO_Pin_9
#define PEO_PM_PWM8_SRC         GPIO_PinSource9
#define PEO_PM_PWM7_8_AF        GPIO_AF_TIM10

/* ============================================================================
   7. DRV8871 MOTOR DRIVER MODULES (simplified, v1.8)
   ----------------------------------------------------------------------------
   The DRV8871 modules in use expose ONLY: IN1, IN2 (PWM inputs) and
   OUT1, OUT2 (to motor), plus VM/GND. They do NOT bring out nSLEEP or
   nFAULT - so this design has no sleep control and no per-motor fault pin.
   Consequence: stall/over-current detection relies on the whole-vehicle
   current monitor (ACS712, section 9) - it cannot localize which motor.
   ============================================================================ */

/* ============================================================================
   8. I2C BUS - I2C1 (PB6/PB7, AF4, 400 kHz) - MPU6050 DEDICATED (v2.3)
   ----------------------------------------------------------------------------
   DEDICATED to the MPU6050 alone (whole-vehicle vibration). The IMU may
   sample faster than the rangefinders/current monitor, so it gets a clean
   bus with no other traffic.
     MPU6050   0x68 (or 0x69 via AD0)
   NOTE: I2C2 (PB10/PB11) is unavailable (M1 PWM on TIM2).
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
   8b. I2C BUS - I2C3 (PA8/PC9, AF4, 400 kHz) - VL53L0X x3 + INA3221 (v2.3)
   ----------------------------------------------------------------------------
   Shared bus for the 3x VL53L0X rangefinders + the INA3221 current monitor:
     VL53L0X front    0x52
     VL53L0X left     0x54
     VL53L0X right    0x56
     INA3221          0x40-0x43  (path B current monitor, low sample rate)
   Re-addressing (AN4846): all three VL53L0X ship at factory default 0x29.
   XSHUT (section 10) is held LOW at power-up, then released one sensor at a
   time to assign the address above before the next sensor is released.
   NOTES:
     - I2C3 alternate pins PH7/PH8 are NOT broken out on LQFP100, so PA8/PC9
       are the only option. PA8 (also TIM1_CH1) and PC9 (also TIM8_CH4) are
       free because TIM1/TIM8 use their CH1/CH2 pins (PE9/11, PC6/7).
   ============================================================================ */

/** I2C3 - VL53L0X + INA3221 bus (PA8/PC9, AF4) **/
#define PEO_PM_VL53_I2C          I2C3
#define PEO_PM_VL53_I2C_CLKSPEED (400000U)
#define PEO_PM_VL53_SCL_PORT     GPIOA
#define PEO_PM_VL53_SCL_PIN      GPIO_Pin_8
#define PEO_PM_VL53_SCL_SRC      GPIO_PinSource8
#define PEO_PM_VL53_SDA_PORT     GPIOC
#define PEO_PM_VL53_SDA_PIN      GPIO_Pin_9
#define PEO_PM_VL53_SDA_SRC      GPIO_PinSource9
#define PEO_PM_VL53_I2C_AF       GPIO_AF_I2C3

/** I2C3 slave addresses (7-bit) **/
#define PEO_PM_VL53_FRONT_ADDR  (0x52U)          /* I2C3 */
#define PEO_PM_VL53_LEFT_ADDR   (0x54U)          /* I2C3 */
#define PEO_PM_VL53_RIGHT_ADDR  (0x56U)          /* I2C3 */
#define PEO_PM_INA3221_ADDR     (0x40U)          /* I2C3, path B (low rate) */

/* ============================================================================
   9. CURRENT SENSING - PATH A: ACS712 (ADC1_IN4, PA4, 12-bit)
   ----------------------------------------------------------------------------
   ACS712-05B: Vout = 2.5 V + I * 0.185 V/A.
   With a 3.3 V ADC reference the readable range is (3.3-2.5)/0.185
   = +4.3 A (and -4.3 A) before clipping - the 5 A full scale saturates
   the ADC first. 4 motors stalled simultaneously (~6 A) read as rail -
   for anomaly detection this saturation IS a usable stall flag.
   Front-end RC: 1 kOhm + 100 nF (cutoff ~1.6 kHz) before PA4.
   If path B (INA3221) is chosen instead, PA4 stays unused as spare ADC.
   ============================================================================ */

/** ACS712 current sense - path A (if path B is chosen, PA4 stays spare) **/
#define PEO_PM_ACS_ADC          ADC1
#define PEO_PM_ACS_CH           ADC_Channel_4
#define PEO_PM_ACS_PORT         GPIOA
#define PEO_PM_ACS_PIN          GPIO_Pin_4
#define PEO_PM_ACS_MV_PER_A     (185U)          /* ACS712-05B sensitivity */

/* ============================================================================
   10. VL53L0X XSHUT x3  (restored v2.0)
   ----------------------------------------------------------------------------
   GPIO outputs, active-low reset per sensor. Held LOW at power-up to keep all
   three at the factory default 0x29, then released one at a time (AN4846) so
   each can be re-addressed (see section 8) before the next is released.
     XSHUT front = PC0    XSHUT left = PC1    XSHUT right = PC2
   Push-pull output, no pull-up needed (VL53L0X has an internal XSHUT pull-up).
   ============================================================================ */

/** VL53L0X XSHUT (active-low; low = held in reset) **/
#define PEO_PM_VL53_FRONT_XSHUT_PORT   GPIOC
#define PEO_PM_VL53_FRONT_XSHUT_PIN    GPIO_Pin_0
#define PEO_PM_VL53_LEFT_XSHUT_PORT    GPIOC
#define PEO_PM_VL53_LEFT_XSHUT_PIN     GPIO_Pin_1
#define PEO_PM_VL53_RIGHT_XSHUT_PORT   GPIOC
#define PEO_PM_VL53_RIGHT_XSHUT_PIN    GPIO_Pin_2

/* ============================================================================
   11. TFT DISPLAY (board socket J4, 8x1 female, SPI2)  [restored v1.9]
   ----------------------------------------------------------------------------
   J4 pinout: 1=3V3  2=GND  3=SDI/PB15  4=SCL/PB13  5=CS/PB12
              6=SDO/PB14  7=D/C/PC5  8=BLK/PB1      (NO RST pin!)
   Driver: hardware SPI2 (AF5, 21 Mbps) or bit-banged SPI.
   Display compatibility: ST7789/ST7735 OK (RST flywire PE12); LCD12864 no.
   ============================================================================ */

/** TFT - board socket J4 **/
#define PEO_PM_TFT_SPI          SPI2
#define PEO_PM_TFT_AF           GPIO_AF_SPI2
#define PEO_PM_TFT_CS_PORT      GPIOB
#define PEO_PM_TFT_CS_PIN       GPIO_Pin_12     /* CS, plain GPIO */
#define PEO_PM_TFT_DC_PORT      GPIOC
#define PEO_PM_TFT_DC_PIN       GPIO_Pin_5      /* D/C, plain GPIO */
#define PEO_PM_TFT_BLK_PORT     GPIOB
#define PEO_PM_TFT_BLK_PIN      GPIO_Pin_1      /* backlight, plain GPIO */
#define PEO_PM_TFT_SCK_PORT     GPIOB
#define PEO_PM_TFT_SCK_PIN      GPIO_Pin_13     /* SPI2_SCK  (J4 pin 4) */
#define PEO_PM_TFT_SCK_SRC      GPIO_PinSource13
#define PEO_PM_TFT_MOSI_PORT    GPIOB
#define PEO_PM_TFT_MOSI_PIN     GPIO_Pin_15     /* SPI2_MOSI (J4 pin 3) */
#define PEO_PM_TFT_MOSI_SRC     GPIO_PinSource15
#define PEO_PM_TFT_MISO_PORT    GPIOB
#define PEO_PM_TFT_MISO_PIN     GPIO_Pin_14     /* SPI2_MISO (J4 pin 6) */
#define PEO_PM_TFT_MISO_SRC     GPIO_PinSource14

/** TFT RST - flywire to PE12 (J4 has no RST pin) **/
#define PEO_PM_TFT_RST_PORT     GPIOE
#define PEO_PM_TFT_RST_PIN      GPIO_Pin_12

/* ============================================================================
   12. BOARD-RESERVED PINS - NEVER INITIALIZE, NEVER TOUCH
   ----------------------------------------------------------------------------
   K1 button     PA0                     (active high)
   LED D2        PA1                     (sink)
   W25Q16BV      PA15 (CS)  PB3 (CLK)  PB4 (DO)  PB5 (DI)   (SPI FLASH)
   USB           PA11 (D-)  PA12 (D+)
   SWD           PA13 (SWDIO)  PA14 (SWCLK)   (ST-Link, keep!)
   SD socket     PC8-PC12, PD2 - SD function NOT used. v1.8: PC8/PC9 were
                 M3 PWM and are now FREE; PC10-12/PD2 were nFAULT and are now
                 FREE (nFAULT removed - simplified DRV8871 has no fault pin).
   Not broken out: PB2 (BOOT1), PC14, PC15, PH0, PH1
   ============================================================================ */

/* ============================================================================
   13. INTERRUPT HANDLER NAMES (StdPeriph, startup_stm32f40xx.s)
   ----------------------------------------------------------------------------
   ALL four encoders are pure (ARR = 0xFFFF), read as 16-bit DELTA - no
   encoder timer needs an Update interrupt. The PWM timers (TIM2/5/9/10/11)
   are edge-aligned with no interrupt either. So no timer IRQ handler is
   used in this design.
   ============================================================================ */

/* ============================================================================
   15. DMA STREAM / CHANNEL MAP  (RM0090 Table 43 - DMA1 request mapping)
   ----------------------------------------------------------------------------
   Hardware routing facts - they live HERE (not in a component header) so
   every component can see them through the PEO_PINMAP.H include chain.
   ----------------------------------------------------------------------------
   | Peripheral | Direction | Stream / Channel | IRQ channel (see nvic.h) |
   |------------|-----------|------------------|--------------------------|
   | USART2     | RX        | DMA1_Stream5 Ch4 | DMA1_Stream5_IRQn        |
   | USART2     | TX        | DMA1_Stream6 Ch4 | DMA1_Stream6_IRQn        |
   | SPI2 (TFT) | RX (opt.) | DMA1_Stream3 Ch0 | DMA1_Stream3_IRQn        |
   | SPI2 (TFT) | TX (opt.) | DMA1_Stream4 Ch0 | DMA1_Stream4_IRQn        |
   | ADC1 (opt.)| RX        | DMA2_Stream0 Ch0 | DMA2_Stream0_IRQn        |
   ----------------------------------------------------------------------------
   Recommended USART2 setup: RX CIRCULAR mode (fixed ring buffer), IDLE-line
   interrupt marks frame boundaries; TX normal mode on demand.
   ============================================================================ */

/** ESP32 link DMA (USART2) **/
#define PEO_DMA_COMM_RX_STREAM  DMA1_Stream5
#define PEO_DMA_COMM_RX_CH      DMA_Channel_4
#define PEO_DMA_COMM_TX_STREAM  DMA1_Stream6
#define PEO_DMA_COMM_TX_CH      DMA_Channel_4

/** TFT SPI2 DMA (optional; zero if bit-banged SPI) **/
#define PEO_DMA_TFT_RX_STREAM   DMA1_Stream3
#define PEO_DMA_TFT_RX_CH       DMA_Channel_0
#define PEO_DMA_TFT_TX_STREAM   DMA1_Stream4
#define PEO_DMA_TFT_TX_CH       DMA_Channel_0

/** ACS712 ADC1 DMA (optional; default is polling) **/
#define PEO_DMA_ADC_STREAM      DMA2_Stream0
#define PEO_DMA_ADC_CH          DMA_Channel_0

/* ============================================================================
   16. EXTI LINES
   ----------------------------------------------------------------------------
   v1.8: none - the simplified DRV8871 modules have no nFAULT pin, so no EXTI
   fault inputs exist. (Reserved for future use.)
   ============================================================================ */

/* ============================================================================
   17. POWER-UP AND USAGE CHECKLIST
   ----------------------------------------------------------------------------
   [ ] HSE_VALUE set to 8000000 (StdPeriph default is 25 MHz - wrong!)
   [ ] No SD card inserted (PC8-12/PD2 no longer used, but keep socket empty)
   [ ] Encoder mode: TIM_EncoderInterfaceConfig(TI12, Rising, Rising),
       then TIM_ICInit for CH1 and CH2 (IC1F filter e.g. 0x0F); ARR = 0xFFFF
   [ ] PWM timers (TIM2/5/9/10/11) ARR set for 20 kHz (8400/4200 per bus)
   [ ] XSHUT x3 (PC0/1/2) held LOW at power-up, released one at a time (AN4846)
       to re-address the three VL53L0X (0x52/0x54/0x56) before first ranging
   [ ] Battery output fused (2 A-class) - BMS is not a fuse
   [ ] Never sustain motor stall (stall ~1.5 A per motor, 18 W heat)
   ============================================================================ */

#endif /* __PEO_PINMAP_H__ */
