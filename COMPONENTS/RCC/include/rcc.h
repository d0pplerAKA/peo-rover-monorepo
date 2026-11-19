/**
 * @file  rcc.h
 * @brief Define the shared peripheral clock masks and startup clock interface.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * Design rule (per architecture decision 2026-11-10):
 *   - Every clock enable lives HERE, inside COMPONENTS/RCC.
 *   - Modules (USART, motors, I2C, ...) do NOT touch RCC registers - they
 *     assume RCC_Clock_Init() already ran in main().
 *
 * main() call order:
 *   RCC_Clock_Init();   <- this component, FIRST
 *   NVIC_InitAll();     <- COMPONENTS/NVIC
 *   ... module inits ...
 *
 * SystemInit() (HSE/PLL, 8 MHz -> 168 MHz) is called by
 * startup_stm32f40xx.s before main(); see PEO_PINMAP.H section 1.
 */
#ifndef __RCC_H__
#define __RCC_H__

#include "stm32f4xx.h"

/* ============================================================================
   CLOCK-ENABLE MAP (F407 clock domains, RM0090 RCC chapter)
   ----------------------------------------------------------------------------
   | Peripheral        | Bus  | StdPeriph macro            | Used by           |
   |-------------------|------|----------------------------|-------------------|
   | GPIOA-E           | AHB1 | RCC_AHB1Periph_GPIOx      | All assigned GPIO   |
   | TIM3, TIM4        | APB1 | RCC_APB1Periph_TIMx       | Encoders M2/M3      |
   | TIM5              | APB1 | RCC_APB1Periph_TIM5       | PWM M1            |
   | TIM12             | APB1 | RCC_APB1Periph_TIM12      | PWM M3            |
   | TIM6              | APB1 | RCC_APB1Periph_TIM6       | 1 kHz control loop|
   | TIM7              | APB1 | RCC_APB1Periph_TIM7       | INA3221 polling      |
   | I2C1              | APB1 | RCC_APB1Periph_I2C1       | MPU6050 (dedicated)|
   | I2C3              | APB1 | RCC_APB1Periph_I2C3       | 3x VL53L1X         |
   | I2C2              | APB1 | RCC_APB1Periph_I2C2       | INA3221 (dedicated)|
   | TIM1, TIM8        | APB2 | RCC_APB2Periph_TIMx       | Encoders M1/M4      |
   | TIM9              | APB2 | RCC_APB2Periph_TIM9       | PWM M2            |
   | TIM10, TIM11      | APB2 | RCC_APB2Periph_TIMx       | PWM M4            |
   | USART1            | APB2 | RCC_APB2Periph_USART1     | debug console     |
   | USART2            | APB1 | RCC_APB1Periph_USART2     | ESP32 link (PD5/6) |
   | USART3            | APB1 | RCC_APB1Periph_USART3     | Gamepad (PD8/9)  |
   | SYSCFG            | APB2 | RCC_APB2Periph_SYSCFG     | sensor EXTI routing |
   ----------------------------------------------------------------------------
   NOT enabled (and why):
     AHB2 (USB OTG FS): USB not used (PA11/PA12 idle).
     AHB3 (FSMC):      not used.
     APB1 SPI3/CAN/DAC and APB2 SDIO: unused.
     APB2 SPI1: pins are reserved for external Flash, but its clock is not enabled.
   ============================================================================ */

/** Aggregate: every AHB1 peripheral this design needs **/
/* Combined AHB1 clock-enable mask for assigned GPIO ports and DMA1. */
#define PEO_RCC_AHB1_EN    (RCC_AHB1Periph_GPIOA | \
                            RCC_AHB1Periph_GPIOB | \
                            RCC_AHB1Periph_GPIOC | \
                            RCC_AHB1Periph_GPIOD | \
                            RCC_AHB1Periph_GPIOE | \
                            RCC_AHB1Periph_DMA1)

/** Aggregate: every APB1 peripheral this design needs **/
/* Combined APB1 clock-enable mask for low-speed-bus timers, I2C, and serial links. */
#define PEO_RCC_APB1_EN    (RCC_APB1Periph_TIM3   | \
                            RCC_APB1Periph_TIM4   | \
                            RCC_APB1Periph_TIM5   | \
                            RCC_APB1Periph_TIM6   | \
                            RCC_APB1Periph_TIM7   | \
                            RCC_APB1Periph_TIM12  | \
                            RCC_APB1Periph_TIM14  | \
                            RCC_APB1Periph_I2C1   | \
                            RCC_APB1Periph_I2C2   | \
                            RCC_APB1Periph_I2C3   | \
                            RCC_APB1Periph_USART2 | \
                            RCC_APB1Periph_USART3)

/** Aggregate: every APB2 peripheral this design needs **/
/* Combined APB2 clock-enable mask for high-speed-bus timers, debug USART, and EXTI routing. */
#define PEO_RCC_APB2_EN    (RCC_APB2Periph_TIM1   | \
                            RCC_APB2Periph_TIM8   | \
                            RCC_APB2Periph_TIM9   | \
                            RCC_APB2Periph_TIM10  | \
                            RCC_APB2Periph_TIM11  | \
                            RCC_APB2Periph_USART1 | \
                            RCC_APB2Periph_SYSCFG)

/* ============================================================================
   PUBLIC API
   ============================================================================ */

/**
 * @brief Supply peripheral clocks before any component accesses its hardware.
 *
 * Called first in main after startup SystemInit establishes the main clock tree. Enables
 * the central AHB1/APB1/APB2 clock masks; components do not repeat clock setup.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void RCC_Clock_Init(void);

#endif /* __RCC_H__ */
