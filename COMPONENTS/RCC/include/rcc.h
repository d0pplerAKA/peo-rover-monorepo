/**
 ******************************************************************************
 * @file    rcc.h
 * @brief   PEO Rover - RCC component public API (clock domain for ALL modules)
 *
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
 ******************************************************************************
 */
#ifndef __RCC_H__
#define __RCC_H__

#include "stm32f4xx.h"

/* ============================================================================
   CLOCK-ENABLE MAP (F407 clock domains, RM0090 RCC chapter)
   ----------------------------------------------------------------------------
   | Peripheral        | Bus  | StdPeriph macro            | Used by           |
   |-------------------|------|----------------------------|-------------------|
   | GPIOA-F           | AHB1 | RCC_AHB1Periph_GPIOx      | all pins          |
   | TIM2              | APB1 | RCC_APB1Periph_TIM2       | PWM M1            |
   | TIM3, TIM4        | APB1 | RCC_APB1Periph_TIMx       | encoders M2/M4    |
   | TIM5              | APB1 | RCC_APB1Periph_TIM5       | PWM M2            |
   | TIM6              | APB1 | RCC_APB1Periph_TIM6       | 1 kHz control loop|
   | TIM7              | APB1 | RCC_APB1Periph_TIM7       | INA3221 轮询      |
   | I2C1              | APB1 | RCC_APB1Periph_I2C1       | MPU6050 (dedicated)|
   | I2C3              | APB1 | RCC_APB1Periph_I2C3       | 3x VL53L1X         |
   | I2C2              | APB1 | RCC_APB1Periph_I2C2       | INA3221 (dedicated)|
   | SPI2              | APB1 | RCC_APB1Periph_SPI2       | TFT (J4)          |
   | TIM1, TIM8        | APB2 | RCC_APB2Periph_TIMx       | encoders M1/M3    |
   | TIM9              | APB2 | RCC_APB2Periph_TIM9       | PWM M3            |
   | TIM10, TIM11      | APB2 | RCC_APB2Periph_TIMx       | PWM M4            |
   | USART1            | APB2 | RCC_APB2Periph_USART1     | debug console     |
   | USART2            | APB1 | RCC_APB1Periph_USART2     | ESP32 link (PD5/6) |
   | ADC1              | APB2 | RCC_APB2Periph_ADC1       | ACS712 (path A)   |
   | SYSCFG            | APB2 | RCC_APB2Periph_SYSCFG     | (reserved)        |
   ----------------------------------------------------------------------------
   NOT enabled (and why):
     AHB2 (USB OTG FS): USB not used (PA11/PA12 idle).
     AHB3 (FSMC):      not used.
     APB1 SPI3/I2C3/CAN/DAC, APB2 SPI1/SDIO: not used.
   ============================================================================ */

/** Aggregate: every AHB1 peripheral this design needs **/
#define PEO_RCC_AHB1_EN    (RCC_AHB1Periph_GPIOA | \
                            RCC_AHB1Periph_GPIOB | \
                            RCC_AHB1Periph_GPIOC | \
                            RCC_AHB1Periph_GPIOD | \
                            RCC_AHB1Periph_GPIOE | \
                            RCC_AHB1Periph_GPIOF | \
                            RCC_AHB1Periph_DMA1)

/** Aggregate: every APB1 peripheral this design needs **/
#define PEO_RCC_APB1_EN    (RCC_APB1Periph_TIM2   | \
                            RCC_APB1Periph_TIM3   | \
                            RCC_APB1Periph_TIM4   | \
                            RCC_APB1Periph_TIM5   | \
                            RCC_APB1Periph_TIM6   | \
                            RCC_APB1Periph_TIM7   | \
                            RCC_APB1Periph_TIM14  | \
                            RCC_APB1Periph_I2C1   | \
                            RCC_APB1Periph_I2C2   | \
                            RCC_APB1Periph_I2C3   | \
                            RCC_APB1Periph_SPI2   | \
                            RCC_APB1Periph_USART2)

/** Aggregate: every APB2 peripheral this design needs **/
#define PEO_RCC_APB2_EN    (RCC_APB2Periph_TIM1   | \
                            RCC_APB2Periph_TIM8   | \
                            RCC_APB2Periph_TIM9   | \
                            RCC_APB2Periph_TIM10  | \
                            RCC_APB2Periph_TIM11  | \
                            RCC_APB2Periph_USART1 | \
                            RCC_APB2Periph_ADC1   | \
                            RCC_APB2Periph_SYSCFG)

/* ----------------------------------------------------------------------------
   ADC CLOCK PRESCALER (F4-family fact, RM0090 section 13.13.16)
   ----------------------------------------------------------------------------
   On STM32F4 the ADC clock divider is NOT an RCC field. It is the
   ADCPRE[1:0] field (bits 17:16) of the ADC common control register
   ADC_CCR, programmed through ADC_CommonInit() with the standard library
   keyword ADC_Prescaler_Div6 (stm32f4xx_adc.h).
     ADCPRE: 00 = PCLK2/2, 01 = PCLK2/4, 10 = PCLK2/6, 11 = PCLK2/8
     PCLK2 = 84 MHz  ->  /6 = 14 MHz  (datasheet limit: fADC <= 36 MHz)
   The F1-family keyword RCC_ADCCLKConfig(RCC_PCLK2_Div6) does NOT exist
   in this FWLIB (stm32f4xx_rcc.h has no ADCCLKConfig at all).
   -------------------------------------------------------------------------- */
#define PEO_RCC_ADC_PRESCALER   ADC_Prescaler_Div6

/* ============================================================================
   PUBLIC API
   ============================================================================ */

/**
 * @brief  Enable every peripheral clock this design needs, and set the ADC
 *         clock prescaler. Call FIRST in main() (before NVIC_InitAll()).
 */
void RCC_Clock_Init(void);

#endif /* __RCC_H__ */
