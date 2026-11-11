/**
 ******************************************************************************
 * @file    rcc.c
 * @brief   PEO Rover - RCC component: one-shot total clock initialization
 *
 * The ONLY place in the whole firmware that touches RCC enable registers.
 * Modules never enable their own clocks - they rely on this function.
 ******************************************************************************
 */
#include "include/rcc.h"

/**
 * @brief  Enable all peripheral clocks (AHB1 + APB1 + APB2) and configure
 *         the ADC clock prescaler (PCLK2 / 6 = 14 MHz).
 * @note   HSE/PLL system clock setup is NOT here: startup_stm32f40xx.s
 *         calls SystemInit() before main() (see PEO_PINMAP.H section 1).
 */
void RCC_Clock_Init(void)
{
    RCC_AHB1PeriphClockCmd(PEO_RCC_AHB1_EN, ENABLE);   /* GPIOA-E, DMA1   */
    RCC_APB1PeriphClockCmd(PEO_RCC_APB1_EN, ENABLE);   /* TIM2/3/4/5, USART2, I2C1/3, SPI2 */
    RCC_APB2PeriphClockCmd(PEO_RCC_APB2_EN, ENABLE);   /* TIM1/8/9/10/11, USART1, ADC1, SYSCFG */

    /* ADC clock: PCLK2 / 6 = 14 MHz.
       On F4 the divider is ADCPRE[1:0] in ADC_CCR (RM0090 13.13.16), set
       via ADC_CommonInit() with ADC_Prescaler_Div6 - NOT via RCC (the
       F1-style RCC_ADCCLKConfig does not exist in this library).
       Datasheet: fADC <= 36 MHz, so /2 (42 MHz) would be out of spec. */
    {
        ADC_CommonInitTypeDef adc_common;
        adc_common.ADC_Mode             = ADC_Mode_Independent;
        adc_common.ADC_Prescaler        = PEO_RCC_ADC_PRESCALER;
        adc_common.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
        adc_common.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
        ADC_CommonInit(&adc_common);
    }
}
