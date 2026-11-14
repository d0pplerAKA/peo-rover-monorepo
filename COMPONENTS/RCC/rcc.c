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
 * @brief  Enable all peripheral clocks (AHB1 + APB1 + APB2).
 * @note   HSE/PLL system clock setup is NOT here: startup_stm32f40xx.s
 *         calls SystemInit() before main() (see PEO_PINMAP.H section 1).
 */
void RCC_Clock_Init(void)
{
    RCC_AHB1PeriphClockCmd(PEO_RCC_AHB1_EN, ENABLE);   /* GPIOA-E         */
    RCC_APB1PeriphClockCmd(PEO_RCC_APB1_EN, ENABLE);   /* TIM2/3/4/5, I2C1/3, SPI2 */
    RCC_APB2PeriphClockCmd(PEO_RCC_APB2_EN, ENABLE);   /* TIM1/8/9/10/11, USART1, SYSCFG */
}
