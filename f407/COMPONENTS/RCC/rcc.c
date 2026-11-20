/**
 * @file  rcc.c
 * @brief Enable the peripheral clocks needed by all rover components.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * The ONLY place in the whole firmware that touches RCC enable registers.
 * Modules never enable their own clocks - they rely on this function.
 */
#include "include/rcc.h"

/**
 * @brief Supply peripheral clocks before any component accesses its hardware.
 *
 * Called first in main after startup SystemInit establishes the main clock tree. Enables
 * the central AHB1/APB1/APB2 clock masks; components do not repeat clock setup.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note HSE/PLL system clock setup is NOT here: startup_stm32f40xx.s calls SystemInit()
 *       before main() (see PEO_PINMAP.H section 1).
 */
void RCC_Clock_Init(void)
{
    RCC_AHB1PeriphClockCmd(PEO_RCC_AHB1_EN, ENABLE);   /* Enable GPIOA through GPIOE and DMA1 clocks. */
    RCC_APB1PeriphClockCmd(PEO_RCC_APB1_EN, ENABLE);   /* Enable TIM3/4/5/6/7/14, I2C1/2/3, USART2/3, and SPI3 clocks. */
    RCC_APB2PeriphClockCmd(PEO_RCC_APB2_EN, ENABLE);   /* Enable TIM1/8/9/10/11, USART1, and SYSCFG clocks. */
}
