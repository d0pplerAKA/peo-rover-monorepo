/**
 * @file  nvic.c
 * @brief Configure all application interrupt channels and their priorities centrally.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * Central owner of application IRQ-channel enables and priority grouping.
 * Modules rely on this setup; TIMER separately configures the core SysTick
 * priority through SysTick_Config(), and host reset may use NVIC_SystemReset().
 */
#include "include/nvic.h"

/**
 * @brief Set interrupt priorities and enable all application interrupt channels.
 *
 * Called second in main, after RCC and before peripheral setup. Smaller preemption
 * numbers have higher priority; each component separately enables its own peripheral
 * interrupt source.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note Must run AFTER RCC_Clock_Init() (clocks first) and BEFORE any module init that
 *       arms peripheral interrupt bits.
 */
void NVIC_InitAll(void)
{
    /* Priority group FIRST - must precede every NVIC_Init() */
    NVIC_PriorityGroupConfig(PEO_NVIC_PRIO_GROUP);

    /* 1 kHz control loop (TIM6) - HIGHEST: it must not be preempted by the
       sensor I2C reads (MPU6050 DMP FIFO / VL53L1X result), otherwise the
       loop sample period jitters and the fixed-1 ms speed scaling breaks. */
    PEO_NVIC_Enable(PEO_NVIC_CTRL_IRQ,              1,  0);

    /* Sensor data-ready INTs: MPU6050 (EXTI3) + VL53L1X (EXTI0/1/2).
       They do blocking I2C reads, so they sit BELOW the control loop. */
    PEO_NVIC_Enable(PEO_NVIC_MPU_INT_IRQ,           2,  0);
    PEO_NVIC_Enable(PEO_NVIC_VL53_FRONT_INT_IRQ,    2,  1);
    PEO_NVIC_Enable(PEO_NVIC_VL53_LEFT_INT_IRQ,     2,  2);
    PEO_NVIC_Enable(PEO_NVIC_VL53_RIGHT_INT_IRQ,    2,  3);

    /* Lowest: INA3221 poll (TIM7), ESP32 link RX (USART2), report (TIM14). */
    PEO_NVIC_Enable(PEO_NVIC_INA_IRQ,               3,  0);
    PEO_NVIC_Enable(PEO_NVIC_ESP_IRQ,               3,  1);
    PEO_NVIC_Enable(PEO_NVIC_ESP_TX_IRQ,            3,  2);
    PEO_NVIC_Enable(PEO_NVIC_GAMEPAD_IRQ,            3,  3);   /* gamepad RX (USART3) */

    /* Optional (uncomment when the driver lands):
       PEO_NVIC_Enable(PEO_NVIC_DBG_IRQ, PEO_NVIC_PRIO_DBG);   // debug console RX
    */
}
