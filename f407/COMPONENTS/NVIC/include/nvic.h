/**
 * @file  nvic.h
 * @brief Document interrupt ownership and expose the central NVIC setup interface.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * Design rule (per architecture decision 2026-11-10):
 *   - Every NVIC channel enable + the priority group live HERE, inside
 *     COMPONENTS/NVIC.
 *   - Modules (USART, motors, I2C, ...) do NOT call NVIC_* functions -
 *     they assume NVIC_InitAll() already ran in main(). They still own
 *     their peripheral-side interrupt enables (TIM_ITConfig, USART_ITConfig,
 *     DMA_ITConfig, EXTI_Init...), which decide whether an interrupt can
 *     actually fire.
 *
 * main() call order:
 *   RCC_Clock_Init();   <- COMPONENTS/RCC
 *   NVIC_InitAll();     <- this component, SECOND
 *   ... module inits ...
 *
 * NAMING NOTE: the total init is NVIC_InitAll(), NOT NVIC_Init() - the
 *   StdPeriph library already defines NVIC_Init(NVIC_InitTypeDef*) in
 *   misc.c and the two would collide at link time.
 */
#ifndef __NVIC_H__
#define __NVIC_H__

#include "stm32f4xx.h"


/* ============================================================================
   1. NVIC PRIORITY SCHEME
   ----------------------------------------------------------------------------
   Group 2: 2 bits PREEMPTION priority (0..3) + 2 bits sub-priority (0..3).
   The 1 kHz control loop (TIM6) is the HIGHEST priority: it must not be
   preempted by the sensor I2C reads, otherwise the fixed-1 ms speed scaling
   breaks. Sensor data-ready INTs sit one level below it.
   ----------------------------------------------------------------------------
   | Preempt (sub) | Function                                    |
   |---------------|---------------------------------------------|
   | 1 (0)         | 1 kHz control loop (TIM6)                  |
   | 2 (0)         | MPU6050 data-ready (EXTI3)                 |
   | 2 (1/2/3)     | VL53L1X data-ready x3 (EXTI0/1/2)          |
   | 3 (0/1/2/3)   | INA3221 poll + ESP32 RX + report (TIM14) + gamepad RX (USART3) |
   | optional      | USART1 debug RX (commented in nvic.c)      |
   ============================================================================ */
#define PEO_NVIC_PRIO_GROUP         NVIC_PriorityGroup_2  /* Split priority bits into two preemption bits and two subpriority bits. */


/* ============================================================================
   2. IRQ CHANNEL MAP
   ----------------------------------------------------------------------------
   | Function               | IRQ channel macro       | Handler (startup .s)    | Preempt |
   |------------------------|-------------------------|-------------------------|---------|
   | 1 kHz control loop     | TIM6_DAC_IRQn           | TIM6_DAC_IRQHandler     | 1 (0)   |
   | MPU6050 data-ready     | EXTI3_IRQn              | EXTI3_IRQHandler        | 2 (0)   |
   | VL53L1X front ready    | EXTI0_IRQn              | EXTI0_IRQHandler        | 2 (1)   |
   | VL53L1X left ready     | EXTI1_IRQn              | EXTI1_IRQHandler        | 2 (2)   |
   | VL53L1X right ready    | EXTI2_IRQn              | EXTI2_IRQHandler        | 2 (3)   |
   | INA3221 current/voltage  | TIM7_IRQn               | TIM7_IRQHandler         | 3 (0)   |
   | ESP32 link RX          | USART2_IRQn             | USART2_IRQHandler       | 3 (1)   |
   | sensorhub report       | TIM8_TRG_COM_TIM14_IRQn | TIM8_TRG_COM_TIM14_IRQHandler | 3 (2) |
   | gamepad RX (USART3)    | USART3_IRQn             | USART3_IRQHandler             | 3 (3) |
   | Debug RX (optional)    | USART1_IRQn             | USART1_IRQHandler       | -       |
   ----------------------------------------------------------------------------
   No interrupts are used for the four encoders (16-bit difference reads),
   edge-aligned PWM timers TIM5/9/10/11/12, or polling I2C2.
   The simplified DRV8871 modules have no nFAULT signal.
   Optional entries (debug RX) are commented out in nvic.c until their
   drivers land - uncomment when the corresponding component is added.
   ============================================================================ */

/** IRQ channel macros **/
#define PEO_NVIC_DBG_IRQ        USART1_IRQn           /* optional */

#define PEO_NVIC_VL53_FRONT_INT_IRQ   EXTI0_IRQn      /* VL53L1X data-ready */
#define PEO_NVIC_VL53_LEFT_INT_IRQ    EXTI1_IRQn      /* VL53L1X data-ready */
#define PEO_NVIC_VL53_RIGHT_INT_IRQ   EXTI2_IRQn      /* VL53L1X data-ready */
#define PEO_NVIC_MPU_INT_IRQ          EXTI3_IRQn      /* MPU6050 data-ready  */
#define PEO_NVIC_CTRL_IRQ             TIM6_DAC_IRQn   /* 1 kHz control loop  */
#define PEO_NVIC_INA_IRQ              TIM7_IRQn       /* INA3221 current/voltage polling. */
#define PEO_NVIC_ESP_IRQ              USART2_IRQn     /* ESP32 link RX          */
#define PEO_NVIC_ESP_TX_IRQ           TIM8_TRG_COM_TIM14_IRQn  /* Sensor-hub reporting on TIM14. */
#define PEO_NVIC_GAMEPAD_IRQ          USART3_IRQn     /* Gamepad reception on USART3. */

/* 3. DMA stream/channel map (RM0090 Table 43, DMA1 request mapping).
   The TFT was removed, so its DMA channel is no longer assigned; PB14/PB15
   now drive M3 PWM. See PEO_PINMAP.H section 13 for USART2 DMA assignments. */

/* DMA stream/channel macros (PEO_DMA_*) live in PEO_PINMAP.H section 13 -
   hardware routing facts must be visible to every component. */

/* ============================================================================
   4. EXTI MAP
   ----------------------------------------------------------------------------
   v2.6: INT x4 -> VL53L1X front/left/right = EXTI0/1/2 (PE0/1/2); MPU6050 =
   EXTI3 (PE3). All on independent EXTI lines (EXTI0-3 own IRQ vectors).
   ============================================================================ */

/* ============================================================================
   5. PUBLIC API
   ============================================================================ */

/**
 * @brief Set interrupt priorities and enable all application interrupt channels.
 *
 * Called second in main, after RCC and before peripheral setup. Smaller preemption
 * numbers have higher priority; each component separately enables its own peripheral
 * interrupt source.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void NVIC_InitAll(void);

/**
 * @brief Enable one interrupt channel at the selected urgency.
 *
 * Internal convenience helper used by NVIC_InitAll after choosing priority Group 2.
 * @param irq_channel IRQ channel identifier from the STM32 IRQn map.
 * @param preempt_prio Preemption priority 0..3; smaller numbers can interrupt larger
 *                     numbers.
 * @param sub_prio Subpriority 0..3, ordering pending interrupts at the same preemption
 *                 level; does not enable preemption.
 * @retval None This function does not return a value.
 */
static inline void PEO_NVIC_Enable(uint8_t irq_channel, uint8_t preempt_prio, uint8_t sub_prio)
{
    NVIC_InitTypeDef nvic;  /* Interrupt-controller setup for one IRQ channel and its two priority fields. */
    nvic.NVIC_IRQChannel                   = irq_channel;
    nvic.NVIC_IRQChannelPreemptionPriority = preempt_prio;
    nvic.NVIC_IRQChannelSubPriority        = sub_prio;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

#endif /* __NVIC_H__ */
