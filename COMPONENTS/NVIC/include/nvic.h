/**
 ******************************************************************************
 * @file    nvic.h
 * @brief   PEO Rover - NVIC component public API (interrupt map for ALL modules)
 *
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
 ******************************************************************************
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
   | 3 (0/1/2)     | INA3221 poll + ESP32 RX + report (TIM14)   |
   | optional      | USART1 debug RX (commented in nvic.c)      |
   ============================================================================ */
#define PEO_NVIC_PRIO_GROUP         NVIC_PriorityGroup_2


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
   | INA3221 电流/电压轮询  | TIM7_IRQn               | TIM7_IRQHandler         | 3 (0)   |
   | ESP32 link RX          | USART2_IRQn             | USART2_IRQHandler       | 3 (1)   |
   | sensorhub report       | TIM8_TRG_COM_TIM14_IRQn | TIM8_TRG_COM_TIM14_IRQHandler | 3 (2) |
   | Debug RX (optional)    | USART1_IRQn             | USART1_IRQHandler       | -       |
   ----------------------------------------------------------------------------
   不使用中断：4 路编码器均采用 16 位差分读取；PWM 定时器
   (TIM5/9/10/11/12) 均为边沿对齐；简化版 DRV8871 无 nFAULT；I2C2 不使用中断。
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
#define PEO_NVIC_INA_IRQ              TIM7_IRQn       /* INA3221 电流/电压轮询 */
#define PEO_NVIC_ESP_IRQ              USART2_IRQn     /* ESP32 link RX          */
#define PEO_NVIC_ESP_TX_IRQ           TIM8_TRG_COM_TIM14_IRQn  /* sensorhub 上报 (TIM14) */

/* ============================================================================
   3. DMA STREAM / CHANNEL MAP  (RM0090 Table 43 - DMA1 request mapping)
   ----------------------------------------------------------------------------
   TFT 已移除，不再分配其 DMA 通道；PB14/PB15 用于 M3 PWM。
   ESP32 链路的 USART2 DMA 映射见 PEO_PINMAP.H 第 13 节。
   ============================================================================ */

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
 * @brief  Configure the NVIC priority group and enable EVERY interrupt
 *         channel this design uses (with the priorities from section 1).
 *         Call AFTER RCC_Clock_Init() and BEFORE module inits.
 *         Peripheral-side interrupt enables (TIM_ITConfig etc.) remain
 *         the responsibility of each module.
 */
void NVIC_InitAll(void);

/** Internal helper: enable one IRQ channel with a preemption priority. **/
static inline void PEO_NVIC_Enable(uint8_t irq_channel, uint8_t preempt_prio, uint8_t sub_prio)
{
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel                   = irq_channel;
    nvic.NVIC_IRQChannelPreemptionPriority = preempt_prio;
    nvic.NVIC_IRQChannelSubPriority        = sub_prio;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

#endif /* __NVIC_H__ */
