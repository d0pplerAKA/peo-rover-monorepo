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
   Group 4: all 4 priority bits are PREEMPTION priority (0..15), no sub-
   priority. Rationale: odometry integrity (encoder 32-bit accumulation)
   outranks everything; debug output is the most expendable.
   ----------------------------------------------------------------------------
   | Priority | Function                                          |
   |----------|---------------------------------------------------|
   | 0x02     | ESP32 comms (USART2 + DMA1 Stream5/6)             |
   | 0x03     | I2C1 event/error (MPU6050 bus)                    |
   | 0x04     | ADC1 (ACS712 current sampling, if interrupt-driven)|
   | 0x07     | USART1 debug console (RX interrupt, optional)     |
   ============================================================================ */
#define PEO_NVIC_PRIO_GROUP    NVIC_PriorityGroup_4
#define PEO_NVIC_PRIO_COMM     (0x02U)
#define PEO_NVIC_PRIO_I2C      (0x03U)
#define PEO_NVIC_PRIO_ADC      (0x04U)
#define PEO_NVIC_PRIO_DBG      (0x07U)

/* ============================================================================
   2. IRQ CHANNEL MAP
   ----------------------------------------------------------------------------
   | Function               | IRQ channel macro       | Handler (startup .s)    | Prio |
   |------------------------|-------------------------|-------------------------|------|
   | ESP32 RX done          | DMA1_Stream5_IRQn       | DMA1_Stream5_IRQHandler | 0x02 |
   | ESP32 TX done          | DMA1_Stream6_IRQn       | DMA1_Stream6_IRQHandler | 0x02 |
   | ESP32 frame boundary   | USART2_IRQn (IDLE flag) | USART2_IRQHandler       | 0x02 |
   | MPU6050 event          | I2C1_EV_IRQn            | I2C1_EV_IRQHandler      | 0x03 |
   | MPU6050 error          | I2C1_ER_IRQn            | I2C1_ER_IRQHandler      | 0x03 |
   | VL53L0X/INA3221 event  | I2C3_EV_IRQn            | I2C3_EV_IRQHandler      | 0x03 |
   | VL53L0X/INA3221 error  | I2C3_ER_IRQn            | I2C3_ER_IRQHandler      | 0x03 |
   | ADC1 (optional)        | ADC_IRQn                | ADC_IRQHandler          | 0x04 |
   | Debug RX (optional)    | USART1_IRQn             | USART1_IRQHandler       | 0x07 |
   ----------------------------------------------------------------------------
   NO interrupt: all 4 encoders are pure (16-bit delta), all PWM timers
   (TIM2/5/9/10/11) edge-aligned; no nFAULT (simplified DRV8871); no I2C2.
   Optional entries (ADC, debug RX) are commented out in nvic.c until their
   drivers land - uncomment when the corresponding component is added.
   ============================================================================ */

/** IRQ channel macros **/
#define PEO_NVIC_COMM_RX_IRQ    DMA1_Stream5_IRQn     /* USART2 RX */
#define PEO_NVIC_COMM_TX_IRQ    DMA1_Stream6_IRQn     /* USART2 TX */
#define PEO_NVIC_COMM_IRQ       USART2_IRQn           /* IDLE-line frame detect */

#define PEO_NVIC_I2C1_EV_IRQ    I2C1_EV_IRQn
#define PEO_NVIC_I2C1_ER_IRQ    I2C1_ER_IRQn
#define PEO_NVIC_I2C3_EV_IRQ    I2C3_EV_IRQn
#define PEO_NVIC_I2C3_ER_IRQ    I2C3_ER_IRQn

#define PEO_NVIC_ADC_IRQ        ADC_IRQn              /* optional */
#define PEO_NVIC_DBG_IRQ        USART1_IRQn           /* optional */

/* ============================================================================
   3. DMA STREAM / CHANNEL MAP  (RM0090 Table 43 - DMA1 request mapping)
   ----------------------------------------------------------------------------
   | Peripheral | Direction | Stream / Channel | IRQ channel          |
   |------------|-----------|------------------|----------------------|
   | USART2     | RX        | DMA1_Stream5 Ch4 | DMA1_Stream5_IRQn    |
   | USART2     | TX        | DMA1_Stream6 Ch4 | DMA1_Stream6_IRQn    |
   | SPI2 (TFT) | RX (opt.) | DMA1_Stream3 Ch0 | DMA1_Stream3_IRQn    |
   | SPI2 (TFT) | TX (opt.) | DMA1_Stream4 Ch0 | DMA1_Stream4_IRQn    |
   | ADC1 (opt.)| RX        | DMA2_Stream0 Ch0 | DMA2_Stream0_IRQn    |
   ============================================================================ */

/* DMA stream/channel macros (PEO_DMA_*) live in PEO_PINMAP.H section 15 -
   hardware routing facts must be visible to every component. */

/* ============================================================================
   4. EXTI MAP
   ----------------------------------------------------------------------------
   v1.8: none - no nFAULT (simplified DRV8871). Reserved for future use.
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
static inline void PEO_NVIC_Enable(uint8_t irq_channel, uint8_t preempt_prio)
{
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel                   = irq_channel;
    nvic.NVIC_IRQChannelPreemptionPriority = preempt_prio;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

#endif /* __NVIC_H__ */
