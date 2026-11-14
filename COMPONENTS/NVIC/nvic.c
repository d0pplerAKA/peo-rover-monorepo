/**
 ******************************************************************************
 * @file    nvic.c
 * @brief   PEO Rover - NVIC component: one-shot total interrupt initialization
 *
 * The ONLY place in the whole firmware that touches NVIC registers.
 * Modules never call NVIC_* functions - they rely on this function.
 ******************************************************************************
 */
#include "include/nvic.h"

/**
 * @brief  Set the priority group (Group 4) and enable every interrupt
 *         channel in the design map with its assigned priority.
 * @note   Must run AFTER RCC_Clock_Init() (clocks first) and BEFORE any
 *         module init that arms peripheral interrupt bits.
 */
void NVIC_InitAll(void)
{
    /* Priority group FIRST - must precede every NVIC_Init() */
    NVIC_PriorityGroupConfig(PEO_NVIC_PRIO_GROUP);

    /* VL53L1X data-ready INT (EXTI0/1/2) + MPU6050 data-ready INT (EXTI3) */
    PEO_NVIC_Enable(PEO_NVIC_VL53_FRONT_INT_IRQ,    2,  0);
    PEO_NVIC_Enable(PEO_NVIC_VL53_LEFT_INT_IRQ,     2,  1);
    PEO_NVIC_Enable(PEO_NVIC_VL53_RIGHT_INT_IRQ,    2,  2);
    PEO_NVIC_Enable(PEO_NVIC_MPU_INT_IRQ,           1,  0);
    PEO_NVIC_Enable(PEO_NVIC_CTRL_IRQ,              3,  0);   /* 1 kHz control loop (TIM6) */
    PEO_NVIC_Enable(PEO_NVIC_INA_IRQ,               3,  1);   /* INA3221 poll (TIM7) */
    PEO_NVIC_Enable(PEO_NVIC_ESP_IRQ,               3,  2);   /* ESP32 link RX (USART2) */
    PEO_NVIC_Enable(PEO_NVIC_ESP_TX_IRQ,            3,  3);   /* sensorhub report (TIM14) */

    /* Optional (uncomment when the driver lands):
       PEO_NVIC_Enable(PEO_NVIC_ADC_IRQ, PEO_NVIC_PRIO_ADC);   // ACS712 ADC
       PEO_NVIC_Enable(PEO_NVIC_DBG_IRQ, PEO_NVIC_PRIO_DBG);   // debug console RX
    */
}
