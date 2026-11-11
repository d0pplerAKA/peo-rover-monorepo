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

    /* ESP32 link: DMA RX/TX + USART2 IDLE framing */
    PEO_NVIC_Enable(PEO_NVIC_COMM_RX_IRQ, PEO_NVIC_PRIO_COMM);      /* DMA1_Stream5 */
    PEO_NVIC_Enable(PEO_NVIC_COMM_TX_IRQ, PEO_NVIC_PRIO_COMM);      /* DMA1_Stream6 */
    PEO_NVIC_Enable(PEO_NVIC_COMM_IRQ,    PEO_NVIC_PRIO_COMM);      /* USART2       */

    /* MPU6050 IMU bus (I2C1) */
    PEO_NVIC_Enable(PEO_NVIC_I2C1_EV_IRQ, PEO_NVIC_PRIO_I2C);
    PEO_NVIC_Enable(PEO_NVIC_I2C1_ER_IRQ, PEO_NVIC_PRIO_I2C);

    /* VL53L0X + INA3221 bus (I2C3) */
    PEO_NVIC_Enable(PEO_NVIC_I2C3_EV_IRQ, PEO_NVIC_PRIO_I2C);
    PEO_NVIC_Enable(PEO_NVIC_I2C3_ER_IRQ, PEO_NVIC_PRIO_I2C);

    /* Optional (uncomment when the driver lands):
       PEO_NVIC_Enable(PEO_NVIC_ADC_IRQ, PEO_NVIC_PRIO_ADC);   // ACS712 ADC
       PEO_NVIC_Enable(PEO_NVIC_DBG_IRQ, PEO_NVIC_PRIO_DBG);   // debug console RX
    */
}
