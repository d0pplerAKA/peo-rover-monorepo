/**
 ******************************************************************************
 * @file    main.c
 * @brief   PEO Rover - STM32F407VGT6 firmware entry
 *
 * Init order (architecture rule, see PEO_PINMAP.H):
 *   1. SystemInit()  - called by startup_stm32f40xx.s BEFORE main():
 *                      HSE 8 MHz -> PLL -> SYSCLK 168 MHz (HSE_VALUE must
 *                      be 8000000 in stm32f4xx_conf.h).
 *   2. RCC_Clock_Init()  - ALL peripheral clocks in one shot
 *                          (COMPONENTS/RCC, includes ADC prescaler).
 *   3. NVIC_InitAll()    - priority group + ALL IRQ channels in one shot
 *                          (COMPONENTS/IRQ).
 *   4. Component inits   - each module only configures its own peripheral
 *                          (GPIO/timers/USART/DMA/EXTI...), never RCC/NVIC.
 ******************************************************************************
 */
#include "stm32f4xx.h"
#include "PEO_PINMAP.H"

#include "COMPONENTS/RCC/include/rcc.h"
#include "COMPONENTS/NVIC/include/nvic.h"
#include "COMPONENTS/USART/include/usart.h"

#include <stdio.h>

int main(void)
{
    /* 1-2. Total clock and interrupt initialization (components) */
    RCC_Clock_Init();
    NVIC_InitAll();

    /* 3. Communication components */
    USART_DBG_Init(PEO_PM_DBG_BAUDRATE);     /* 115200, PA9/PA10 USB-TTL */
    USART_COMM_Init();                       /* 4.2 Mbps ESP32 link      */

    /* 4. Boot banner (printf goes through fputc -> USART1) */
    printf("PEO Rover boot\r\n");
    printf("SYSCLK = %lu Hz | gear %u:1 | %u counts/output-rev | %u um/count\r\n",
           PEO_PM_SYSCLK_HZ,
           PEO_PM_GEAR_RATIO,
           PEO_PM_CNT_PER_OUT_REV,
           PEO_PM_UM_PER_CNT);

    while (1)
    {
        /* TODO: 1 kHz control loop
           - read 4 encoders (TIM1/3/8/4 CNT + update accumulation)
           - run motor PWM (TIM1/9/8/4, DRV8871 direction logic)
           - current sensing path A/B, MPU6050, 3x VL53L0X
           - ESP32 frame handling: USART_COMM_FramePending() -> parse
        */
    }
}
