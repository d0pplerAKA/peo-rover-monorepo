/**
 * @file  delay.c
 * @brief Provide legacy busy-wait delays for sensor and I2C initialization.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
#include "include/delay.h"


/**
 * @brief Spend approximately one microsecond executing no-operation instructions.
 *
 * Used by the legacy delay helpers at the intended 168 MHz core clock. Call overhead and
 * interrupts affect timing; this is not a hardware-timed deadline.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void delay_1us(void)        // 168Mhz
{
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();

    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();
}

/**
 * @brief Build an approximate millisecond delay from 1000 short waits.
 *
 * Internal helper used by delay_ms; it keeps the CPU busy instead of sleeping.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void delay_1ms(void)
{
    for(int i = 0; i < 1000; i++)
        delay_1us();
}

/**
 * @brief Wait approximately the requested number of milliseconds.
 *
 * Used during I2C and range-sensor setup. This legacy NOP-based delay differs from TIMER
 * mdelay, which measures DWT cycles.
 * @param d_ms Requested delay in milliseconds; zero returns immediately.
 * @retval None This function does not return a value.
 */
void delay_ms(uint16_t d_ms)
{
    for(int i = 0; i < d_ms; i++)
        delay_1ms();
}

/**
 * @brief Execute a short legacy software delay loop for bus timing.
 *
 * Compatibility helper available to I2C code; one loop contains one NOP plus loop
 * overhead, so the parameter is not a calibrated microsecond duration.
 * @param us Loop iteration count; the historical name suggests microseconds, but actual
 *           time depends on clock and generated code.
 * @retval None This function does not return a value.
 */
void delay_i2c(uint16_t us)
{
    while(us > 0)
    {
        __nop();
        us--;
    }
}
