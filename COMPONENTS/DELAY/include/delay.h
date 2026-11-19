/**
 * @file  delay.h
 * @brief Declare the legacy software delay helpers used during hardware setup.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
#ifndef __DELAY_H__
#define __DELAY_H__

#include "stm32f4xx.h"

/**
 * @brief Wait approximately the requested number of milliseconds.
 *
 * Used during I2C and range-sensor setup. This legacy NOP-based delay differs from TIMER
 * mdelay, which measures DWT cycles.
 * @param d_ms Requested delay in milliseconds; zero returns immediately.
 * @retval None This function does not return a value.
 */
void delay_ms(uint16_t d_ms);
/**
 * @brief Execute a short legacy software delay loop for bus timing.
 *
 * Compatibility helper available to I2C code; one loop contains one NOP plus loop
 * overhead, so the parameter is not a calibrated microsecond duration.
 * @param us Loop iteration count; the historical name suggests microseconds, but actual
 *           time depends on clock and generated code.
 * @retval None This function does not return a value.
 */
void delay_i2c(uint16_t us);
/**
 * @brief Spend approximately one microsecond executing no-operation instructions.
 *
 * Used by the legacy delay helpers at the intended 168 MHz core clock. Call overhead and
 * interrupts affect timing; this is not a hardware-timed deadline.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void delay_1us(void);

#endif
