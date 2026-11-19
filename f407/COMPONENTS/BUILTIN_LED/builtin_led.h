/**
 * @file  builtin_led.h
 * @brief Expose simple controls for the onboard rover status LED.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
#ifndef __BUILTIN_LED_H__
#define __BUILTIN_LED_H__

#include "stm32f4xx.h"

#include "COMPONENTS/PEO_PINMAP.H"

/**
 * @brief Prepare the onboard status LED and leave it off.
 *
 * Called by main after RCC enables GPIOA; later main lights D2 to show startup
 * completed.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void BUILTIN_LED_Init(void);
/**
 * @brief Light the active-low D2 status LED.
 *
 * Called at the end of startup or by the host LED_ON command; PA1 low completes the
 * current path.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void BUILTIN_LED_On(void);
/**
 * @brief Turn off the active-low D2 status LED.
 *
 * Called by the host LED_OFF command; PA1 high stops current through D2.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void BUILTIN_LED_Off(void);
/**
 * @brief Change an illuminated LED to off, or an unlit LED to on.
 *
 * Called by the host LED_TOGGLE command, using the current GPIO output state.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void BUILTIN_LED_Toggle(void);

#endif

