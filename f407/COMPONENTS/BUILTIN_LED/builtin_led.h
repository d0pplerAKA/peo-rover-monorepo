/**
 * @file  builtin_led.h
 * @brief Expose controls for the onboard RGB status LED.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
#ifndef __BUILTIN_LED_H__
#define __BUILTIN_LED_H__

#include "stm32f4xx.h"

#include "COMPONENTS/PEO_PINMAP.H"

/**
 * @brief Prepare the three RGB LED channels and leave them all off.
 *
 * Called by main after RCC enables the GPIO port; later main lights the LED to
 * show startup completed.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void BUILTIN_LED_Init(void);
/**
 * @brief Set the three RGB channels individually.
 *
 * Each argument is nonzero to light that channel or zero to leave it off; the
 * pin polarity is taken from PEO_PM_LED_ON_LEVEL.
 * @param red Red channel request (nonzero = on).
 * @param green Green channel request (nonzero = on).
 * @param blue Blue channel request (nonzero = on).
 * @retval None This function does not return a value.
 */
void BUILTIN_LED_SetRGB(uint8_t red, uint8_t green, uint8_t blue);
/**
 * @brief Light all three channels (white) as the legacy status indicator.
 *
 * Called at the end of startup or by the host LED_ON command.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void BUILTIN_LED_On(void);
/**
 * @brief Turn off all three LED channels.
 *
 * Called by the host LED_OFF command.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void BUILTIN_LED_Off(void);
/**
 * @brief Invert every LED channel (an illuminated one goes off and vice versa).
 *
 * Called by the host LED_TOGGLE command, using the current GPIO output state.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void BUILTIN_LED_Toggle(void);

#endif
