/**
 * @file  builtin_led.c
 * @brief Drive the onboard active-low status LED for startup and host commands.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * GPIOA clock is enabled centrally by RCC, not by this component.
 * D2 cathode connects to PA1; its anode connects to VCC through a current-limiting
 * resistor. Driving PA1 low allows current to flow and lights the LED.
 */
#include "builtin_led.h"

/**
 * @brief Prepare the onboard status LED and leave it off.
 *
 * Called by main after RCC enables GPIOA; later main lights D2 to show startup
 * completed.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void BUILTIN_LED_Init(void)
{
    GPIO_InitTypeDef led;  /* GPIO setup record for the active-low D2 output. */

    led.GPIO_Mode  = GPIO_Mode_OUT;      /* General-purpose push-pull output: actively drive the pin high or low. */
    led.GPIO_OType = GPIO_OType_PP;
    led.GPIO_PuPd  = GPIO_PuPd_NOPULL;   /* No pull-up or pull-down is needed while the output driver sets the level. */
    led.GPIO_Speed = GPIO_Speed_2MHz;    /* An LED does not need fast GPIO switching. */
    led.GPIO_Pin   = PEO_PM_LED_PIN;

    GPIO_Init(PEO_PM_LED_PORT, &led);
    BUILTIN_LED_Off();                   /* Keep the LED off at startup. */
}

/**
 * @brief Light the active-low D2 status LED.
 *
 * Called at the end of startup or by the host LED_ON command; PA1 low completes the
 * current path.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void BUILTIN_LED_On(void)
{
    if (PEO_PM_LED_ON_LEVEL == Bit_RESET)
        GPIO_ResetBits(PEO_PM_LED_PORT, PEO_PM_LED_PIN);   /* Sink current by driving low to light the LED. */
    else
        GPIO_SetBits(PEO_PM_LED_PORT, PEO_PM_LED_PIN);
}

/**
 * @brief Turn off the active-low D2 status LED.
 *
 * Called by the host LED_OFF command; PA1 high stops current through D2.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void BUILTIN_LED_Off(void)
{
    if (PEO_PM_LED_ON_LEVEL == Bit_RESET)
        GPIO_SetBits(PEO_PM_LED_PORT, PEO_PM_LED_PIN);     /* Drive high to stop sinking current and turn the LED off. */
    else
        GPIO_ResetBits(PEO_PM_LED_PORT, PEO_PM_LED_PIN);
}

/**
 * @brief Change an illuminated LED to off, or an unlit LED to on.
 *
 * Called by the host LED_TOGGLE command, using the current GPIO output state.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void BUILTIN_LED_Toggle(void)
{
    GPIO_ToggleBits(PEO_PM_LED_PORT, PEO_PM_LED_PIN);
}
