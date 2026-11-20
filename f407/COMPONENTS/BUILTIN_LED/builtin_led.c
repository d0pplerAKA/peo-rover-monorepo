/**
 * @file  builtin_led.c
 * @brief Drive the onboard active-low RGB status LED.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * GPIO clocks are enabled centrally by RCC, not by this component.
 * Three channels sit on PC0 (red), PC1 (green) and PC2 (blue); each LED
 * anode connects to VCC through a current-limiting resistor, so driving a
 * pin low lights its channel (PEO_PM_LED_ON_LEVEL = Bit_RESET).
 */
#include "builtin_led.h"

#define LED_CHANNEL_COUNT 3U  /* Red, green and blue. */

/* Pin masks per channel, ordered red/green/blue. */
static const uint16_t s_led_pins[LED_CHANNEL_COUNT] = {
    PEO_PM_LED_RED,
    PEO_PM_LED_GREEN,
    PEO_PM_LED_BLUE
};

#define LED_ALL_PINS (PEO_PM_LED_RED | PEO_PM_LED_GREEN | PEO_PM_LED_BLUE)  /* Combined mask of the three RGB channel pins. */

/**
 * @brief Write one RGB channel's electrical state.
 *
 * Internal helper; translates a logical on/off request into the pin level
 * selected by PEO_PM_LED_ON_LEVEL.
 * @param channel Zero-based channel index 0..2 (red/green/blue).
 * @param on Nonzero to light the channel, zero to turn it off.
 * @retval None This function does not return a value.
 */
static void led_channel_write(uint8_t channel, uint8_t on)
{
    if (channel >= LED_CHANNEL_COUNT)
        return;

    if (on) {
        GPIO_WriteBit(PEO_PM_LED_PORT, s_led_pins[channel], PEO_PM_LED_ON_LEVEL);
    } else {
        GPIO_WriteBit(PEO_PM_LED_PORT, s_led_pins[channel],
                      (PEO_PM_LED_ON_LEVEL == Bit_RESET) ? Bit_SET : Bit_RESET);
    }
}

/**
 * @brief Prepare the three RGB LED channels and leave them all off.
 *
 * Called by main after RCC enables the GPIO port; later main lights the LED to
 * show startup completed.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void BUILTIN_LED_Init(void)
{
    GPIO_InitTypeDef led;  /* GPIO setup record for the RGB channel outputs. */

    /* Latch the off level before enabling the output driver so no channel
       flashes during initialization. */
    led_channel_write(0, 0);
    led_channel_write(1, 0);
    led_channel_write(2, 0);

    led.GPIO_Mode  = GPIO_Mode_OUT;      /* General-purpose push-pull output: actively drive each pin high or low. */
    led.GPIO_OType = GPIO_OType_PP;
    led.GPIO_PuPd  = GPIO_PuPd_NOPULL;   /* No pull-up or pull-down is needed while the output driver sets the level. */
    led.GPIO_Speed = GPIO_Speed_2MHz;    /* LEDs do not need fast GPIO switching. */
    led.GPIO_Pin   = LED_ALL_PINS;

    GPIO_Init(PEO_PM_LED_PORT, &led);
}

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
void BUILTIN_LED_SetRGB(uint8_t red, uint8_t green, uint8_t blue)
{
    led_channel_write(0, red);
    led_channel_write(1, green);
    led_channel_write(2, blue);
}

/**
 * @brief Light all three channels (white) as the legacy status indicator.
 *
 * Called at the end of startup or by the host LED_ON command.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void BUILTIN_LED_On(void)
{
    BUILTIN_LED_SetRGB(1, 1, 1);
}

/**
 * @brief Turn off all three LED channels.
 *
 * Called by the host LED_OFF command.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void BUILTIN_LED_Off(void)
{
    BUILTIN_LED_SetRGB(0, 0, 0);
}

/**
 * @brief Invert every LED channel (an illuminated one goes off and vice versa).
 *
 * Called by the host LED_TOGGLE command, using the current GPIO output state.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void BUILTIN_LED_Toggle(void)
{
    GPIO_ToggleBits(PEO_PM_LED_PORT, LED_ALL_PINS);
}
