/**
 * @file  led_status.h
 * @brief Status LED driver that shows whether the Xbox controller is connected.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

#ifndef __LED_STATUS_H__
#define __LED_STATUS_H__

#include <stdbool.h>

/**
 * @brief  Initialise the LEDC PWM peripheral and start the status LED task.
 *
 * The LED is the on-board blue LED on GPIO8 (active low). The task runs a
 * breathing / flashing / solid state machine so the link state is visible at a
 * glance.
 */
void led_status_init(void);

/**
 * @brief  Publish the current connection state to the LED task.
 *
 * Called by the sender task every 10 ms. A rising edge (false -> true) makes the
 * LED flash briefly, then stay solid on while connected.
 *
 * @param  connected  true = controller connected, false = not connected.
 */
void led_status_notify_connected(bool connected);

#endif /* __LED_STATUS_H__ */
