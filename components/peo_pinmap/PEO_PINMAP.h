/**
 * @file  PEO_PINMAP.h
 * @brief ESP32-C3 SuperMini pin assignment — single source of truth for all pin macros.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

#ifndef __PEO_PINMAP_H__
#define __PEO_PINMAP_H__

/**
 * If a pin changes (new board, rewiring), edit it HERE only — every component
 * reads its pins from this header, so there is no second place to update.
 *
 * C3 -> F407 link (direct to F407 USART3, bypassing the S3; TX/RX crossed, common ground):
 *   C3 UART1 GPIO4(TX) / GPIO5(RX)  <->  F407 USART3 PD8(TX) / PD9(RX)
 * Note: C3 GPIO20/21 is UART0, which the on-board USB-to-serial chip already
 * uses, so it must NOT be used as a link pin.
 */

#include "driver/gpio.h"
#include "driver/uart.h"

/* ============================================================================
 * ESP32-C3 -> F407 USART3 link (used by the c3_uart component).
 * Baud rate matches the F407 USART3 (115200).
 * ============================================================================ */
#define PEO_PM_C3_UART      UART_NUM_1   /**< UART peripheral number used for the link. */
#define PEO_PM_C3_TX        GPIO_NUM_4   /**< C3 TX pin (connects to F407 RX). */
#define PEO_PM_C3_RX        GPIO_NUM_5   /**< C3 RX pin (connects from F407 TX). */
#define PEO_PM_C3_BAUDRATE  (115200U)    /**< Link baud rate, in bits per second. */

/* On-board blue status LED: GPIO8, active LOW (lights when the pin is low; the
 * LED is wired from 3V3 through a current-limiting resistor to the pin).
 * GPIO8 is a strapping pin, but the LED wiring keeps it high at power-on (LED
 * off), and the LEDC driver has no issue with it after boot. */
#define PEO_PM_LED_GPIO     GPIO_NUM_8   /**< GPIO that drives the on-board status LED. */

#endif /* __PEO_PINMAP_H__ */
