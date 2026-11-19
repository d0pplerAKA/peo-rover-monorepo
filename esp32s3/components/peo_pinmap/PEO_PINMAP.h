#ifndef __PEO_PINMAP_H__
#define __PEO_PINMAP_H__

/**
 * @file  PEO_PINMAP.h
 * @brief ESP32-S3 pin assignment — the single source of truth for every pin macro.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

/*
 * Target board: ESP32-S3 N8R8 (8 MB Quad Flash + 8 MB Octal PSRAM).
 * GPIO35/36/37 are reserved for the Octal PSRAM bus — never assign peripherals
 * to them.
 *
 * This file lives in components/peo_pinmap/ so that every component
 * (esp_link, camera_service, ws2812_led, ...) can include the same pin table.
 * Change a pin here once and every component picks it up.
 */

#include "driver/gpio.h"

/* ============================================================================
   OV2640 camera (used by camera_service)
   ============================================================================ */
#define PEO_PM_CAM_SCL      GPIO_NUM_1
#define PEO_PM_CAM_SDA      GPIO_NUM_2
#define PEO_PM_CAM_D0       GPIO_NUM_39
#define PEO_PM_CAM_D2       GPIO_NUM_38
#define PEO_PM_CAM_D4       GPIO_NUM_47
#define PEO_PM_CAM_D6       GPIO_NUM_21
#define PEO_PM_CAM_PCLK     GPIO_NUM_13
#define PEO_PM_CAM_PWDN     GPIO_NUM_14
#define PEO_PM_CAM_VS       GPIO_NUM_4
#define PEO_PM_CAM_HS       GPIO_NUM_5
#define PEO_PM_CAM_RST      GPIO_NUM_6
#define PEO_PM_CAM_D1       GPIO_NUM_7
#define PEO_PM_CAM_D3       GPIO_NUM_15
#define PEO_PM_CAM_D5       GPIO_NUM_16
#define PEO_PM_CAM_D7       GPIO_NUM_9
/* The camera module has its own XCLK oscillator, so we do NOT drive a clock
 * pin: GPIO_NUM_NC means "no pin used". NOTE: xclk_freq_hz in camera_service
 * must still match the module crystal (OV2640 uses it for its PLL), 20 MHz. */
#define PEO_PM_CAM_XCLK     GPIO_NUM_NC

/* ============================================================================
   WS2812 status LED (used by ws2812_led, RMT TX direct drive)
   ----------------------------------------------------------------------------
   A single WS2812, data pin on GPIO48. Different dev boards may use a different
   pin: to move it, change ONLY this macro (ws2812_led reads it from here).
   ============================================================================ */
#define PEO_PM_WS2812_GPIO   GPIO_NUM_48

/* ============================================================================
   ESP32-S3 <-> F407 UART link (used by esp_link)
   ----------------------------------------------------------------------------
   Wired to F407 USART2: PD5 = F407 TX -> ESP32 RX, PD6 = F407 RX <- ESP32 TX.
   Baud rate matches F407 PEO_PM_ESP_BAUDRATE (921600).
   ============================================================================ */
/* Confirmed by user 2026-11-14: GPIO11/12 (any pin via the GPIO matrix).
   TX = 11 -> F407 USART2_RX (PD6), RX = 12 <- F407 USART2_TX (PD5). */
#define PEO_PM_ESP_UART        UART_NUM_1
#define PEO_PM_ESP_TX          GPIO_NUM_11
#define PEO_PM_ESP_RX          GPIO_NUM_12
#define PEO_PM_ESP_BAUDRATE    (921600U)

#endif /* __PEO_PINMAP_H__ */
