#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file  ws2812_led.h
 * @brief Single WS2812 status LED (RMT TX direct drive) + manual color control.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

/*
 * Status meanings (decided with user on 2026-11-16):
 *   - Not connected to the laptop hotspot (PEO_NET_EVT_STA clear): blue
 *     breathing (sine brightness, 1.6 s period);
 *   - Hotspot connected: green flashes 3 times, then steady green;
 *   - Disconnected: back to breathing.
 *
 * Manual control: other components call ws2812_set_color() through a FreeRTOS
 * queue (1-deep, latest-wins), which immediately overrides the steady color
 * while connected. As soon as the connection state changes (disconnect or
 * reconnect) the status-LED behavior takes over again.
 *
 * Never touch RMT/GPIO directly. The LED is maintained by its own task on core 1.
 */

typedef struct {
    uint8_t r, g, b;      /* Color channels, each 0-255. */
    uint8_t brightness;   /* Global brightness 0-100 (percent). */
} ws2812_cmd_t;

/** @brief  Create the command queue. Must be called before ws2812_task (app_main does it once). */
void ws2812_queues_init(void);

/** @brief  LED maintenance task: block for a command -> scale brightness -> RMT send. Runs on core 1. */
void ws2812_task(void *arg);

/** @brief  The only entry point other components may use (1-deep latest-wins queue, never blocks the caller).
 *  @retval false if the component is not initialized. */
bool ws2812_set_color(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);

#ifdef __cplusplus
}
#endif
