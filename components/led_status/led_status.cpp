/**
 * @file  led_status.cpp
 * @brief Status LED implementation: PWM + breathing/flashing/solid state machine.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

/**
 * The LED shows, at a glance, whether the Xbox controller is connected:
 *   - not connected        -> slow 1 s "breathing" fade
 *   - link just came up    -> 5 Hz flash for 1 s
 *   - connected            -> solid on
 *
 * The LED is the on-board blue LED on GPIO8 and is ACTIVE LOW (driving the pin
 * low turns the LED on), so the PWM duty cycle is inverted relative to the
 * brightness we want to show.
 */

#include "led_status.h"
#include "PEO_PINMAP.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <math.h>
#include <atomic>
#include <string.h>
#include "driver/ledc.h"

/** Log tag shown in the serial console for messages from this file. */
static const char *TAG = "led_status";

/** Maximum PWM duty for a 13-bit LEDC timer (2^13 - 1 = 8191). */
static constexpr uint32_t LED_MAX_DUTY = (1U << 13) - 1;
/** One full breathing-cycle duration, in milliseconds. */
static constexpr uint32_t LED_BREATH_MS = 1000;
/** Total flash duration after a rising connect edge, in milliseconds. */
static constexpr uint32_t LED_FLASH_MS = 1000;
/** On/off half-period of the 5 Hz flash, in milliseconds. */
static constexpr uint32_t LED_FLASH_ON_MS = 100;

/** Latest connection state, shared with the LED task (atomic = safe across tasks). */
static std::atomic<bool> s_connected{false};

/**
 * @brief  LED task: samples the connection state and drives the LED brightness.
 *
 * Runs forever at ~50 Hz (20 ms delay). It watches the shared s_connected flag:
 * a false->true edge starts a 1 s flash, after which the LED stays solid while
 * connected. Any time the flag is false the LED returns to breathing.
 *
 * @param  arg  Unused (FreeRTOS task argument).
 */
static void led_status_task(void *arg)
{
    // LED visual states: slow breathing, fast flash, or steady on.
    enum class State { BREATHING, FLASHING, SOLID };
    State state = State::BREATHING;
    bool prev_connected = false;   // previous sample, used to detect the rising edge
    uint32_t state_start = 0;      // millis() timestamp when the current state began

    while (1) {
        const uint32_t now = millis();
        const bool connected = s_connected.load(std::memory_order_relaxed);

        // Rising edge (just connected) -> start flashing.
        if (connected && !prev_connected) {
            state = State::FLASHING;
            state_start = now;
        }
        // Not connected (at any time) -> back to breathing.
        if (!connected)
            state = State::BREATHING;
        prev_connected = connected;

        uint32_t duty = 0;   // PWM duty this cycle (active-low: 0 = LED fully on)
        switch (state) {
        case State::BREATHING: {
            // Sine-wave fade over 1 s, inverted for active-low: as the phase
            // rises, brightness rises and the duty falls toward 0.
            const float phase = (float)(now % LED_BREATH_MS) / LED_BREATH_MS * 6.283185307179586f;
            const float brightness = (sinf(phase) + 1.0f) / 2.0f;
            duty = (uint32_t)((1.0f - brightness) * LED_MAX_DUTY);
            break;
        }
        case State::FLASHING: {
            const uint32_t elapsed = (uint32_t)(now - state_start);
            if (elapsed < LED_FLASH_MS) {
                // 5 Hz square wave: 100 ms on, 100 ms off (active-low duty).
                duty = ((elapsed / LED_FLASH_ON_MS) % 2 == 0) ? 0 : LED_MAX_DUTY;
            } else {
                // Flash finished -> steady on.
                state = State::SOLID;
                duty = 0;
            }
            break;
        }
        case State::SOLID:
            duty = 0;   // active-low: duty 0 = LED fully on
            break;
        }

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief  Set up the LEDC timer/channel and start the LED task.
 *
 * The status LED is a convenience feature: if either LEDC config step fails we
 * only log a warning and return, so a broken LED can never take down the
 * mission-critical heartbeat tasks.
 */
void led_status_init(void)
{
    ledc_timer_config_t t;
    memset(&t, 0, sizeof t);
    t.speed_mode = LEDC_LOW_SPEED_MODE;
    t.duty_resolution = LEDC_TIMER_13_BIT;   // 13-bit resolution, 0..8191 duty range
    t.timer_num = LEDC_TIMER_0;
    t.freq_hz = 5000;                        // PWM frequency (high enough to avoid visible flicker)
    t.clk_cfg = LEDC_AUTO_CLK;
    // Status LED is an auxiliary feature: on init failure just log and disable,
    // do not abort, so the core heartbeat tasks are never dragged down.
    if (ledc_timer_config(&t) != ESP_OK) {
        ESP_LOGW(TAG, "ledc timer config failed; status LED disabled");
        return;
    }

    ledc_channel_config_t c;
    memset(&c, 0, sizeof c);
    c.gpio_num = PEO_PM_LED_GPIO;
    c.speed_mode = LEDC_LOW_SPEED_MODE;
    c.channel = LEDC_CHANNEL_0;
    c.timer_sel = LEDC_TIMER_0;
    c.duty = LED_MAX_DUTY;   /* active-low: start with the LED OFF; if the task never runs the LED stays off (so it is not mistaken for "connected") */
    c.hpoint = 0;
    if (ledc_channel_config(&c) != ESP_OK) {
        ESP_LOGW(TAG, "ledc channel config failed; status LED disabled");
        return;
    }

    if (xTaskCreate(led_status_task, "led_status", 2048, NULL, 1, NULL) != pdPASS)
        ESP_LOGE(TAG, "led status task allocation failed");
}

/**
 * @brief  Publish the current connection state to the LED task.
 * @param  connected  true = controller connected, false = not connected.
 */
void led_status_notify_connected(bool connected)
{
    s_connected.store(connected, std::memory_order_relaxed);
}
