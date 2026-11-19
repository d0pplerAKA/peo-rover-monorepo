/**
 * @file  ws2812_led.cpp
 * @brief Single WS2812 status LED task: RMT TX direct drive + manual color control.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

#include "ws2812_led.h"
#include "PEO_PINMAP.h"
#include "esp_net.h"

#include <driver/rmt_tx.h>
#include <esp_err.h>
#include <esp_log.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>

static const char *TAG = "ws2812";

/* Status LED colors (change these to taste):
 *   breathing = blue  (not connected to the hotspot),
 *   steady    = green (connected),
 *   flash     = green, flashed 3 times on the connect edge. */
#define WS2812_BREATH_R   0
#define WS2812_BREATH_G   100
#define WS2812_BREATH_B   255
#define WS2812_STEADY_R   0
#define WS2812_STEADY_G   255
#define WS2812_STEADY_B   0
#define WS2812_TICK_MS    50      /* State-machine tick: 50 ms between LED updates. */
#define WS2812_FLASH_MS   150     /* Connect flash: 150 ms on, 150 ms off, per blink. */

/* 1-deep latest-wins queue: the LED only cares about the final color, so a new
 * command overwrites an older one that has not been consumed yet. */
static QueueHandle_t s_cmd_queue = NULL;
static rmt_channel_handle_t s_tx_chan = NULL;
static rmt_encoder_handle_t s_encoder = NULL;

/** @brief  Create the command queue (called once by app_main, before the task). */
void ws2812_queues_init(void)
{
    if (!s_cmd_queue)
        s_cmd_queue = xQueueCreate(1, sizeof(ws2812_cmd_t));
    if (!s_cmd_queue)
        ESP_LOGE(TAG, "cmd queue create failed");
}

/** @brief  Scale a color channel by a brightness percentage (0-100 -> 0-255).
 *  @param v          Color channel value, 0-255.
 *  @param brightness Brightness percent, 0-100.
 *  @retval Scaled channel value, 0-255. */
static inline uint8_t scale(uint8_t v, uint8_t brightness)
{
    return (uint8_t)(((uint16_t)v * (uint16_t)brightness) / 100u);
}

/** @brief  Send one GRB frame to the LED (GRB order + brightness scaling).
 *  @param r, g, b    Raw color channels, 0-255.
 *  @param brightness Brightness percent, 0-100.
 *  @retval none
 *  The low-level gap between frames (the >50 us RESET pause) is guaranteed by the
 *  caller's tick cadence, not by this function. */
static void led_show(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
    uint8_t grb[3] = {
        scale(g, brightness),
        scale(r, brightness),
        scale(b, brightness),
    };
    rmt_transmit_config_t tx = {0};   /* loop_count = 0: send exactly once. */
    if (rmt_transmit(s_tx_chan, s_encoder, grb, sizeof(grb), &tx) == ESP_OK)
        rmt_tx_wait_all_done(s_tx_chan, pdMS_TO_TICKS(10));
}

/** @brief  Set up the RMT TX channel and byte encoder for WS2812 timing.
 *  @retval true on success; false if any RMT resource could not be created.
 *
 *  RMT runs at 10 MHz resolution (1 tick = 100 ns). WS2812 standard timing
 *  (GRB order, 800 kHz): T0H 0.4 us / T0L 0.9 us, T1H 0.8 us / T1L 0.5 us —
 *  all inside the +-150 ns tolerance of the chip. */
static bool rmt_init(void)
{
    rmt_tx_channel_config_t ch = {
        .gpio_num = PEO_PM_WS2812_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,      /* 80 MHz APB clock. */
        .resolution_hz = 10000000,
        .mem_block_symbols = 64,             /* 1 WS2812 = 24 bits = 24 RMT symbols. */
        .trans_queue_depth = 1,
    };
    rmt_bytes_encoder_config_t enc = {
        .bit0 = { .duration0 = 4, .level0 = 1, .duration1 = 9, .level1 = 0 },
        .bit1 = { .duration0 = 8, .level0 = 1, .duration1 = 5, .level1 = 0 },
        .flags = { .msb_first = 1 },          /* WS2812 sends G7..G0, R7..R0, B7..B0. */
    };
    if (rmt_new_tx_channel(&ch, &s_tx_chan) != ESP_OK)
        return false;
    if (rmt_new_bytes_encoder(&enc, &s_encoder) != ESP_OK)
    {
        rmt_del_channel(s_tx_chan);
        s_tx_chan = NULL;
        return false;
    }
    if (rmt_enable(s_tx_chan) != ESP_OK)
        return false;
    return true;
}

/** @brief  LED maintenance task: wait for commands, drive the breathing/steady/flash states.
 *  @param arg  Unused.
 *  @retval none (never returns; it deletes itself only on a fatal init failure). */
void ws2812_task(void *arg)
{
    (void)arg;
    if (!s_cmd_queue)
    {
        ESP_LOGE(TAG, "queue missing; task exits");
        vTaskDelete(NULL);
        return;
    }
    if (!rmt_init())
    {
        ESP_LOGE(TAG, "RMT init failed; task exits");
        vTaskDelete(NULL);
        return;
    }
    EventGroupHandle_t eg = esp_net_conn_event();
    if (!eg)
    {
        ESP_LOGE(TAG, "connection event group missing; task exits");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "task up (RMT GPIO%u, status LED)", (unsigned)PEO_PM_WS2812_GPIO);

    /* Breathing sine brightness table: 32 steps x 50 ms = 1.6 s per cycle. */
    static uint8_t breath_lut[32];
    for (int i = 0; i < 32; i++)
        breath_lut[i] = (uint8_t)(((sinf((float)i * 2.0f * (float)M_PI / 32.0f) + 1.0f) / 2.0f) * 100.0f);

    bool sta = false;              /* Previous STA (hotspot-connected) state. */
    uint8_t breath_i = 0;          /* Index into the breathing table. */
    ws2812_cmd_t manual = {0};     /* Last manual color requested. */
    bool has_manual = false;       /* true = a manual color overrides the status color. */

    for (;;)
    {
        /* 50 ms tick: wake immediately if the STA bit changes, otherwise at timeout. */
        xEventGroupWaitBits(eg, PEO_NET_EVT_STA, pdFALSE, pdFALSE,
                            pdMS_TO_TICKS(WS2812_TICK_MS));
        bool sta_now = (xEventGroupGetBits(eg) & PEO_NET_EVT_STA) != 0;

        if (sta_now && !sta)
        {
            /* Connect edge: green flash 3 times, then steady green. */
            ESP_LOGI(TAG, "hotspot connected; flash then steady");
            for (int k = 0; k < 3; k++)
            {
                led_show(WS2812_STEADY_R, WS2812_STEADY_G, WS2812_STEADY_B, 100);
                vTaskDelay(pdMS_TO_TICKS(WS2812_FLASH_MS));
                led_show(0, 0, 0, 100);
                vTaskDelay(pdMS_TO_TICKS(WS2812_FLASH_MS));
            }
            has_manual = false;
        }
        else if (!sta_now && sta)
        {
            ESP_LOGI(TAG, "hotspot lost; back to breathing");
            has_manual = false;   /* Disconnect -> breathing: give up the manual color. */
        }
        sta = sta_now;

        /* Manual command (sent by the upper computer once connected): latest-wins
         * override, kept until the connection state changes. */
        ws2812_cmd_t cmd;
        if (xQueueReceive(s_cmd_queue, &cmd, 0) == pdTRUE)
        {
            manual = cmd;
            has_manual = true;
        }

        if (sta_now)
        {
            if (has_manual)
                led_show(manual.r, manual.g, manual.b, manual.brightness);
            else
                led_show(WS2812_STEADY_R, WS2812_STEADY_G, WS2812_STEADY_B, 100);
        }
        else
        {
            led_show(WS2812_BREATH_R, WS2812_BREATH_G, WS2812_BREATH_B, breath_lut[breath_i]);
            breath_i = (uint8_t)((breath_i + 1) & 31);
        }
    }
}

/** @brief  The only entry point other components may use.
 *  @param r, g, b    Color channels, 0-255.
 *  @param brightness Brightness percent, 0-100 (clamped to 100).
 *  @retval false if the component is not initialized; true means the command was queued.
 *  Uses a 1-deep queue + overwrite: when full, the newest value replaces the
 *  pending one, so the caller never blocks. */
bool ws2812_set_color(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
    if (!s_cmd_queue)
        return false;
    ws2812_cmd_t cmd = {
        .r = r, .g = g, .b = b,
        .brightness = (uint8_t)(brightness > 100 ? 100 : brightness),
    };
    return xQueueOverwrite(s_cmd_queue, &cmd) == pdTRUE;
}
