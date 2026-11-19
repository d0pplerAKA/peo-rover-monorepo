/**
 * @file  main.cpp
 * @brief ESP32-S3 firmware entry point: creates all queues and starts every task.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

/*
 * This file only *assembles* the firmware. Each component (esp_link, esp_net,
 * telemetry, cmd_parser, camera, ws2812) owns its own hardware and its own
 * queue; this file creates the shared queues up front and then starts every
 * task. The ESP32-S3 is a pure "bridge/router" for this competition: it does
 * not make driving decisions itself, it only forwards commands and streams
 * sensor data / video.
 *
 * Which CPU core runs what (core affinity is a scheduling policy, not a
 * correctness guarantee):
 *   core 0 (radio + protocol):  esp_rx / esp_tx / esp_net
 *   core 1 (application logic): telemetry / cmd_parser / camera / ws2812
 *       (Arduino's loopTask is disabled, so core 1 is free for our tasks.)
 *
 * Queue rule: every queue is created by its component's xxx_queues_init()
 * BEFORE any task is started. Task priority alone cannot guarantee which core
 * starts first, so a core-1 consumer might outrun a core-0 creator otherwise.
 */

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_psram.h>
#include "nvs_flash.h"

#include "esp_link.h"
#include "esp_net.h"
#include "telemetry.h"
#include "cmd_parser.h"
#include "camera_service.h"
#include "ws2812_led.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "peo_boot";

/**
 * @brief  Create one FreeRTOS task and pin it to a CPU core, with failure handling.
 *
 * @param entry     The task function to run.
 * @param name      Human-readable task name (also its FreeRTOS name, shown in dumps).
 * @param stack     Stack size in bytes.
 * @param priority  FreeRTOS priority (higher number = higher priority).
 * @param core      CPU core to pin the task to (0 or 1).
 * @param required  true = the rover cannot work without this task; false = it can be degraded.
 *
 * @retval none
 *
 * A *required* task that cannot be allocated is fatal: we latch a safety fault,
 * permanently disable recovery, and park forever. A half-alive system that can
 * still drive is more dangerous than one that stops. An *optional* task that
 * fails only logs a warning and the rest of the system keeps booting.
 */
static void start_task(TaskFunction_t entry, const char *name, uint32_t stack,
                       UBaseType_t priority, BaseType_t core, bool required)
{
    if (xTaskCreatePinnedToCore(entry, name, stack, NULL, priority, NULL, core) != pdPASS) {
        if (required) {
            ESP_LOGE(TAG, "Required task %s allocation failed; fatal fault, resume permanently disabled (internal heap=%u)",
                     name, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
            esp_link_fatal_fault();
            /* A required task is missing: disable recovery permanently, keep the
             * current state, and never reboot-loop the vehicle. */
            while (true) vTaskDelay(pdMS_TO_TICKS(1000));
        }
        ESP_LOGW(TAG, "Optional task %s allocation failed; degraded operation, continuing startup (internal heap=%u)",
                 name, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        return;  /* Optional task failed: only degrade, keep creating the remaining tasks. */
    }
}

/**
 * @brief  Firmware entry point: initialize NVS/Arduino, create queues, start tasks.
 *
 * @param arg  Unused (FreeRTOS calls entry points with one argument).
 *
 * @retval none
 */
extern "C" void app_main()
{
    ESP_LOGI(TAG, "ESP32-S3 N8R8 boot");
    /* PID (motor gains) and WiFi share the default NVS partition; it must be
     * ready before Arduino and before any queue/task initialization.
     * If init fails we report the error loudly instead of erasing the stored
     * PID on the car. */
    ESP_ERROR_CHECK(nvs_flash_init());
    initArduino();
    ESP_LOGI(TAG, "PSRAM=%u bytes; internal heap=%u, largest block=%u",
             (unsigned)esp_psram_get_size(),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (esp_psram_get_size() != 8U * 1024U * 1024U)
        ESP_LOGW(TAG, "Expected 8 MiB PSRAM; verify module marking and startup PSRAM test");

    /* Create every component queue first; only then start the tasks. */
    esp_link_queues_init();
    esp_net_queues_init();
    const bool camera_ready = camera_queues_init();
    ws2812_queues_init();
    /* Latch STOP right after boot: a freshly rebooted bridge must not leave any
     * old motor target active from before the reboot. */
    esp_link_request_stop();

    /* --- required tasks: the rover stops if any of these cannot start --- */
    start_task(esp_link_rx_task, "esp_rx", 4096, 6, 0, true);
    start_task(esp_link_tx_task, "esp_tx", 3072, 5, 0, true);
    start_task(esp_net_task, "esp_net", 10240, 5, 0, true);
    /* --- optional tasks: degraded operation if any of these fail --- */
    start_task(telemetry_task, "telemetry", 6144, 4, 1, false);
    start_task(cmd_parser_task, "cmd_parser", 6144, 4, 1, false);
    start_task(ws2812_task, "ws2812", 3072, 2, 1, false);   /* LED task always on core 1 */
    if (camera_ready)
        start_task(camera_task, "camera", 8192, 3, 1, false);
    else
        ESP_LOGE(TAG, "camera queue init failed; camera disabled");
    ESP_LOGI(TAG, "Task assembly complete; stored PID restores automatically, motion requires fresh input");
}
