/**
 * @file    main.cpp
 * @brief   PEO Rover - ESP32-S3 firmware entry（纯 FreeRTOS 任务装配）
 *
 * Entry assembles queues/tasks, sequences startup and reports hardware memory.
 * Component tasks own their hardware initialization and exchange queue snapshots.
 *
 * Xbox 手柄 BLE 已移至 ESP32-C3（用户决策 2026-11-14：BLE 与 WiFi AP 同核共存
 * 不稳定）。本机经 c3_link 组件接收 C3 的 UART 状态帧（PROTOCOL.md §1.4），
 * 映射逻辑在 c3_map_task。原 xbox_service / xbox_controller（BLE）组件已移除。
 *
 * Core affinity is a scheduling policy:
 *   core 0（协议与射频）：esp_rx / esp_tx / c3_link_rx / esp_net
 *   core 1（应用逻辑；AUTOSTART_ARDUINO 关闭，无 Arduino loopTask）：macro / c3_map /
 *                                                    telemetry / cmd_parser / camera
 *
 * 队列初始化：全部队列由 xxx_queues_init() 在任务创建之前统一创建——
 * 跨核绑定时优先级不保证时序（曾发生 core1 的 c3_map 抢在 core0 的
 * c3_link_rx 建队之前 peek NULL 触发 assert）。
 */

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_psram.h>

#include "esp_link.h"
#include "c3_link.h"
#include "esp_net.h"
#include "macro_engine.h"
#include "telemetry.h"
#include "cmd_parser.h"
#include "camera_service.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "peo_boot";

static void start_task(TaskFunction_t entry, const char *name, uint32_t stack,
                       UBaseType_t priority, BaseType_t core, bool required)
{
    if (xTaskCreatePinnedToCore(entry, name, stack, NULL, priority, NULL, core) != pdPASS) {
        if (required) {
            ESP_LOGE(TAG, "Required task %s allocation failed; fatal fault, resume permanently disabled (internal heap=%u)",
                     name, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
            esp_link_fatal_fault();
            /* 必需任务缺失时永久禁止恢复，保持当前状态，不循环重启车辆。 */
            while (true) vTaskDelay(pdMS_TO_TICKS(1000));
        }
        ESP_LOGW(TAG, "Optional task %s allocation failed; degraded operation, continuing startup (internal heap=%u)",
                 name, (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        return;  /* 非必需任务失败仅降级，继续创建后续任务。 */
    }
}

extern "C" void app_main()
{
    ESP_LOGI(TAG, "ESP32-S3 N8R8 boot");
    initArduino();
    ESP_LOGI(TAG, "PSRAM=%u bytes; internal heap=%u, largest block=%u",
             (unsigned)esp_psram_get_size(),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (esp_psram_get_size() != 8U * 1024U * 1024U)
        ESP_LOGW(TAG, "Expected 8 MiB PSRAM; verify module marking and startup PSRAM test");

    esp_link_queues_init();
    c3_link_queues_init();
    esp_net_queues_init();
    macro_queues_init();
    const bool camera_ready = camera_queues_init();
    esp_link_request_stop(); // Rebooted bridge must not leave old motor targets active.

    start_task(esp_link_rx_task, "esp_rx", 4096, 6, 0, true);
    start_task(esp_link_tx_task, "esp_tx", 3072, 5, 0, true);
    start_task(c3_link_rx_task, "c3_rx", 4096, 5, 0, true);
    start_task(esp_net_task, "esp_net", 10240, 5, 0, true);
    start_task(macro_task, "macro", 6144, 4, 1, false);
    start_task(c3_map_task, "c3_map", 4096, 4, 1, true);
    start_task(telemetry_task, "telemetry", 6144, 4, 1, false);
    start_task(cmd_parser_task, "cmd_parser", 6144, 4, 1, false);
    if (camera_ready)
        start_task(camera_task, "camera", 8192, 3, 1, false);
    else
        ESP_LOGE(TAG, "camera queue init failed; camera disabled");
    ESP_LOGI(TAG, "Task assembly complete; fresh deliberate input required to resume");
}
