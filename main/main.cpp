/**
 * @file    main.cpp
 * @brief   PEO Rover - ESP32-S3 firmware entry（纯 FreeRTOS 任务装配）
 *
 * Entry assembles queues/tasks, sequences startup and reports hardware memory.
 * Component tasks own their hardware initialization and exchange queue snapshots.
 *
 * 2026-11-15 系统收敛（用户拍板）：Xbox 手柄 BLE 桥（C3 + c3_link + c3_map）与
 * 离线宏引擎（macro_engine）已废弃删除；组员自动化 = 上位机原生 Python 脚本
 * （PROTOCOL §2.4 组员编程模型），本机只做「传感器上报 + 命令透传 + 视频推流」。
 *
 * Core affinity is a scheduling policy:
 *   core 0（协议与射频）：esp_rx / esp_tx / esp_net
 *   core 1（应用逻辑；AUTOSTART_ARDUINO 关闭，无 Arduino loopTask）：
 *                                                    telemetry / cmd_parser / camera
 *
 * 队列初始化：全部队列由 xxx_queues_init() 在任务创建之前统一创建——
 * 跨核绑定时优先级不保证时序。
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
    /* PID/WiFi 共用默认 NVS 分区；必须在 Arduino 和队列/任务初始化前就绪。
       初始化失败明确报错，不自动擦除车上保存的 PID。 */
    ESP_ERROR_CHECK(nvs_flash_init());
    initArduino();
    ESP_LOGI(TAG, "PSRAM=%u bytes; internal heap=%u, largest block=%u",
             (unsigned)esp_psram_get_size(),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (esp_psram_get_size() != 8U * 1024U * 1024U)
        ESP_LOGW(TAG, "Expected 8 MiB PSRAM; verify module marking and startup PSRAM test");

    esp_link_queues_init();
    esp_net_queues_init();
    const bool camera_ready = camera_queues_init();
    ws2812_queues_init();
    esp_link_request_stop(); // Rebooted bridge must not leave old motor targets active.

    start_task(esp_link_rx_task, "esp_rx", 4096, 6, 0, true);
    start_task(esp_link_tx_task, "esp_tx", 3072, 5, 0, true);
    start_task(esp_net_task, "esp_net", 10240, 5, 0, true);
    start_task(telemetry_task, "telemetry", 6144, 4, 1, false);
    start_task(cmd_parser_task, "cmd_parser", 6144, 4, 1, false);
    start_task(ws2812_task, "ws2812", 3072, 2, 1, false);   /* 灯珠维护任务固定 core1 */
    if (camera_ready)
        start_task(camera_task, "camera", 8192, 3, 1, false);
    else
        ESP_LOGE(TAG, "camera queue init failed; camera disabled");
    ESP_LOGI(TAG, "Task assembly complete; stored PID restores automatically, motion requires fresh input");
}
