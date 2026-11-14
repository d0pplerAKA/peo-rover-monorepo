/**
 * @file    main.cpp
 * @brief   PEO Rover - ESP32-C3 firmware entry（Xbox BLE 桥）
 *
 * C3 SuperMini（单核 160MHz，4MB flash）只跑 xbox_ble_task：
 *   BLE 驱动手柄 + ~50Hz GAMEPAD_STATE 状态帧发 ESP32-S3（PROTOCOL.md §1.4）。
 * 不做映射；映射逻辑在 ESP32-S3 c3_link 组件。
 *
 * BLE RAM 必须在 initArduino() 前保留（同 S3 工程：NimBLE 2.6 仍查旧头文件名，
 * 该构造在 initArduino() 之前运行，防止 Arduino 释放 BLE 内存）。
 */

#include <Arduino.h>
// Arduino 4 BLE retention（同 peo_rover_esp32s3 的修复）
#include <esp32-hal-alloc-ble-mem.h>
#include <esp_log.h>

#include "xbox_service.h"
#include "c3_uart.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "peo_c3_boot";

extern "C" void app_main()
{
    ESP_LOGI(TAG, "ESP32-C3 SuperMini: BLE controller RAM retained before initArduino");
    initArduino();

    c3_uart_init();

    if (xTaskCreate(xbox_ble_task, "xbox_ble", 6144, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "xbox_ble task allocation failed");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Task assembly complete");
}
