/**
 * @file    main.cpp
 * @brief   PEO Rover - ESP32-C3 firmware entry（Xbox BLE 桥）
 *
 * C3 SuperMini（单核 160MHz，4MB flash）跑三个业务任务：xbox_ble_task（BLE 连接+watchdog）+ xbox_sender_task（100Hz 心跳发送）+ led_status_task（状态灯）
 *   BLE 驱动手柄 + 100Hz GAMEPAD_STATE 状态帧直发 F407 USART3（PROTOCOL.md §1.4）。
 * 只上报原始状态（摇杆/扳机/按键/锁）；摇杆映射与安全仲裁在 F407 端。
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
#include "led_status.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "peo_c3_boot";

extern "C" void app_main()
{
    ESP_LOGI(TAG, "ESP32-C3 SuperMini: BLE controller RAM retained before initArduino");
    initArduino();

    c3_uart_init();
    led_status_init();

    /* 先建发送任务（100Hz 心跳，不依赖 BLE）；失败则无心跳兜底 → fatal */
    if (xTaskCreate(xbox_sender_task, "xbox_send", 4096, NULL, 6, NULL) != pdPASS) {
        ESP_LOGE(TAG, "xbox sender task allocation failed");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    /* 再建 BLE 任务；失败则离线心跳照发（非 fatal） */
    if (xTaskCreate(xbox_ble_task, "xbox_ble", 6144, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "xbox ble task allocation failed; offline heartbeats only");
    }
    ESP_LOGI(TAG, "Task assembly complete");
}
