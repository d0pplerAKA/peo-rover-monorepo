/**
 * @file  main.cpp
 * @brief ESP32-C3 firmware entry point that starts the Xbox BLE bridge tasks.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

/**
 * This file is the "main()" of the ESP32-C3 (a single-core, 160 MHz, 4 MB flash
 * microcontroller). It starts three FreeRTOS tasks:
 *
 *   - xbox_ble_task     : talks to the Xbox controller over BLE and runs the
 *                         first-notification watchdog.
 *   - xbox_sender_task  : sends the controller state to the F407 every 10 ms
 *                         (100 Hz), even when no controller is connected.
 *   - led_status_task   : drives the status LED (created inside led_status).
 *
 * The C3 only forwards RAW controller state (sticks / triggers / buttons / lock).
 * Stick mapping and safety arbitration happen on the F407 (PROTOCOL.md §1.4).
 *
 * Note: BLE RAM must be reserved BEFORE initArduino() runs (same fix as in the
 * S3 project). NimBLE 2.6 still looks up the old header name, so including
 * <esp32-hal-alloc-ble-mem.h> before initArduino() runs reserves the BLE memory
 * and stops Arduino from freeing it.
 */

#include <Arduino.h>
// Reserve BLE RAM before initArduino() (see note above).
#include <esp32-hal-alloc-ble-mem.h>
#include <esp_log.h>

#include "xbox_service.h"
#include "c3_uart.h"
#include "led_status.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/** Log tag shown in the serial console for messages from this file. */
static const char *TAG = "peo_c3_boot";

/**
 * @brief  Firmware entry point (called once by the C runtime on boot).
 *
 * Reserves BLE RAM, initialises the Arduino layer, the UART link and the status
 * LED, then starts the sender and BLE tasks.
 */
extern "C" void app_main()
{
    ESP_LOGI(TAG, "ESP32-C3 SuperMini: BLE controller RAM retained before initArduino");
    initArduino();

    c3_uart_init();     // Set up the UART that talks to the F407 (USART3).
    led_status_init();  // Set up the status LED and its task.

    // Start the sender task FIRST: it sends the 100 Hz heartbeat and does not
    // depend on BLE at all. If it cannot start there is no heartbeat fallback,
    // so treat it as fatal (spin forever).
    if (xTaskCreate(xbox_sender_task, "xbox_send", 4096, NULL, 6, NULL) != pdPASS) {
        ESP_LOGE(TAG, "xbox sender task allocation failed");
        while (true) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // Then start the BLE task. If it cannot start, the offline heartbeat keeps
    // going anyway, so this is NOT fatal.
    if (xTaskCreate(xbox_ble_task, "xbox_ble", 6144, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "xbox ble task allocation failed; offline heartbeats only");
    }
    ESP_LOGI(TAG, "Task assembly complete");
}
