#pragma once
#include <stdint.h>
#include <stdexcept>
using TickType_t = uint32_t;
using BaseType_t = int;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) (ms)
#define portTICK_PERIOD_MS 1  // host_ticks in task.h is measured in milliseconds.
#define configASSERT(v) do { if (!(v)) throw std::runtime_error("configASSERT"); } while (0)
using portMUX_TYPE = int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(p) ((void)(p))
#define portEXIT_CRITICAL(p) ((void)(p))

// Minimal single-task notification surface used by esp_link; no blocking scheduler.
struct HostTaskNotification { uint32_t value = 0; bool pending = false; };
using TaskHandle_t = HostTaskNotification*;
enum eNotifyAction { eSetBits };
inline HostTaskNotification host_task_notification;
inline TaskHandle_t xTaskGetCurrentTaskHandle() { return &host_task_notification; }
inline BaseType_t xTaskNotify(TaskHandle_t task, uint32_t value, eNotifyAction action) {
    configASSERT(task && action == eSetBits);
    task->value |= value;
    task->pending = true;
    return pdPASS;
}
inline BaseType_t xTaskNotifyWait(uint32_t clear_entry, uint32_t clear_exit,
                                uint32_t* value, TickType_t) {
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    if (!task->pending) task->value &= ~clear_entry;
    if (!task->pending) return pdFALSE;
    if (value) *value = task->value;
    task->value &= ~clear_exit;
    task->pending = false;
    return pdTRUE;
}
