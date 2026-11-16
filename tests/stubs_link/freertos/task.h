#pragma once
#include "FreeRTOS.h"
#include <vector>
extern std::vector<char> mock_tx_order;
extern bool mock_stop_on_delay;
inline TickType_t mock_ticks = 100;
inline TickType_t xTaskGetTickCount() { return mock_ticks; }
inline void vTaskDelay(TickType_t ticks) { mock_ticks += ticks; if(ticks==2) mock_tx_order.push_back('I'); if(mock_stop_on_delay) throw StopTask{}; }
inline void vTaskDelete(void*) { throw StopTask{}; }

// Only the single TX task and eSetBits are needed; no host scheduler is simulated.
struct MockTaskNotification { uint32_t value = 0; bool pending = false; };
using TaskHandle_t = MockTaskNotification*;
enum eNotifyAction { eSetBits };
inline MockTaskNotification mock_tx_notification;
inline TaskHandle_t xTaskGetCurrentTaskHandle() { return &mock_tx_notification; }
inline BaseType_t xTaskNotify(TaskHandle_t task, uint32_t value, eNotifyAction action) {
    configASSERT(task && action == eSetBits);
    task->value |= value;
    task->pending = true;
    return pdPASS;
}
inline BaseType_t xTaskNotifyWait(uint32_t clear_entry, uint32_t clear_exit,
                                uint32_t* value, TickType_t ticks) {
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    if (!task->pending) task->value &= ~clear_entry;
    if (!task->pending) {
        // Preserve the harness's old "stop when TX would block on empty" boundary.
        if (mock_stop_empty && ticks) throw StopTask{};
        return pdFALSE;
    }
    if (value) *value = task->value;
    task->value &= ~clear_exit;
    task->pending = false;
    return pdTRUE;
}
