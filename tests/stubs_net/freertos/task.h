#pragma once
#include "FreeRTOS.h"
#include <functional>
struct TaskDone {};
inline TickType_t host_ticks = 0;
inline std::function<void()> delay_hook;
inline void vTaskDelay(TickType_t t) { host_ticks += t; if (delay_hook) delay_hook(); }
inline void vTaskDelayUntil(TickType_t* last, TickType_t t) { *last+=t; vTaskDelay(t); }
inline TickType_t xTaskGetTickCount() { return host_ticks; }
inline void vTaskDelete(void*) { throw TaskDone{}; }
