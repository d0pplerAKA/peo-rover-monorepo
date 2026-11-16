#pragma once
#include "FreeRTOS.h"

/* 无调度器的互斥桩；锁序错误/递归加锁在 host 中直接暴露。 */
struct HostSemaphore { bool locked = false; };
using SemaphoreHandle_t = HostSemaphore*;
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return new HostSemaphore{}; }
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t) {
    configASSERT(semaphore && !semaphore->locked);
    semaphore->locked = true;
    return pdTRUE;
}
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t semaphore) {
    configASSERT(semaphore && semaphore->locked);
    semaphore->locked = false;
    return pdTRUE;
}
