#pragma once
#include "FreeRTOS.h"
#include <deque>
#include <vector>
#include <cstring>
struct MockQueue { unsigned depth, size; std::deque<std::vector<uint8_t>> items; };
using QueueHandle_t = MockQueue*;
inline QueueHandle_t xQueueCreate(unsigned n, unsigned size) { return new MockQueue{n, size, {}}; }
inline BaseType_t xQueueSend(QueueHandle_t q, const void* data, TickType_t) {
    configASSERT(q); if(q->items.size()==q->depth) return pdFALSE;
    auto p=static_cast<const uint8_t*>(data); q->items.emplace_back(p,p+q->size); return pdTRUE;
}
inline BaseType_t xQueueOverwrite(QueueHandle_t q, const void* data) {
    configASSERT(q && q->depth==1); q->items.clear(); return xQueueSend(q,data,0);
}
inline BaseType_t xQueueReceive(QueueHandle_t q, void* data, TickType_t ticks) {
    configASSERT(q); if(q->items.empty()) { if(mock_stop_empty && ticks) throw StopTask{}; return pdFALSE; }
    std::memcpy(data,q->items.front().data(),q->size); q->items.pop_front(); return pdTRUE;
}
inline BaseType_t xQueuePeek(QueueHandle_t q, void* data, TickType_t) {
    configASSERT(q); if(q->items.empty()) return pdFALSE;
    std::memcpy(data,q->items.front().data(),q->size); return pdTRUE;
}
inline void xQueueReset(QueueHandle_t q) { configASSERT(q); q->items.clear(); }
inline void vQueueDelete(QueueHandle_t q) { delete q; }
