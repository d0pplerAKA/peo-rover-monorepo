#pragma once
#include "FreeRTOS.h"
#include "task.h"
inline int receive_budget=-1;
#include <deque>
#include <vector>
#include <cstring>
struct HostQueue { size_t capacity, size; std::deque<std::vector<char>> data; };
using QueueHandle_t = HostQueue*;
inline bool fail_queue_create = false;
inline QueueHandle_t xQueueCreate(size_t n, size_t s) { return fail_queue_create ? nullptr : new HostQueue{n,s,{}}; }
inline int xQueueSend(QueueHandle_t q, const void* p, TickType_t) { if (!q || q->data.size()>=q->capacity) return 0; q->data.emplace_back((const char*)p,(const char*)p+q->size); return 1; }
inline int xQueuePeek(QueueHandle_t q, void* p, TickType_t) { configASSERT(q); if (q->data.empty()) return 0; memcpy(p,q->data.front().data(),q->size); return 1; }
inline int xQueueReceive(QueueHandle_t q, void* p, TickType_t t) { if(receive_budget==0)throw TaskDone{};if(receive_budget>0)--receive_budget; if (!xQueuePeek(q,p,t)) return 0; q->data.pop_front(); return 1; }
inline int xQueueOverwrite(QueueHandle_t q,const void* p) { if (!q) return 0; q->data.clear(); return xQueueSend(q,p,0); }
inline void vQueueDelete(QueueHandle_t q) { delete q; }
