#pragma once
#include "FreeRTOS.h"
using EventBits_t = unsigned;
using EventGroupHandle_t = unsigned*;
inline EventGroupHandle_t xEventGroupCreate() { return new unsigned(0); }
inline EventBits_t xEventGroupSetBits(EventGroupHandle_t e,unsigned b) { configASSERT(e); return *e |= b; }
inline EventBits_t xEventGroupWaitBits(EventGroupHandle_t e,unsigned b,bool,bool,TickType_t) { configASSERT(e); if(!(*e & b)) throw StopTask{}; return *e; }
