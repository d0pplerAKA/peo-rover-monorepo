#pragma once
#include <cstdint>
#include <stdexcept>
#include <mutex>
using BaseType_t = int;
using UBaseType_t = unsigned;
using TickType_t = uint32_t;
constexpr int pdTRUE = 1, pdFALSE = 0, pdPASS = 1;
constexpr TickType_t portMAX_DELAY = UINT32_MAX;
#define pdMS_TO_TICKS(x) (x)
#define portTICK_PERIOD_MS 1  // Host tick matches the firmware's 1000Hz configuration.
#define configASSERT(x) do { if (!(x)) throw std::runtime_error("configASSERT: " #x); } while (0)
using portMUX_TYPE = std::recursive_mutex;
#define portMUX_INITIALIZER_UNLOCKED {}
#define portENTER_CRITICAL(x) (x)->lock()
#define portEXIT_CRITICAL(x) (x)->unlock()
struct StopTask {};
extern bool mock_stop_empty;
