#pragma once
#include <stdint.h>
/* Host test stub: per-core idle run-time counter (ESP-IDF freertos/idf_additions.h).
   Returns 0 so telemetry's first sample stays the "no history" branch. */
using configRUN_TIME_COUNTER_TYPE = uint32_t;
inline configRUN_TIME_COUNTER_TYPE ulTaskGetIdleRunTimeCounterForCore(int core) { (void)core; return 0; }
