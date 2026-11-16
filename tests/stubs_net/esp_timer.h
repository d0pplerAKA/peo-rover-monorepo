#pragma once
#include <stdint.h>
/* Host test stub: monotonic microseconds. Zero is enough for log throttling;
   tests do not assert on this clock. */
inline int64_t esp_timer_get_time(void) { return 0; }
