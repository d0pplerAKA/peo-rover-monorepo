#pragma once
#include <stdint.h>
/* Host test stub: pretend ample heap. */
#define MALLOC_CAP_INTERNAL 0x08
inline uint32_t esp_get_free_heap_size(void) { return 300000; }
inline uint32_t esp_get_minimum_free_heap_size(void) { return 300000; }
inline size_t heap_caps_get_free_size(uint32_t caps) { (void)caps; return 300000; }
inline size_t heap_caps_get_total_size(uint32_t caps) { (void)caps; return 500000; }
