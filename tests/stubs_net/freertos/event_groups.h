#pragma once
using EventGroupHandle_t = void*;
inline int xEventGroupWaitBits(void*, unsigned, int, int, unsigned) { return 1; }
