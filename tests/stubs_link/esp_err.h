#pragma once
#include <stdexcept>
using esp_err_t = int;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERROR_CHECK(x) do { if((x)!=ESP_OK) throw std::runtime_error("ESP_ERROR_CHECK"); } while(0)
inline const char* esp_err_to_name(int) { return "mock-error"; }
