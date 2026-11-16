#pragma once
#include <cstdint>
#include <cstring>
#ifndef ESP_OK
#define ESP_OK 0
#endif
struct wifi_config_t { struct { unsigned char ssid[32], password[64]; int ssid_len,max_connection,authmode,channel; } ap; };
struct wifi_ap_record_t { uint8_t primary; int8_t rssi; };
struct wifi_scan_config_t {
    int scan_type;
    struct { struct { uint32_t min, max; } active; } scan_time;
};
using wifi_init_config_t = int;
#define WIFI_INIT_CONFIG_DEFAULT() 0
#define WIFI_AUTH_WPA2_PSK 1
#define WIFI_MODE_AP 1
#define WIFI_MODE_STA 2
#define WIFI_SCAN_TYPE_ACTIVE 0
#define WIFI_IF_AP 1
inline int esp_wifi_init(int*) {return 0;}
inline int esp_wifi_set_mode(int) {return 0;}
inline int esp_wifi_set_config(int,void*) {return 0;}
inline int esp_wifi_start() {return 0;}
// Compile adapters only: no scan delay, RF activity, AP records or driver-owned list.
inline int esp_wifi_stop() {return ESP_OK;}
inline int esp_wifi_scan_start(const wifi_scan_config_t*, bool) {return ESP_OK;}
inline int esp_wifi_scan_get_ap_num(uint16_t* num) {if(num)*num=0;return ESP_OK;}
inline int esp_wifi_scan_get_ap_records(uint16_t* num, wifi_ap_record_t*) {if(num)*num=0;return ESP_OK;}
inline int esp_wifi_clear_ap_list() {return ESP_OK;}
inline size_t strlcpy(char* d,const char* s,size_t n) { size_t len=strlen(s); if(n){size_t k=len<n-1?len:n-1; memcpy(d,s,k);d[k]=0;} return len; }
