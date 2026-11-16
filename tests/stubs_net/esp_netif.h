#pragma once
struct esp_netif_t {};
// Typed, stable placeholders only; neither factory creates a real network interface.
inline esp_netif_t* esp_netif_create_default_wifi_ap() {static esp_netif_t ap;return &ap;}
inline esp_netif_t* esp_netif_create_default_wifi_sta() {static esp_netif_t sta;return &sta;}
inline int esp_netif_init() {return 0;}
