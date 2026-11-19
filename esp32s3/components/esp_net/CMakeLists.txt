idf_component_register(SRCS "esp_net.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos esp_link esp_wifi esp_netif esp_event esp_timer lwip esp32-camera mdns vfs)
