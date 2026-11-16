idf_component_register(SRCS "c3_link_rx.cpp" "c3_map.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos esp_driver_uart peo_pinmap esp_link arduino-esp32)
