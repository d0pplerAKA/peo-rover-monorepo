idf_component_register(SRCS "c3_uart.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos esp_driver_uart peo_pinmap)
