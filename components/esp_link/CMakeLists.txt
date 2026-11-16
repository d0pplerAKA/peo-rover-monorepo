idf_component_register(SRCS "esp_link.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos esp_driver_uart peo_pinmap)
