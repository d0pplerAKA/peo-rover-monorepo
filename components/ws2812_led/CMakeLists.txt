idf_component_register(SRCS "ws2812_led.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos peo_pinmap esp_driver_rmt)
