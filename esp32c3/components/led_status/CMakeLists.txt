idf_component_register(SRCS "led_status.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos esp_driver_ledc peo_pinmap arduino-esp32)
