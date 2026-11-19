idf_component_register(SRCS "camera_service.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos esp_net esp32-camera peo_pinmap esp_psram esp_timer esp_hw_support)
