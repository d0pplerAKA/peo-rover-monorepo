idf_component_register(SRCS "cmd_parser.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos esp_link esp_net cjson camera_service ws2812_led)
