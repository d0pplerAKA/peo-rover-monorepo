idf_component_register(SRCS "main.cpp"
                       REQUIRES arduino-esp32
                                esp_psram
                                esp_hw_support
                                esp_link
                                esp_net
                                telemetry
                                cmd_parser
                                camera_service
                       PRIV_REQUIRES nvs_flash)
