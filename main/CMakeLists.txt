idf_component_register(SRCS "main.cpp"
                       REQUIRES arduino-esp32
                                esp_psram
                                esp_hw_support
                                esp_link
                                c3_link
                                esp_net
                                macro_engine
                                telemetry
                                cmd_parser
                                camera_service)
