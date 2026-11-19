idf_component_register(SRCS "telemetry.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos esp_link esp_net cjson)
