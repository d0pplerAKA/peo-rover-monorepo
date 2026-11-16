idf_component_register(SRCS "telemetry.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos esp_link c3_link esp_net macro_engine cjson)
