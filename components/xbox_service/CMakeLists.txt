idf_component_register(SRCS "xbox_service.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos xbox_controller c3_uart arduino-esp32)
