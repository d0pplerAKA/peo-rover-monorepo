cmake_minimum_required(VERSION 3.16)

# 排除 esp_lcd：本工程只做 ESP32-S3 与 F407 通讯，不用 LCD 显示；
# 且 esp_lcd 在 Windows + gcc 15.2.0 工具链下存在已知 ICE（internal compiler error）bug
set(EXCLUDE_COMPONENTS esp_lcd)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(peo_rover_esp32s3)

# ESP-IDF v6.1 把 stringop-truncation 警告升级为错误，导致 Arduino 依赖的 esp-modbus 等第三方组件编译失败；降回警告
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=stringop-truncation" APPEND)

# esp-nimble-cpp 配合 Arduino 组件时要求，防止 Arduino 释放 BLE 内存
add_compile_definitions(ARDUINO_ARCH_ESP32=1)
