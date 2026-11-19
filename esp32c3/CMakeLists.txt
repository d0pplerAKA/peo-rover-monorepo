cmake_minimum_required(VERSION 3.16)

# Windows cmd.exe 8191 字符上限：编译参数进响应文件（同 peo_rover_esp32s3 工程）
set(CMAKE_NINJA_FORCE_RESPONSE_FILE 1)

# 排除 esp_lcd：本工程不用 LCD；规避 Windows + gcc 15.2.0 工具链已知 ICE bug（同 S3 工程）
set(EXCLUDE_COMPONENTS esp_lcd)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(peo_rover_esp32c3)

# ESP-IDF v6.1 把 stringop-truncation 升级为错误，第三方组件（Arduino 依赖）编译失败；降回警告
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=stringop-truncation" APPEND)

add_compile_definitions(ARDUINO_ARCH_ESP32=1)
