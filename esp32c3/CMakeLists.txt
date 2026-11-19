cmake_minimum_required(VERSION 3.16)

# Windows cmd.exe 8191-char limit: build args go into response files (same as the peo_rover_esp32s3 project)
set(CMAKE_NINJA_FORCE_RESPONSE_FILE 1)

# Exclude esp_lcd: no LCD here; avoids the known Windows + gcc 15.2.0 ICE bug (same as the S3 project)
set(EXCLUDE_COMPONENTS esp_lcd)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(peo_rover_esp32c3)

# ESP-IDF v6.1 escalates stringop-truncation to an error, breaking third-party components (Arduino deps); demote back to a warning
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=stringop-truncation" APPEND)

add_compile_definitions(ARDUINO_ARCH_ESP32=1)
