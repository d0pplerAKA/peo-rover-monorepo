cmake_minimum_required(VERSION 3.16)

# Fixes "ninja: CreateProcess: The parameter is incorrect" on Windows (command line too long):
# This project has many managed components; expanded build commands exceed cmd.exe's 8191-char limit.
# Force the Ninja generator to put all compile/link args into response files (@rsp), keeping the command line short.
set(CMAKE_NINJA_FORCE_RESPONSE_FILE 1)

# Exclude esp_lcd: this project only bridges ESP32-S3 and F407, no LCD;
# and esp_lcd has a known ICE (internal compiler error) under Windows + gcc 15.2.0
set(EXCLUDE_COMPONENTS esp_lcd)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(peo_rover_esp32s3)

# ESP-IDF v6.1 escalates stringop-truncation to an error, breaking third-party components (esp-modbus etc.) that Arduino depends on; demote back to a warning
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=stringop-truncation" APPEND)

# Arduino platform macro for components.
add_compile_definitions(ARDUINO_ARCH_ESP32=1)
