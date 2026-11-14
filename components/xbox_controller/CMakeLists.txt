idf_component_register(
    SRCS "XboxControllerNotificationParser.cpp" "XboxControllerElite2NotificationParser.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp-nimble-cpp arduino-esp32
)

# 兼容 NimBLE-Arduino 的 NIMBLE_MAX_CONNECTIONS 宏（esp-nimble-cpp 用 MYNEWT_VAL(BLE_MAX_CONNECTIONS)）
target_compile_definitions(${COMPONENT_LIB} PUBLIC "NIMBLE_MAX_CONNECTIONS=MYNEWT_VAL(BLE_MAX_CONNECTIONS)")
