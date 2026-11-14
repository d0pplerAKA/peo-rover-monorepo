/**
 * @file main.cpp
 * @brief peo_rover_esp32s3 —— Xbox 手柄接收端
 *
 * ESP32-S3 通过 BLE 连接 Xbox Series X 手柄，读取按键/摇杆状态。
 * 后续：把读取到的数据通过 UART 发给 STM32F407。
 */
#include "Arduino.h"
#include "XboxSeriesXControllerESP32_asukiaaa.hpp"

XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

extern "C" void app_main()
{
    initArduino();

    Serial.begin(115200);
    Serial.println("Starting Xbox controller client");

    xboxController.begin();

    while (true) {
        xboxController.onLoop();

        if (xboxController.isConnected()) {
            if (!xboxController.isWaitingForFirstNotification()) {
                Serial.println(xboxController.xboxNotif.toString());
            }
        } else {
            Serial.println("not connected");
        }

        delay(500);
    }
}
