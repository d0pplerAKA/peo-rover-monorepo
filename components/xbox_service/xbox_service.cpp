/**
 * @file    xbox_service.cpp
 * @brief   PEO Rover - ESP32-C3 Xbox BLE 桥任务：驱动 asukiaaa 库 + 状态帧发送
 *
 * asukiaaa 库用法：begin() 初始化 NimBLE -> 循环 onLoop() 自动完成
 * 扫描/连接/断线重连；通知到达时库内部更新 xboxNotif 字段。
 * 每 20ms 打包 GAMEPAD_STATE 状态帧发 ESP32-S3（无论手柄是否连接，兼作心跳）。
 */

#include "xbox_service.h"
#include "c3_uart.h"

#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

static const char *TAG = "xbox_service";

/* 目标 MAC 过滤（PEO_XB_TARGET_MAC）；空串 = 按 appearance 自动扫描 */
static XboxSeriesXControllerESP32_asukiaaa::Core s_xbox(PEO_XB_TARGET_MAC);

/** 把 asukiaaa 通知字段装进 GAMEPAD_STATE 状态帧 payload（PROTOCOL.md §1.4） */
static void fill_state(c3_gamepad_payload_t *p)
{
    const auto snapshot = s_xbox.getSnapshot();
    const auto &n = snapshot.notification;

    p->seq       = snapshot.notificationSequence;
    p->disc_gen  = snapshot.disconnectGeneration;
    p->connected = snapshot.connected ? 1 : 0;
    p->battery   = snapshot.battery;

    p->joy_lx     = n.joyLHori;
    p->joy_ly     = n.joyLVert;
    p->joy_rx     = n.joyRHori;
    p->joy_ry     = n.joyRVert;
    p->trigger_lt = n.trigLT;
    p->trigger_rt = n.trigRT;

    p->buttons = 0;
    if (n.btnA)          p->buttons |= XB_BTN_A;
    if (n.btnB)          p->buttons |= XB_BTN_B;
    if (n.btnX)          p->buttons |= XB_BTN_X;
    if (n.btnY)          p->buttons |= XB_BTN_Y;
    if (n.btnLB)         p->buttons |= XB_BTN_LB;
    if (n.btnRB)         p->buttons |= XB_BTN_RB;
    if (n.btnStart)      p->buttons |= XB_BTN_START;
    if (n.btnSelect)     p->buttons |= XB_BTN_SELECT;
    if (n.btnShare)      p->buttons |= XB_BTN_SHARE;
    if (n.btnLS)         p->buttons |= XB_BTN_LS;
    if (n.btnRS)         p->buttons |= XB_BTN_RS;
    if (n.btnDirUp)      p->buttons |= XB_BTN_DPAD_UP;
    if (n.btnDirDown)    p->buttons |= XB_BTN_DPAD_DOWN;
    if (n.btnDirLeft)    p->buttons |= XB_BTN_DPAD_LEFT;
    if (n.btnDirRight)   p->buttons |= XB_BTN_DPAD_RIGHT;
    if (n.btnXbox)       p->buttons |= XB_BTN_XBOX;
}

void xbox_ble_task(void *arg)
{
    if (!s_xbox.begin()) {
        /* BLE 初始化失败：保持发 connected=0 心跳帧。
           S3 端据此显示手柄离线，且不会误判 C3 失联触发 STOP。 */
        ESP_LOGE(TAG, "BLE init failed; sending disconnected heartbeats only");
        while (1) {
            c3_gamepad_payload_t p = {};
            c3_uart_send_gamepad(&p);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    ESP_LOGI(TAG, "ble task up (target %s)", PEO_XB_TARGET_MAC);

    while (1)
    {
        s_xbox.onLoop();    /* 扫描/连接/断线重连状态机 */

        c3_gamepad_payload_t p;
        fill_state(&p);
        if (!c3_uart_send_gamepad(&p))
            ESP_LOGW(TAG, "state frame send failed");

        vTaskDelay(pdMS_TO_TICKS(20));   /* ~50Hz */
    }
}
