/**
 * @file    xbox_service.cpp
 * @brief   PEO Rover - ESP32-C3 Xbox BLE 桥任务：驱动 asukiaaa 库 + 状态帧发送
 *
 * asukiaaa 库用法：begin() 初始化 NimBLE -> 循环 onLoop() 自动完成
 * 扫描/连接/断线重连；通知到达时库内部更新 xboxNotif 字段。
 * 每 10ms 打包 GAMEPAD_STATE 状态帧发 F407（无论手柄是否连接，兼作心跳）。
 */

#include "xbox_service.h"
#include "c3_uart.h"
#include "led_status.h"

#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <atomic>

static const char *TAG = "xbox_service";

/* 控制权锁状态（C3 端维护）：长按锁键翻转。
   false=锁关闭（手操，手柄控制），true=锁开启（自主，上位机控制）。 */
static bool s_lock_open = false;

/* 目标 MAC 过滤（PEO_XB_TARGET_MAC）；空串 = 按 appearance 自动扫描 */
static XboxSeriesXControllerESP32_asukiaaa::Core s_xbox(PEO_XB_TARGET_MAC);

/* 锁键长按检测：仅当「持续有新的 HID 通知到达」（live）才累计按压时长。
   手柄停更/睡眠（btn 残留为按下、但 BLE 未断）不再被当作持续按住，避免误翻转控制权。
   断连代次变化（新会话）整体重置；触发后须观察到释放才能再次触发。 */
static uint32_t s_lock_press_ms = 0;
static uint32_t s_lock_disc_gen = 0;
static bool     s_lock_press_active = false;
static bool     s_lock_triggered = false;

/* 锁翻转震动请求：sender（update_lock）置位，BLE 任务消费并写 HID rumble 报告。 */
static std::atomic<bool> s_rumble_request{false};

static void update_lock(bool btn_pressed, uint32_t disc_gen)
{
    /* 断连代次变化 = 新会话，整体重置长按状态 */
    if (disc_gen != s_lock_disc_gen) {
        s_lock_disc_gen = disc_gen;
        s_lock_press_active = false;
        s_lock_press_ms = 0;
        s_lock_triggered = false;
        return;
    }

    /* 观察到释放：取消计时 + 允许下次触发 */
    if (!btn_pressed) {
        s_lock_press_active = false;
        s_lock_press_ms = 0;
        s_lock_triggered = false;
        return;
    }

    /* 本次长按已触发，持续按住不重复翻转（须先观察到释放） */
    if (s_lock_triggered)
        return;

    /* 累计按压时长。⚠️ 手柄只在输入变化时上报 HID，按住期间无新报告属正常，
       不能用「近期有新 HID」做新鲜度门控（否则按住 ~100ms 就被误判停更、计时归零、
       长按永远攒不到阈值）。断连/失联由 disc_gen 代次变化兜底。 */
    if (!s_lock_press_active) {
        s_lock_press_active = true;
        s_lock_press_ms = millis();
    } else if ((uint32_t)(millis() - s_lock_press_ms) >= XB_LOCK_HOLD_MS) {
        s_lock_open = !s_lock_open;
        s_lock_triggered = true;
        s_rumble_request.store(true, std::memory_order_relaxed);  /* 锁翻转 → 请求手柄震动 */
        ESP_LOGI(TAG, "lock toggled -> %s", s_lock_open ? "OPEN (auto)" : "CLOSED (manual)");
    }
}

/* ===== 首通知 watchdog：BLE 已连但长期无首个有效 HID → 强制断连重连 ===== */
static uint32_t s_first_notif_wait_ms = 0;
static uint32_t s_first_notif_disc_gen = 0;
static bool     s_waiting_first = false;

static void force_disconnect(void)
{
    /* 库内目标 MAC 按 type=0（public）过滤；此版本构造函数需显式传 type。
       getClientByPeerAddress 同时比较地址字节与 type，因此沿用相同类型查找。 */
    NimBLEClient *pClient = NimBLEDevice::getClientByPeerAddress(NimBLEAddress(PEO_XB_TARGET_MAC, 0));
    if (pClient != nullptr) {
        ESP_LOGW(TAG, "no HID after connect; forcing disconnect to reconnect");
        pClient->disconnect();
    }
}

static void watchdog_first_notification(void)
{
    if (!s_xbox.isWaitingForFirstNotification()) {
        s_waiting_first = false;
        s_first_notif_wait_ms = 0;
        return;
    }

    const uint32_t disc_gen = s_xbox.getSnapshot().disconnectGeneration;

    if (!s_waiting_first) {
        s_waiting_first = true;
        s_first_notif_wait_ms = millis();
        s_first_notif_disc_gen = disc_gen;
        return;
    }

    if (disc_gen != s_first_notif_disc_gen) {
        s_first_notif_disc_gen = disc_gen;
        s_first_notif_wait_ms = millis();
        return;
    }

    if ((uint32_t)(millis() - s_first_notif_wait_ms) >= XB_FIRST_NOTIF_TIMEOUT_MS) {
        force_disconnect();
        s_waiting_first = false;
        s_first_notif_wait_ms = 0;
    }
}

/** 把 asukiaaa 通知字段装进 GAMEPAD_STATE 状态帧 payload（PROTOCOL.md §1.4） */
static void fill_state(c3_gamepad_payload_t *p)
{
    const auto snapshot = s_xbox.getSnapshot();
    const auto &n = snapshot.notification;

    p->seq       = snapshot.notificationSequence;
    p->disc_gen  = snapshot.disconnectGeneration;
    p->connected = snapshot.connected ? 1 : 0;
    p->battery   = snapshot.battery;

    /* 输入新鲜度（C3 本地判定）：已连接且最近 5s 内有有效 HID 通知。
       F407 端据此判定输入停摆（输入过期→停车），不再由 seq 时间戳推断。 */
    p->input_fresh = (snapshot.connected &&
        (uint32_t)(millis() - snapshot.receivedNotificationAt) <= XB_INPUT_MAX_AGE_MS) ? 1 : 0;

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

    /* 控制权锁：长按锁键（XB_LOCK_BTN）翻转，随状态帧上报给 F407。 */
    update_lock((p->buttons & XB_LOCK_BTN) != 0, snapshot.disconnectGeneration);
    p->lock = s_lock_open ? 1 : 0;
}

/* 锁翻转的震动反馈：主马达（left+right）80% 强度短震 0.3s。 */
static void rumble_lock_feedback(void)
{
    XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase r;
    r.setAllOff();
    r.v.select.left = true;
    r.v.select.right = true;
    r.v.power.left = 80;
    r.v.power.right = 80;
    r.v.timeActive = 30;   /* 0.30s */
    r.v.timeSilent = 0;
    r.v.countRepeat = 0;
    s_xbox.writeHIDReport(r);
}

void xbox_ble_task(void *arg)
{
    if (!s_xbox.begin()) {
        /* 边界（用户拍板）：BLE init 失败 → 离线心跳由 sender 任务持续发送；BLE 需重启恢复，不做自愈 */
        ESP_LOGE(TAG, "BLE init failed; offline (heartbeat handled by sender task, reboot to recover)");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "ble task up (target %s)", PEO_XB_TARGET_MAC);

    while (1) {
        s_xbox.onLoop();                /* 扫描/连接/断线重连状态机（可能同步阻塞） */
        watchdog_first_notification();  /* 首通知超时恢复 */
        if (s_rumble_request.exchange(false, std::memory_order_relaxed))
            rumble_lock_feedback();     /* 锁翻转震动（在 BLE 任务写，避免 sender 跨任务碰 pConnectedClient） */
        vTaskDelay(pdMS_TO_TICKS(20));  /* BLE 状态机节拍 */
    }
}

void xbox_sender_task(void *arg)
{
    /* 100Hz 心跳发送：不依赖 BLE，连接/重连/发现期间照发当前快照（首个有效 HID 前 connected=0）。 */
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        c3_gamepad_payload_t p;
        fill_state(&p);
        if (!c3_uart_send_gamepad(&p))
            ESP_LOGW(TAG, "state frame send failed");
        /* 状态灯用 isConnected()（BLE 链路建立即 true），而非 snapshot.connected（要等首个 HID 才 true）——
           否则手柄已连但还没按键时灯会一直呼吸（用户实机报出）。 */
        led_status_notify_connected(s_xbox.isConnected());
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}
