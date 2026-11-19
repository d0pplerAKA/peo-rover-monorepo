/**
 * @file  xbox_service.cpp
 * @brief Xbox BLE bridge implementation: drives the asukiaaa library and sends state frames.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

/**
 * asukiaaa library usage: begin() initialises NimBLE, then onLoop() is called in
 * a loop to automatically scan / connect / reconnect. When a notification arrives
 * the library updates its internal xboxNotif fields for us to read.
 * Every 10 ms we pack a GAMEPAD_STATE frame and send it to the F407 (whether or
 * not the controller is connected — the frame doubles as the heartbeat).
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

/** Log tag shown in the serial console for messages from this file. */
static const char *TAG = "xbox_service";

/**
 * Control lock state (maintained on the C3): a long press of the lock button
 * toggles it. false = closed (manual, controller drives), true = open
 * (autonomous, upper computer drives).
 */
static bool s_lock_open = false;

/** The asukiaaa driver instance, filtered to the target MAC (PEO_XB_TARGET_MAC).
 *  Empty string = auto-scan by appearance instead. */
static XboxSeriesXControllerESP32_asukiaaa::Core s_xbox(PEO_XB_TARGET_MAC);

/**
 * Lock-button long-press tracking. An Xbox controller only sends an HID report
 * when an input CHANGES, so while the button is held there are no new reports;
 * we therefore time the hold from "pressed" to "released" without requiring a
 * constant stream of fresh reports (otherwise the hold would be wrongly judged
 * as stale ~100 ms in and never reach the threshold).
 * A disconnect-generation change (new session) resets everything, and after a
 * trigger we require the button to be observed released before it can fire again.
 */
static uint32_t s_lock_press_ms = 0;           /**< millis() timestamp when the current press began. */
static uint32_t s_lock_disc_gen = 0;           /**< Disconnect generation the press belongs to. */
static bool     s_lock_press_active = false;   /**< true while the lock button is held. */
static bool     s_lock_triggered = false;      /**< true after the hold already toggled the lock. */

/** Lock-toggle rumble request: the sender (update_lock) sets it, the BLE task
 *  consumes it and writes the HID rumble report. */
static std::atomic<bool> s_rumble_request{false};

/**
 * @brief  Update the control lock from the lock-button state.
 *
 * Plain-language behaviour:
 *  - If the disconnect generation changed (a new session), reset all tracking.
 *  - If the button is released, cancel the current press and allow a new trigger.
 *  - If the button is held and this hold already toggled the lock, do nothing.
 *  - Otherwise time the hold; once it reaches XB_LOCK_HOLD_MS, toggle the lock,
 *    mark it triggered, and request the rumble feedback.
 *
 * @param  btn_pressed  true while the lock button (Share) is held.
 * @param  disc_gen     Current disconnect generation from the controller snapshot.
 */
static void update_lock(bool btn_pressed, uint32_t disc_gen)
{
    // New session (disconnect generation changed) -> reset all long-press state.
    if (disc_gen != s_lock_disc_gen) {
        s_lock_disc_gen = disc_gen;
        s_lock_press_active = false;
        s_lock_press_ms = 0;
        s_lock_triggered = false;
        return;
    }

    // Button observed released -> cancel the timer and allow a future trigger.
    if (!btn_pressed) {
        s_lock_press_active = false;
        s_lock_press_ms = 0;
        s_lock_triggered = false;
        return;
    }

    // This hold already toggled the lock -> do not toggle again until released.
    if (s_lock_triggered)
        return;

    // Time the hold. NOTE: the controller only sends HID reports when an input
    // changes, so getting no new report while held is normal — do NOT gate
    // freshness on "a recent HID report arrived" (that would falsely reset the
    // timer ~100 ms into the hold and the threshold would never be reached).
    // Disconnect / link loss is covered by the disconnect-generation reset above.
    if (!s_lock_press_active) {
        s_lock_press_active = true;
        s_lock_press_ms = millis();
    } else if ((uint32_t)(millis() - s_lock_press_ms) >= XB_LOCK_HOLD_MS) {
        s_lock_open = !s_lock_open;
        s_lock_triggered = true;
        s_rumble_request.store(true, std::memory_order_relaxed);  /* lock toggled -> ask for rumble */
        ESP_LOGI(TAG, "lock toggled -> %s", s_lock_open ? "OPEN (auto)" : "CLOSED (manual)");
    }
}

/* ===== First-notification watchdog: BLE connected but no first valid HID for
 * too long -> force a disconnect so the stack reconnects. ===== */
static uint32_t s_first_notif_wait_ms = 0;   /**< millis() when waiting for the first HID began. */
static uint32_t s_first_notif_disc_gen = 0;  /**< Disconnect generation the wait belongs to. */
static bool     s_waiting_first = false;     /**< true while waiting for the first HID. */

/**
 * @brief  Force a disconnect of the target controller so the stack reconnects.
 *
 * The library filters the target MAC with address type 0 (public); this version's
 * constructor needs the type passed explicitly. getClientByPeerAddress compares
 * both address bytes and type, so we look it up with the same type.
 */
static void force_disconnect(void)
{
    NimBLEClient *pClient = NimBLEDevice::getClientByPeerAddress(NimBLEAddress(PEO_XB_TARGET_MAC, 0));
    if (pClient != nullptr) {
        ESP_LOGW(TAG, "no HID after connect; forcing disconnect to reconnect");
        pClient->disconnect();
    }
}

/**
 * @brief  Watchdog: if BLE is connected but no first HID notification arrived
 *         within XB_FIRST_NOTIF_TIMEOUT_MS, force a disconnect to trigger a
 *         reconnect.
 */
static void watchdog_first_notification(void)
{
    // Not waiting (first HID already arrived or not connected) -> nothing to do.
    if (!s_xbox.isWaitingForFirstNotification()) {
        s_waiting_first = false;
        s_first_notif_wait_ms = 0;
        return;
    }

    const uint32_t disc_gen = s_xbox.getSnapshot().disconnectGeneration;

    // First time we notice the wait -> start the timer for this session.
    if (!s_waiting_first) {
        s_waiting_first = true;
        s_first_notif_wait_ms = millis();
        s_first_notif_disc_gen = disc_gen;
        return;
    }

    // Session changed mid-wait -> restart the timer for the new session.
    if (disc_gen != s_first_notif_disc_gen) {
        s_first_notif_disc_gen = disc_gen;
        s_first_notif_wait_ms = millis();
        return;
    }

    // Waited too long -> force a reconnect and clear the wait state.
    if ((uint32_t)(millis() - s_first_notif_wait_ms) >= XB_FIRST_NOTIF_TIMEOUT_MS) {
        force_disconnect();
        s_waiting_first = false;
        s_first_notif_wait_ms = 0;
    }
}

/**
 * @brief  Copy the asukiaaa notification fields into a GAMEPAD_STATE payload.
 *
 * Reads one consistent snapshot (thread-safe via the library's StateGuard),
 * fills every field of the frame body, and updates the control lock.
 *
 * @param  p  Pointer to the payload struct to fill (must not be NULL).
 */
static void fill_state(c3_gamepad_payload_t *p)
{
    const auto snapshot = s_xbox.getSnapshot();
    const auto &n = snapshot.notification;

    p->seq       = snapshot.notificationSequence;
    p->disc_gen  = snapshot.disconnectGeneration;
    p->connected = snapshot.connected ? 1 : 0;
    p->battery   = snapshot.battery;

    // Input freshness (judged locally): connected AND a valid HID notification
    // within the last 5 s. The F407 uses this to detect stalled input (stale
    // input -> stop), instead of inferring it from the seq timestamp.
    p->input_fresh = (snapshot.connected &&
        (uint32_t)(millis() - snapshot.receivedNotificationAt) <= XB_INPUT_MAX_AGE_MS) ? 1 : 0;

    p->joy_lx     = n.joyLHori;
    p->joy_ly     = n.joyLVert;
    p->joy_rx     = n.joyRHori;
    p->joy_ry     = n.joyRVert;
    p->trigger_lt = n.trigLT;
    p->trigger_rt = n.trigRT;

    // Build the button bitmap from each individual button field.
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

    // Control lock: toggled by a long press of the lock button; report it in the frame.
    update_lock((p->buttons & XB_LOCK_BTN) != 0, snapshot.disconnectGeneration);
    p->lock = s_lock_open ? 1 : 0;
}

/**
 * @brief  Rumble feedback on lock toggle: both main motors at 80% for 0.3 s.
 */
static void rumble_lock_feedback(void)
{
    XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase r;
    r.setAllOff();
    r.v.select.left = true;    // enable the left motor
    r.v.select.right = true;   // enable the right motor
    r.v.power.left = 80;       // left motor strength (percent)
    r.v.power.right = 80;      // right motor strength (percent)
    r.v.timeActive = 30;       // 0.30 s (30 x 10 ms)
    r.v.timeSilent = 0;
    r.v.countRepeat = 0;
    s_xbox.writeHIDReport(r);
}

/**
 * @brief  BLE task entry point (priority 5): drives the library and watchdog.
 * @param  arg  Unused (FreeRTOS task argument).
 */
void xbox_ble_task(void *arg)
{
    if (!s_xbox.begin()) {
        // Accepted edge case: BLE init failed -> the sender task keeps sending
        // offline heartbeats; BLE needs a reboot to recover (no self-healing here).
        ESP_LOGE(TAG, "BLE init failed; offline (heartbeat handled by sender task, reboot to recover)");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "ble task up (target %s)", PEO_XB_TARGET_MAC);

    while (1) {
        s_xbox.onLoop();                /* scan / connect / reconnect state machine (may block synchronously) */
        watchdog_first_notification();  /* recover if the first HID never arrives */
        if (s_rumble_request.exchange(false, std::memory_order_relaxed))
            rumble_lock_feedback();     /* lock-toggle rumble; written in the BLE task to avoid the sender touching pConnectedClient across tasks */
        vTaskDelay(pdMS_TO_TICKS(20));  /* BLE state-machine tick */
    }
}

/**
 * @brief  Sender task entry point (priority 6): 100 Hz heartbeat + state frames.
 *
 * Runs independently of BLE: it keeps sending the current snapshot during
 * connect / reconnect / discovery (connected = 0 until the first valid HID).
 * Uses vTaskDelayUntil so each loop is a precise 10 ms period (the task body
 * time must not stretch the period).
 *
 * @param  arg  Unused (FreeRTOS task argument).
 */
void xbox_sender_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();   // start of the first period
    while (1) {
        c3_gamepad_payload_t p;
        fill_state(&p);
        if (!c3_uart_send_gamepad(&p))
            ESP_LOGW(TAG, "state frame send failed");
        // Status LED uses isConnected() (true as soon as the BLE link is up),
        // NOT snapshot.connected (which stays false until the first HID arrives) —
        // otherwise the LED would keep breathing while the controller is already
        // connected but no button has been pressed yet (reported on real hardware).
        led_status_notify_connected(s_xbox.isConnected());
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}
