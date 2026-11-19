/**
 * @file  xbox_service.h
 * @brief Xbox BLE bridge: task declarations, button bitmap, and tuning constants.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

#ifndef __XBOX_SERVICE_H__
#define __XBOX_SERVICE_H__

/**
 * The C3 is a pure BLE bridge (decided 2026-11-14: BLE and the WiFi AP sharing
 * one core was unstable; since 2026-11-18 it talks directly to the F407):
 *   - xbox_ble_task    (priority 5): drives the asukiaaa library (scan / connect /
 *                      reconnect / onLoop) plus the first-notification watchdog.
 *   - xbox_sender_task (priority 6): packs a GAMEPAD_STATE frame at 100 Hz and
 *                      sends it to the F407 USART3 via c3_uart.
 * The C3 only reports RAW state and maintains the control lock locally; stick
 * mapping and safety arbitration live on the F407 (PROTOCOL.md §1.4).
 *
 * A state frame is sent whether or not the controller is connected — it doubles
 * as the C3 heartbeat (the F407 stops the rover if no frame arrives for 500 ms).
 */

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/** Target controller MAC address (recorded from a real controller on 2026-11-14).
 *  Set to "" to auto-scan by appearance instead. */
#define PEO_XB_TARGET_MAC   "68:6c:e6:42:57:89"

/** Input-freshness threshold (judged locally on the C3): input_fresh is 1 only
 *  if a valid HID notification arrived within this many milliseconds. The result
 *  goes straight into the state frame and is consumed by the F407 directly (the
 *  F407 no longer infers it from the seq timestamp). */
#define XB_INPUT_MAX_AGE_MS 5000U

/** Control lock: a long press of the lock button (Share) toggles manual/autonomous.
 *  Lock closed (default) = manual (controller drives); lock open = autonomous
 *  (upper computer drives). */
#define XB_LOCK_BTN        XB_BTN_SHARE   /**< Lock button: Share (just below the Xbox logo). */
#define XB_LOCK_HOLD_MS    1500U          /**< Long-press threshold: 1.5 s. */

/** First-notification watchdog: if BLE is connected but no first valid HID
 *  arrives within this time, force a disconnect so the stack reconnects. */
#define XB_FIRST_NOTIF_TIMEOUT_MS 5000U

/* Button bitmap (bit order matches PROTOCOL.md §1.4 "buttons"). */
#define XB_BTN_A            ((uint16_t)(1U << 0))   /**< A button. */
#define XB_BTN_B            ((uint16_t)(1U << 1))   /**< B button. */
#define XB_BTN_X            ((uint16_t)(1U << 2))   /**< X button. */
#define XB_BTN_Y            ((uint16_t)(1U << 3))   /**< Y button. */
#define XB_BTN_LB           ((uint16_t)(1U << 4))   /**< Left bumper (LB). */
#define XB_BTN_RB           ((uint16_t)(1U << 5))   /**< Right bumper (RB). */
#define XB_BTN_START        ((uint16_t)(1U << 6))   /**< Start button. */
#define XB_BTN_SELECT       ((uint16_t)(1U << 7))   /**< Select button. */
#define XB_BTN_SHARE        ((uint16_t)(1U << 8))   /**< Share button (also the lock-toggle button). */
#define XB_BTN_LS           ((uint16_t)(1U << 9))   /**< Left stick clicked in (LS). */
#define XB_BTN_RS           ((uint16_t)(1U << 10))  /**< Right stick clicked in (RS). */
#define XB_BTN_DPAD_UP      ((uint16_t)(1U << 11))  /**< D-pad up. */
#define XB_BTN_DPAD_DOWN    ((uint16_t)(1U << 12))  /**< D-pad down. */
#define XB_BTN_DPAD_LEFT    ((uint16_t)(1U << 13))  /**< D-pad left. */
#define XB_BTN_DPAD_RIGHT   ((uint16_t)(1U << 14))  /**< D-pad right. */
#define XB_BTN_XBOX         ((uint16_t)(1U << 15))  /**< Xbox (guide) button. */

/**
 * @brief  100 Hz heartbeat / sender task (does not depend on BLE). It
 *         exclusively owns fill_state / update_lock / s_lock_* and the UART
 *         send path.
 * @param  arg  Unused (FreeRTOS task argument).
 */
void xbox_sender_task(void *arg);

/**
 * @brief  BLE task: drives the asukiaaa library (scan / connect / reconnect /
 *         onLoop) and the first-notification watchdog.
 * @param  arg  Unused (FreeRTOS task argument).
 */
void xbox_ble_task(void *arg);

#endif /* __XBOX_SERVICE_H__ */
