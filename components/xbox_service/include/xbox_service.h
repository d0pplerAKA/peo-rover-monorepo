#ifndef __XBOX_SERVICE_H__
#define __XBOX_SERVICE_H__

/**
 * @file    xbox_service.h
 * @brief   PEO Rover - ESP32-C3 Xbox BLE 桥任务（BLE 连接 + 状态帧发送）
 *
 * C3 只做 BLE 桥（用户决策 2026-11-14：BLE 与 WiFi AP 同核共存不稳定；2026-11-18 起直连 F407）：
 *   xbox_ble_task（优先级 5）：驱动 asukiaaa 库（扫描/连接/重连/onLoop）+ 首通知 watchdog。
 *   xbox_sender_task（优先级 6）：100Hz 打包 GAMEPAD_STATE 状态帧经 c3_uart 直发 F407 USART3。
 * 只上报原始状态 + C3 本地维护控制权锁；摇杆映射与安全仲裁在 F407 端（PROTOCOL.md §1.4）。
 *
 * 状态帧无论手柄是否连接都发，兼作 C3 存活心跳（F407 端 500ms 无帧 = C3 失联 → 停车）。
 */

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* 目标手柄 MAC（用户实测记录 2026-11-14）；置空 "" 则按 appearance 自动扫描 */
#define PEO_XB_TARGET_MAC   "68:6c:e6:42:57:89"

/* 输入新鲜度判定阈值（C3 本地）：最近 N ms 内有有效 HID 通知才置 input_fresh=1。
   判定结果直接进状态帧，由 F407 直接消费（不再由 seq 时间戳推断）。 */
#define XB_INPUT_MAX_AGE_MS 5000U

/* 控制权锁：长按锁键（Share 截图键）翻转「手操/自主」。
   锁关闭（默认）= 手操（手柄控制）；锁开启 = 自主（上位机控制）。 */
#define XB_LOCK_BTN        XB_BTN_SHARE   /* 锁键：Share（Xbox logo 正下方） */
#define XB_LOCK_HOLD_MS    1500U           /* 长按阈值：1.5s */
/* 首通知 watchdog：BLE 已连但超过此时长仍无首个有效 HID → 强制断连触发重连 */
#define XB_FIRST_NOTIF_TIMEOUT_MS 5000U

/* 按键位图（与 PROTOCOL.md §1.4 buttons 位序一致） */
#define XB_BTN_A            ((uint16_t)(1U << 0))
#define XB_BTN_B            ((uint16_t)(1U << 1))
#define XB_BTN_X            ((uint16_t)(1U << 2))
#define XB_BTN_Y            ((uint16_t)(1U << 3))
#define XB_BTN_LB           ((uint16_t)(1U << 4))
#define XB_BTN_RB           ((uint16_t)(1U << 5))
#define XB_BTN_START        ((uint16_t)(1U << 6))
#define XB_BTN_SELECT       ((uint16_t)(1U << 7))
#define XB_BTN_SHARE        ((uint16_t)(1U << 8))
#define XB_BTN_LS           ((uint16_t)(1U << 9))
#define XB_BTN_RS           ((uint16_t)(1U << 10))
#define XB_BTN_DPAD_UP      ((uint16_t)(1U << 11))
#define XB_BTN_DPAD_DOWN    ((uint16_t)(1U << 12))
#define XB_BTN_DPAD_LEFT    ((uint16_t)(1U << 13))
#define XB_BTN_DPAD_RIGHT   ((uint16_t)(1U << 14))
#define XB_BTN_XBOX         ((uint16_t)(1U << 15))

/** 100Hz 心跳发送任务（不依赖 BLE；独占 fill_state/update_lock/s_lock_* 与 UART 发送） */
void xbox_sender_task(void *arg);
void xbox_ble_task(void *arg);

#endif /* __XBOX_SERVICE_H__ */
