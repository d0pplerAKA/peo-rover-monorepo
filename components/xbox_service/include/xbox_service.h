#ifndef __XBOX_SERVICE_H__
#define __XBOX_SERVICE_H__

/**
 * @file    xbox_service.h
 * @brief   PEO Rover - ESP32-C3 Xbox BLE 桥任务（BLE 连接 + 状态帧发送）
 *
 * C3 只做 BLE 桥（用户决策 2026-11-14：BLE 与 WiFi AP 同核共存不稳定）：
 *   xbox_ble_task（优先级 5）：驱动 asukiaaa 库（扫描/连接/重连/onLoop），
 *                              ~50Hz 打包 GAMEPAD_STATE 状态帧经 c3_uart 发 ESP32-S3。
 * 不做映射；映射逻辑在 ESP32-S3 的 c3_link 组件（PROTOCOL.md §1.4）。
 *
 * 状态帧无论手柄是否连接都发，兼作 C3 存活心跳（S3 端 500ms 无帧 = C3 失联 → 锁 STOP）。
 */

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* 目标手柄 MAC（用户实测记录 2026-11-14）；置空 "" 则按 appearance 自动扫描 */
#define PEO_XB_TARGET_MAC   "68:6c:e6:42:57:89"

/* 按键位图（与 PROTOCOL.md §1.4 buttons 位序、S3 c3_link 组件 XB_BTN_* 一致） */
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

void xbox_ble_task(void *arg);

#endif /* __XBOX_SERVICE_H__ */
