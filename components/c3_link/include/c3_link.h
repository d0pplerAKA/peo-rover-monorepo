#ifndef __C3_LINK_H__
#define __C3_LINK_H__

/**
 * @file    c3_link.h
 * @brief   PEO Rover - ESP32-C3 手柄桥链路（UART RX 解析 + 命令映射）
 *
 * 手柄 BLE 已移至 ESP32-C3（用户决策 2026-11-14：BLE 与 WiFi AP 同核共存不稳定）。
 * C3 只做 BLE 桥，50Hz 发 GAMEPAD_STATE 状态帧（PROTOCOL.md §1.4）。
 *
 * 本组件两个任务（main 里 xTaskCreate 创建）：
 *   c3_link_rx_task（优先级 5）：UART 初始化 + 逐字节帧状态机解析状态帧
 *                              -> 手柄快照入 xbox_state_queue（1 深 overwrite）。
 *   c3_map_task（优先级 4）：~20Hz 读快照，映射成命令帧入 esp_link_cmd_queue
 *                           （断连/失联自动锁 STOP）。原 xbox_map 安全逻辑原样保留。
 *
 * 原 xbox_service 的 BLE 任务与 asukiaaa 库已随组件移除（迁至 C3 工程 peo_rover_esp32c3）。
 */

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

/* ============================================================================
   映射参数（集中调参处；TODO：实测标定后定值）
   ============================================================================ */
#define XB_JOY_CENTER       32768       /* HID 无符号中值；32767 也在死区内 */
#define XB_DEADZONE         2000        /* TODO 标定：摇杆死区（相对中值） */
#define XB_MAP_RATE_MS      50          /* 映射周期 20Hz */
#define XB_SNAPSHOT_MAX_AGE_MS 1000U   /* 快照心跳超时（C3 50Hz 发帧） */
/* 输入新鲜度判定在 C3 本地（阈值 XB_INPUT_MAX_AGE_MS=5s 见 C3 工程 xbox_service.h），
   判定结果经状态帧 input_fresh 字节下发，S3 直接消费（PROTOCOL.md §1.4）。 */
#define XB_MAX_SPEED_D100   4000        /* TODO 标定：满杆目标速度（0.01 cm/s） */

/* C3 失联判定：连续无状态帧超时（C3 50Hz 发帧，500ms 足够冗余） */
#define C3_LINK_TIMEOUT_MS  500U

/* 状态帧 type（与 C3 端 c3_uart.h / PROTOCOL.md §1.4 一致） */
#define C3_TYPE_GAMEPAD_STATE   0x10

/* 按键位图（与 C3 端 xbox_service.h / PROTOCOL.md §1.4 位序一致） */
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

/* ============================================================================
   手柄状态快照（1 深 overwrite 队列，telemetry / c3_map 消费）
   ============================================================================ */
typedef struct
{
    uint16_t joy_lx, joy_ly;        /* 左摇杆（0~65535，中值 32768） */
    uint16_t joy_rx, joy_ry;        /* 右摇杆 */
    uint16_t trigger_lt, trigger_rt;/* 扳机（0~1023） */
    uint16_t buttons;               /* 按键位图（XB_BTN_*） */
    uint8_t  connected;             /* 0=未连接 1=已连接且有数据 */
    uint8_t  input_fresh;           /* 0/1：C3 本地判定的输入新鲜度（5s 内有有效 HID） */
    uint8_t  battery;               /* 电量 0~100 */
    uint32_t notification_ms;       /* 最后有效 HID 通知（seq 变化时刷新） */
    uint32_t snapshot_ms;           /* 快照心跳（每收到一帧刷新） */
    uint32_t notification_sequence; /* 有效 HID 序号（C3 透传，跨重连递增） */
    uint32_t disconnect_generation; /* 断连代次（C3 透传） */
} xbox_state_t;

extern QueueHandle_t xbox_state_queue;  /* 由 c3_link_queues_init() 创建 */

/** 队列初始化（app_main 里、创建任务之前调用；幂等） */
void c3_link_queues_init(void);

void c3_link_rx_task(void *arg);
void c3_map_task(void *arg);

#endif /* __C3_LINK_H__ */
