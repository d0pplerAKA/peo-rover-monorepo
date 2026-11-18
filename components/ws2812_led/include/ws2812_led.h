#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file    ws2812_led.h
 * @brief   单颗 WS2812 状态灯（RMT TX 直驱）+ 手动颜色控制。
 *
 * 状态语义（2026-11-16 用户定案）：
 *   - 未连上笔记本热点（PEO_NET_EVT_STA 未置位）：蓝色呼吸（正弦亮度，1.6s 周期）；
 *   - 连上热点：绿色快闪 3 次 → 绿色常亮；
 *   - 断连：回到呼吸。
 * 手动控制：其他组件经 FreeRTOS 队列调用 ws2812_set_color()（1 深 latest-wins），
 * 连接状态下立即覆盖常亮色；连接状态一变（断连/重连）即回到状态灯语义。
 * 禁止直接操作 RMT/GPIO。灯由独立任务（core1）维护。
 */
typedef struct {
    uint8_t r, g, b;      /* 颜色 0-255 */
    uint8_t brightness;   /* 全局亮度 0-100（%） */
} ws2812_cmd_t;

/* 队列创建：必须在 ws2812_task 之前调用（app_main 统一初始化，与其它组件一致） */
void ws2812_queues_init(void);

/* 灯珠维护任务：阻塞等命令 → 亮度缩放 → RMT 发送。core1 创建。 */
void ws2812_task(void *arg);

/* 其它组件的唯一控制入口（1 深 latest-wins 队列，不阻塞调用方）。
   返回 false = 组件未初始化。 */
bool ws2812_set_color(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);

#ifdef __cplusplus
}
#endif
