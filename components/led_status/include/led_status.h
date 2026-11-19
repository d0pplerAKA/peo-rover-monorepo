#ifndef __LED_STATUS_H__
#define __LED_STATUS_H__

#include <stdbool.h>

/** LEDC 初始化 + 创建呼吸/频闪/常亮状态机任务（GPIO8，active low）。 */
void led_status_init(void);

/** 上报连接状态（由发送任务每 10ms 调）；true=已连上手柄。 */
void led_status_notify_connected(bool connected);

#endif /* __LED_STATUS_H__ */
