#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__

/**
 * @file    telemetry.h
 * @brief   PEO Rover - 遥测合并：传感器帧 -> 25Hz JSON 广播
 *
 * telemetry_task（优先级 4）：每轮延时 40ms 后 peek esp_link_sensor_queue
 * （不移除队列元素，不由 UART 帧到达驱动），合并成 JSON（PROTOCOL.md §2.1 格式）
 * 入 esp_net_tx_queue 广播给上位机。
 * 无传感器快照或接收过期时标记 link_stale；
 * JSON 构建期间 generation 变化则丢弃本帧，ACK 由独立队列优先发送。
 * （gamepad / macro 字段已于 2026-11-15 随组件废弃删除。）
 */

void telemetry_task(void *arg);

#endif /* __TELEMETRY_H__ */
