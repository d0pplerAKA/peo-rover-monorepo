#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__

/**
 * @file    telemetry.h
 * @brief   PEO Rover - 遥测合并：传感器帧 + 手柄快照 -> 50Hz JSON 广播
 *
 * telemetry_task（优先级 4）：每轮延时 20ms 后 peek esp_link_sensor_queue，
 * 同时读取 xbox_state_queue 和宏进度最新快照，不移除队列元素，不由 UART 帧到达驱动；
 * 合并成 JSON（PROTOCOL.md §2.1 格式 + gamepad 字段）入 esp_net_tx_queue
 * 广播给全部 TCP 客户端（也服务上位机的手柄动画显示）。
 * 无传感器快照或接收过期时标记 link_stale，手柄心跳过期时不输出 gamepad；
 * JSON 构建期间 generation 变化则丢弃本帧，ACK 由独立队列优先发送。
 */

void telemetry_task(void *arg);

#endif /* __TELEMETRY_H__ */
