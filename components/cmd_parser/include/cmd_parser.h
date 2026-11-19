#ifndef __CMD_PARSER_H__
#define __CMD_PARSER_H__

/**
 * @file  cmd_parser.h
 * @brief Upper-computer command parser: turns TCP JSON lines into esp_link command frames.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

/*
 * cmd_parser_task (priority 4) blocks waiting for JSON lines from
 * esp_net_rx_queue. For each line it:
 *   1. parses {"cmd": name, ...params...},
 *   2. encodes it into a cmd_id + parameter bytes,
 *   3. pushes that into the esp_link command queue.
 * On success it sends back an ACK {"ack": name, "status": 0} through the
 * esp_net ACK queue, which the network task broadcasts with priority.
 *
 * Local commands (handled on the ESP32-S3, never forwarded to the F407):
 * CAM_RES, VIDEO_ON/OFF, WS2812. There is no local macro engine anymore.
 * STOP_ALL and the one-way safety heartbeat are exempt from stale-generation
 * filtering; the heartbeat only renews the connection, it never replies.
 */

void cmd_parser_task(void *arg);

#endif /* __CMD_PARSER_H__ */
