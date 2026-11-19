#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__

/**
 * @file  telemetry.h
 * @brief Telemetry merge: sensor frames -> 20 Hz JSON broadcast to the upper computer.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

/*
 * telemetry_task (priority 4) wakes every 50 ms (fixed cadence, 20 Hz) and
 * peeks the latest esp_link sensor snapshot (peek does NOT remove the element
 * and is not driven by UART frame arrival). It merges the snapshot into one
 * JSON object (PROTOCOL.md section 2.1 format) and hands it to the network
 * task via esp_net_publish_telemetry, which broadcasts it to the client.
 *
 * - No snapshot, or a snapshot that is too old, is flagged with "link_stale".
 * - If the safety generation changes while the JSON is being built, this frame
 *   is dropped (it would mix two safety epochs); ACKs travel on their own queue.
 * - The old gamepad / macro fields were removed together with those components
 *   on 2026-11-15.
 */

void telemetry_task(void *arg);

#endif /* __TELEMETRY_H__ */
