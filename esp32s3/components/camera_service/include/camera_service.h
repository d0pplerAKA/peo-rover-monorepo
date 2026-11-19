#ifndef __CAMERA_SERVICE_H__
#define __CAMERA_SERVICE_H__

#include <stdint.h>

/**
 * @file  camera_service.h
 * @brief OV2640 camera service: capture -> JPEG -> esp_net TCP video stream.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

/*
 * camera_task (priority 3) initializes esp32-camera using the PEO_PM_CAM_*
 * pins from PEO_PINMAP.h, then loops capturing JPEG frames and hands each to
 * esp_net_send_video() for framed delivery over TCP 8890.
 *
 * If init or rollback fails, the driver is cleaned up and the task stays on
 * standby; a new resolution request can trigger self-healing. The task never
 * exits.
 */

void camera_task(void *arg);

/* ---- Resolution hot-swap (paired with {"cmd":"CAM_RES","res":"QVGA"}, PROTOCOL section 2) ---- */

/** @brief  Create the camera queues. Call before camera_task is created.
 *  @retval false if the resolution queue could not be created (camera then stays disabled). */
bool camera_queues_init(void);

/** @brief  Map a name to a framesize_t.
 *  @param name  "QQVGA" / "QVGA" / "VGA" / "SVGA".
 *  @param out   Receives the matching framesize_t.
 *  @retval true  if the name is valid and out was written;
 *  @retval false otherwise. */
bool camera_resolution_from_name(const char *name, uint8_t *out);

/** @brief  Request a new resolution (1-deep overwrite queue keeps only the latest).
 *  @retval false if the queue does not exist; the request is accepted in both the
 *          active and standby states. */
bool camera_request_resolution(uint8_t framesize);

#endif /* __CAMERA_SERVICE_H__ */
