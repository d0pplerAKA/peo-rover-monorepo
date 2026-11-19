#ifndef __ESP_NET_H__
#define __ESP_NET_H__

/**
 * @file  esp_net.h
 * @brief WiFi STA + TCP 8888 command/telemetry + TCP 8890 video server.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

/*
 * esp_net_task (priority 5): connects to the configured WiFi hotspot in STA
 * mode, then serves TCP 8888 (single client) through select() multiplexing:
 *   - Complete JSON lines are pushed into esp_net_rx_queue together with their
 *     local generation, client slot, receive time and byte-budget info;
 *   - ACKs travel on a dedicated queue and are broadcast with priority over
 *     telemetry (a latest-value mailbox);
 *   - Client disconnect -> automatically sends STOP_ALL (the PROTOCOL
 *     disconnect-protection hard rule).
 * TCP 8890: video stream (camera enqueues frames zero-copy; the esp_net task
 * sends and returns them). Framing = 8-byte big-endian header (magic+fid+len)
 * + JPEG body (TCP itself guarantees integrity).
 *
 * The RX/ACK queues are initialized by app_main before task creation; telemetry
 * uses a static atomic mailbox. The telemetry task peeks a snapshot every 50 ms
 * (20 Hz); the network task broadcasts ACKs first, then telemetry.
 * The video channel is one-way (S3 -> upper computer); it is cleaned up on
 * disconnect, and the upper computer auto-reconnects only while video is on.
 * Oversized/NUL-containing lines, RX OOM/queue-full/over-budget, and slow TX
 * clients all cause a disconnect and a latched STOP.
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_camera.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

/* ============================================================================
   Network parameters (PROTOCOL.md section 5)
   ============================================================================ */
/* Laptop-hotspot STA mode (user decision 2026-11-16: no more S3 soft-AP).
   SSID/password = the user's laptop mobile-hotspot name and password; they
   must be filled in for the actual deployment before flashing. */
#define PEO_NET_STA_SSID        "Y9000X"
#define PEO_NET_STA_PASS        "88888888"
#define PEO_NET_MDNS_HOSTNAME   "peo_rover"      /* Upper computer resolves peo_rover.local. */
#define PEO_NET_TCP_PORT        8888
#define PEO_NET_VIDEO_PORT      8890    /* TCP video stream (decided 2026-11-16; UDP 8889 retired). */
#define PEO_NET_MAX_CLIENTS     1      /* 1:1 mode: a single client exclusively (the TCP accept
                                           layer rejects a second connection; in STA mode only the
                                           laptop hotspot is the AP, other devices cannot reach us anyway). */
#define PEO_NET_HEARTBEAT_TIMEOUT_MS 2000   /* For a client that has sent control commands: timeout
                                               without a valid command / one-way safety heartbeat. */
#define PEO_NET_MAX_LINE        16384   /* JSON bytes excluding newline. */
#define PEO_NET_RX_BUDGET_BYTES 65536   /* Total byte budget for complete RX lines, excluding newline/terminator. */
#define PEO_NET_ACK_QUEUE_DEPTH 8       /* ACK queue depth. */
#define PEO_NET_VIDEO_MAGIC     0xCAFE

typedef struct {
    char *line;                /* Owned by receiver; free exactly once. */
    uint32_t generation;       /* Safety generation when the line began arriving. */
    uint8_t slot;              /* Local TCP client slot; not part of the outgoing JSON. */
    uint32_t received_ms;      /* FreeRTOS monotonic ms when the newline of the complete line arrived. */
    uint32_t line_bytes;       /* String byte count of the line, excluding newline/terminator; used to return budget. */
} esp_net_rx_message_t;
#include <freertos/event_groups.h>

extern QueueHandle_t esp_net_rx_queue;  /* esp_net_rx_message_t */
extern QueueHandle_t esp_net_ack_queue; /* char* JSON line: ACK-only, broadcast with priority then freed */

/** @brief  Take over a malloc'd/free-compatible complete JSON string, replacing any
 *  unconsumed older frame (which is freed).
 *  @param json  JSON string to publish (ownership transfers).
 *  @retval true if taken over (NOT a delivery guarantee); false if NULL (mailbox unchanged).
 *  After the call the producer must not access/free json; never publish the same
 *  pointer twice. Task context only. */
bool esp_net_publish_telemetry(char *json);

/** @brief  Take over an ACK string and wake the network task after a successful enqueue.
 *  @param json  ACK string (ownership transfers even on failure).
 *  @retval true only if enqueued. Non-blocking; keeps the drop-on-full policy. */
bool esp_net_send_ack(char *json);

/** @brief  Wake select(). Task context, outside application locks; no-op when disabled/init failed. */
void esp_net_wake(void);

/** @brief  Create the queues (called in app_main before task creation). */
void esp_net_queues_init(void);

void esp_net_task(void *arg);

/** @brief  Cross-core query: is any TCP client connected?
 *  @retval true if a connection slot is occupied (checked under the lock). */
bool esp_net_has_client(void);

/* Connection event group (maintained by the esp_net task): camera/telemetry use
   it for "no client -> stop" gating, ws2812 uses it for the status LED.
   PEO_NET_EVT_CLIENT = a client is online (set on accept / cleared when all disconnect);
   PEO_NET_EVT_VIDEO  = video is enabled (set/cleared by esp_net_video_set_enabled);
   PEO_NET_EVT_STA    = WiFi is connected to the laptop hotspot (set on STA_CONNECTED / cleared on DISCONNECTED). */
#define PEO_NET_EVT_CLIENT    (1u << 0)
#define PEO_NET_EVT_VIDEO     (1u << 1)
#define PEO_NET_EVT_STA       (1u << 2)
EventGroupHandle_t esp_net_conn_event(void);

/** @brief  Renew client activity from the parser side.
 *  @param slot         Client slot.
 *  @param received_ms  Receive time of the line.
 *  @param commanded    true = this client submitted MOTOR/MOTOR_CLOSED/PID (a control command). */
void esp_net_note_client_activity(uint8_t slot, uint32_t received_ms, bool commanded);

/** @brief  Cross-core byte budget for complete RX lines. After a successful reserve,
 *  release from the dequeuing consumer or the enqueue-failure path.
 *  @param bytes  Line byte count.
 *  @retval true if the bytes were reserved. */
bool esp_net_rx_reserve(uint32_t bytes);
void esp_net_rx_release(uint32_t bytes);

/* Video global switch: off by default at power-on, and auto-off when all TCP
 * clients disconnect. */
void esp_net_video_set_enabled(bool enabled);
bool esp_net_video_enabled(void);

/** @brief  camera (single producer) registers the original frame descriptor before send_video;
 *  registering does NOT transfer ownership. */
void esp_net_video_set_frame_owner(camera_fb_t *fb);

/** @brief  Asynchronously enqueue a JPEG frame.
 *  @param jpeg  JPEG bytes.
 *  @param len   Length in bytes.
 *  @retval true: ownership transferred and the network task was notified;
 *  @retval false: the caller must return the frame itself.
 *  You must register the original camera_fb_t that owns buf first; never cast buf
 *  to a frame descriptor. */
bool esp_net_send_video(const uint8_t *jpeg, uint32_t len);

/** @brief  Drop pending frames; if a frame is mid-flight, close the video connection and
 *  return it, so a half-sent byte stream is never reused.
 *  The camera (single producer) must call this before deinit/exit; the connection is
 *  kept when only a pending (not in-flight) frame exists. */
void esp_net_video_flush(void);

#endif /* __ESP_NET_H__ */
