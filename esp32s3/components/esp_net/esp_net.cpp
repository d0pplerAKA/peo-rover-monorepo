/**
 * @file  esp_net.cpp
 * @brief WiFi STA + TCP 8888 command/telemetry + TCP 8890 video stream implementation.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

/*
 * A single task (select() multiplexing) manages everything network-related:
 *   - STA setup (esp_netif + esp_wifi), connecting to the laptop hotspot;
 *   - TCP listen + one client (each client has its own line buffer);
 *   - JSON line RX -> rx_queue; ACK broadcast with priority over telemetry;
 *     disconnect -> STOP_ALL;
 *   - the camera task enqueues frames zero-copy; this task does the TCP framing
 *     (8-byte header + JPEG) send and the frame return.
 */

#ifndef PEO_NET_USE_EVENTFD
#define PEO_NET_USE_EVENTFD 1
#endif

#include "esp_net.h"
#include "esp_link.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include <esp_camera.h>
#include <stdio.h>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_wifi.h>
#include <mdns.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_timer.h>
#include <fcntl.h>
#include <errno.h>
#include <lwip/sockets.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <atomic>
#include <unistd.h>
#if PEO_NET_USE_EVENTFD
#include <esp_vfs_eventfd.h>
#endif

static const char *TAG = "esp_net";

QueueHandle_t esp_net_rx_queue = NULL;
QueueHandle_t esp_net_ack_queue = NULL;

/* Latest unpublished telemetry; exchange transfers exclusive ownership (seq_cst).
   Lifetime counters: overwrite / client backlog / append allocation failure.
   Do not reset on disconnect: the producer can update them on the other core. */
static std::atomic<char *> s_tel_latest{nullptr};
static std::atomic<uint32_t> s_tel_drop_overwrite{0};
static std::atomic<uint32_t> s_tel_drop_busy{0};
static std::atomic<uint32_t> s_tel_drop_append_fail{0};

/* Initialized once by app_main before producers start; retained for process lifetime.
   Never close/reuse this fd during client cleanup while producers can write it. */
static int s_wake_fd = -1;

static void init_net_wake(void)
{
#if PEO_NET_USE_EVENTFD
    static bool attempted = false;  /* queues_init is serialized before task creation. */
    if (attempted) return;
    attempted = true;
    esp_vfs_eventfd_config_t config = {};
    config.max_fds = 1;
    const esp_err_t err = esp_vfs_eventfd_register(&config);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "eventfd register failed: %s; using legacy polling", esp_err_to_name(err));
        return;
    }
    const int fd = eventfd(0, 0);  /* Task context only; EFD_NONBLOCK is not supported. */
    if (fd < 0)
    {
        ESP_LOGE(TAG, "eventfd create failed: %d; using legacy polling", errno);
        return;
    }
    /* VFS fds are globally unique (sockets also use the VFS), so there is no
       overlap with lwip socket numbers; the only hard limit is the select bitmap. */
    if (fd >= FD_SETSIZE)
    {
        ESP_LOGE(TAG, "eventfd fd=%d exceeds FD_SETSIZE=%d; using legacy polling", fd, FD_SETSIZE);
        close(fd);
        return;
    }
    s_wake_fd = fd;
#endif
}

void esp_net_wake(void)
{
#if PEO_NET_USE_EVENTFD
    if (s_wake_fd >= 0)
    {
        const uint64_t one = 1;
        (void)write(s_wake_fd, &one, sizeof(one));
    }
#endif
}

bool esp_net_send_ack(char *json)
{
    if (!json) return false;
    if (!esp_net_ack_queue || xQueueSend(esp_net_ack_queue, &json, 0) != pdTRUE)
    {
        free(json);  /* Preserve the existing nonblocking, drop-on-full policy. */
        return false;
    }
    esp_net_wake();  /* Queue publication precedes the wake; no application lock held. */
    return true;
}

bool esp_net_publish_telemetry(char *json)
{
    if (!json) return false;
    char *old = s_tel_latest.exchange(json);
    if (old) s_tel_drop_overwrite.fetch_add(1);
    free(old);
    esp_net_wake();
    return true;  /* Ownership transferred, not a delivery acknowledgement. */
}

static uint32_t s_rx_budget = 0;
static portMUX_TYPE s_rx_budget_lock = portMUX_INITIALIZER_UNLOCKED;

bool esp_net_rx_reserve(uint32_t bytes)
{
    portENTER_CRITICAL(&s_rx_budget_lock);
    /* Compare against remaining budget so a very large bytes cannot overflow the sum. */
    const bool allowed = bytes <= PEO_NET_RX_BUDGET_BYTES - s_rx_budget;
    if (allowed) s_rx_budget += bytes;
    portEXIT_CRITICAL(&s_rx_budget_lock);
    return allowed;
}

void esp_net_rx_release(uint32_t bytes)
{
    portENTER_CRITICAL(&s_rx_budget_lock);
    s_rx_budget = bytes <= s_rx_budget ? s_rx_budget - bytes : 0;
    portEXIT_CRITICAL(&s_rx_budget_lock);
}

/* Network-task supervision, renewed cross-core by the parser; stores only local
   activity info and never alters the JSON. */
static struct {
    uint32_t last_cmd_ms;
    bool has_commanded;
    bool stop_latched;          /* One activity timeout latches STOP only once. */
    int fd = -1;                /* Network task keeps a connection mirror for cross-core queries; an empty slot must be -1. */
} s_client_activity[PEO_NET_MAX_CLIENTS];
static portMUX_TYPE s_activity_lock = portMUX_INITIALIZER_UNLOCKED;

void esp_net_note_client_activity(uint8_t slot, uint32_t received_ms, bool commanded)
{
    if (slot >= PEO_NET_MAX_CLIENTS) return;
    portENTER_CRITICAL(&s_activity_lock);
    s_client_activity[slot].last_cmd_ms = received_ms;
    s_client_activity[slot].stop_latched = false;
    if (commanded) s_client_activity[slot].has_commanded = true;
    portEXIT_CRITICAL(&s_activity_lock);
}

bool esp_net_has_client(void)
{
    bool have_client = false;
    portENTER_CRITICAL(&s_activity_lock);
    for (int i = 0; i < PEO_NET_MAX_CLIENTS; ++i)
        have_client |= s_client_activity[i].fd >= 0;
    portEXIT_CRITICAL(&s_activity_lock);
    return have_client;
}

static EventGroupHandle_t s_conn_eg = NULL; /* CLIENT/VIDEO event group (camera/telemetry gating). */

EventGroupHandle_t esp_net_conn_event(void)
{
    return s_conn_eg;
}

static int s_video_listen_fd = -1;   /* TCP video listen socket (8890). */
static int s_video_fd       = -1;    /* Accepted video client. */
static portMUX_TYPE s_video_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_video_enabled = false;
static uint16_t s_frame_id = 0;     /* Updated only when the esp_net task sends. */
struct video_frame_t {
    const uint8_t *buf;
    size_t len;
    camera_fb_t *owner;        /* fb_return needs the original descriptor, never the image data address. */
};
static video_frame_t s_video_pending = {};   /* 1-deep; returns the old frame when overwritten. */
static camera_fb_t *s_video_producer_frame = NULL; /* Staged registration; still owned by camera until enqueue succeeds. */
static SemaphoreHandle_t s_video_queue_mutex = NULL;
static SemaphoreHandle_t s_video_send_mutex = NULL; /* flush waits for the in-flight frame to be returned before deinit. */
static TaskHandle_t s_net_task = NULL;
static const uint32_t VIDEO_NOTIFY_BIT = 1u;

/* ============================================================================
   1. WiFi STA init (laptop hotspot, user decision 2026-11-16)
   ============================================================================ */

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
        ESP_LOGI(TAG, "wifi sta: started, connecting...");
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI(TAG, "wifi sta: connected");
        if (s_conn_eg)
            xEventGroupSetBits(s_conn_eg, PEO_NET_EVT_STA);
    }
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {
        esp_link_request_stop();   /* WiFi down = link lost: latch STOP immediately (user decision 2026-11-17). */
        ESP_LOGW(TAG, "wifi sta: disconnected; reconnecting...");
        if (s_conn_eg)
            xEventGroupClearBits(s_conn_eg, PEO_NET_EVT_STA);
        esp_wifi_connect();   /* Auto-reconnect: each disconnect event triggers one connect attempt
                                (a failure re-fires this event, naturally throttling). */
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "wifi sta: got IP " IPSTR " (mDNS: %s.local)",
                 IP2STR(&ev->ip_info.ip), PEO_NET_MDNS_HOSTNAME);
    }
}

/* STA mode: better throughput/stability than softAP (per Espressif measurements),
   and the laptop has a stronger RF front end; auto-reconnect + power-save off
   (remote-control latency first) + mDNS (the hotspot DHCP address can change). */
static void wifi_init_sta(void)
{
    esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                     wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                     wifi_event_handler, NULL, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wcfg;
    memset(&wcfg, 0, sizeof(wcfg));
    strlcpy((char *)wcfg.sta.ssid, PEO_NET_STA_SSID, sizeof(wcfg.sta.ssid));
    strlcpy((char *)wcfg.sta.password, PEO_NET_STA_PASS, sizeof(wcfg.sta.password));
    wcfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_ERROR_CHECK(esp_wifi_connect());

    /* mDNS: the upper computer connects via peo_rover.local, independent of the DHCP-assigned IP. */
    if (mdns_init() == ESP_OK)
    {
        mdns_hostname_set(PEO_NET_MDNS_HOSTNAME);
        mdns_instance_name_set("PEO Rover ESP32-S3");
        ESP_LOGI(TAG, "mDNS up: %s.local", PEO_NET_MDNS_HOSTNAME);
    }
    else
        ESP_LOGW(TAG, "mDNS init failed; connect by IP only");
}

/* ============================================================================
   2. Disconnect protection uses the independent STOP latch, not a bounded queue.
   ============================================================================ */
static void post_disconnect(void)
{
    esp_link_request_stop();
    /* The engine observes the latch/generation even when its queue is full. */
}

/* ============================================================================
   3. TCP video enqueue and send (camera single producer, esp_net single consumer)
   ============================================================================ */
void esp_net_video_set_enabled(bool enabled)
{
    portENTER_CRITICAL(&s_video_lock);
    s_video_enabled = enabled;
    portEXIT_CRITICAL(&s_video_lock);
    if (s_conn_eg)
    {
        if (enabled)
            xEventGroupSetBits(s_conn_eg, PEO_NET_EVT_VIDEO);
        else
            xEventGroupClearBits(s_conn_eg, PEO_NET_EVT_VIDEO);
    }
    esp_net_wake();
}

bool esp_net_video_enabled(void)
{
    portENTER_CRITICAL(&s_video_lock);
    bool enabled = s_video_enabled;
    portEXIT_CRITICAL(&s_video_lock);
    return enabled;
}

void esp_net_video_set_frame_owner(camera_fb_t *fb)
{
    if (!s_video_queue_mutex) return;
    xSemaphoreTake(s_video_queue_mutex, portMAX_DELAY);
    s_video_producer_frame = fb;
    xSemaphoreGive(s_video_queue_mutex);
}

bool esp_net_send_video(const uint8_t *jpeg, uint32_t len)
{
    if (!s_video_queue_mutex) return false;
    xSemaphoreTake(s_video_queue_mutex, portMAX_DELAY);
    camera_fb_t *owner = s_video_producer_frame;
    s_video_producer_frame = NULL;
    portENTER_CRITICAL(&s_video_lock);
    bool available = s_video_enabled && s_video_fd >= 0;
    portEXIT_CRITICAL(&s_video_lock);
    if (!available || !s_net_task || !owner || owner->buf != jpeg ||
        !jpeg || len == 0 || len > owner->len || len > 256U * 1024U) {
        xSemaphoreGive(s_video_queue_mutex);
        return false;  /* Did not take over this frame; the caller returns it. */
    }
    if (s_video_pending.owner) esp_camera_fb_return(s_video_pending.owner);
    s_video_pending = {jpeg, len, owner};
    /* Enqueue and notify happen in the same critical section; the consumer always trusts the queue contents. */
    xTaskNotify(s_net_task, VIDEO_NOTIFY_BIT, eSetBits);
    xSemaphoreGive(s_video_queue_mutex);
    esp_net_wake();
    return true;
}

/* Video congestion cooldown: EAGAIN from send means the send buffer is full
   (air interface / peer cannot keep up). An in-flight frame must still be fully
   sent (TCP framing cannot be interrupted), but no NEW frame starts during the
   cooldown — the 1-deep queue only sends the latest frame anyway, so the link
   drains; once cooldown ends a new frame starts and a success resumes the stream.
   At 15 fps a 50 ms cooldown costs about 1 frame (depending on enqueue phase);
   it only blocks new frames, the in-flight frame keeps advancing within budget. */
#define VIDEO_COOLDOWN_US 50000
#define VIDEO_PUMP_MAX_SENDS 3   /* Per-round send-call budget for pump; the header and JPEG body share it. */
#ifndef VIDEO_PUMP_MAX_BYTES
#define VIDEO_PUMP_MAX_BYTES 8192
#endif
static_assert(VIDEO_PUMP_MAX_BYTES > 0, "video byte budget must be positive");

static int64_t s_video_cool_until_us = 0;   /* Cooldown deadline (see VIDEO_COOLDOWN_US). */
static uint32_t s_tx_fail_cnt = 0;          /* Cumulative video send failures (diagnostic; reset on disconnect). */
static int s_last_errno = 0;

/* TCP video in-flight frame: once a TCP byte stream starts it must be fully
   sent (otherwise the peer's framing desynchronizes). Framing = 8-byte header
   (magic+fid+len, big-endian) + JPEG body; each select round advances within
   budget using non-blocking send, never blocking the main task (the command /
   telemetry channels stay responsive). */
struct video_inflight_t {
    camera_fb_t *owner;
    const uint8_t *buf;
    size_t len, off;
    uint8_t hdr[8];            /* magic(2)+fid(2)+len(4), big-endian. */
    size_t hdr_off;
    bool hdr_done;
};
/* Send diagnostics (Codex review R-1: make disconnect root causes distinguishable) —
   per-channel (video/telemetry) EAGAIN counts and the largest observed backlog;
   drop_client prints the scene. Defined before pump_video/flush_tx use them
   (pump_video appears earlier in this file). */
static uint32_t s_video_eagain_cnt = 0;
static uint32_t s_telemetry_eagain_cnt = 0;
static size_t   s_max_tx_pending = 0;

/* Periodic stats window (1 s, printed then zeroed; the EAGAIN counters stay
   cumulative for diagnostics) — camera-side stats were removed entirely, so the
   video/telemetry send info is now collected here (network application layer). */
static uint32_t s_stat_v_frames = 0;   /* Frames fully sent in this window. */
static uint64_t s_stat_v_bytes  = 0;   /* Video bytes in this window. */
static uint64_t s_stat_t_bytes  = 0;   /* Command/telemetry bytes in this window. */
static uint32_t s_stat_eagain_v_snap = 0;  /* EAGAIN cumulative snapshot: window delta = new this window. */
static uint32_t s_stat_eagain_t_snap = 0;
static int64_t  s_stat_last_us  = 0;
static uint32_t s_stat_tel_overwrite_snap = 0;
static uint32_t s_stat_tel_busy_snap = 0;
static uint32_t s_stat_tel_alloc_snap = 0;
struct select_stats_t {
    uint32_t calls, timeout, eventfd, readable, writable, error, backoff;
};
/* Readable excludes eventfd. Reasons can overlap within one select result. */
static select_stats_t s_stat_select = {};

static video_inflight_t s_video_inflight = {};

/* Each round limits both the send-call count and the bytes successfully sent
   (header included); when the budget is spent, the rest waits for the next select.
   Returns false = connection error (peer closed); true = not done or done. */
static bool pump_video(size_t &bytes_sent)
{
    bytes_sent = 0;
    int fd = s_video_fd;
    if (fd < 0 || !s_video_inflight.owner) return true;
    video_inflight_t &f = s_video_inflight;
    unsigned sends = 0;
    if (!f.hdr_done)
    {
        ++sends;
        const size_t remaining = sizeof(f.hdr) - f.hdr_off;
        const size_t available = VIDEO_PUMP_MAX_BYTES - bytes_sent;
        const size_t chunk = remaining < available ? remaining : available;
        int n = send(fd, f.hdr + f.hdr_off, chunk, 0);
        if (n < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            {
                s_video_eagain_cnt++;
                s_video_cool_until_us = esp_timer_get_time() + VIDEO_COOLDOWN_US;
                return true;   /* Buffer full: continue next round (the in-flight frame must finish). */
            }
            return false;
        }
        if (n == 0) return true;  /* No progress: consume no byte budget, retry next round. */
        bytes_sent += (size_t)n;
        f.hdr_off += (size_t)n;
        if (f.hdr_off < sizeof(f.hdr)) return true;
        f.hdr_done = true;
    }
    while (f.off < f.len)
    {
        if (sends >= VIDEO_PUMP_MAX_SENDS || bytes_sent >= VIDEO_PUMP_MAX_BYTES) return true;
        ++sends;
        const size_t remaining = f.len - f.off;
        const size_t available = VIDEO_PUMP_MAX_BYTES - bytes_sent;
        const size_t chunk = remaining < available ? remaining : available;
        int n = send(fd, f.buf + f.off, chunk, 0);
        if (n < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
            {
                s_video_eagain_cnt++;
                s_video_cool_until_us = esp_timer_get_time() + VIDEO_COOLDOWN_US;
                return true;
            }
            /* Diagnostic: throttled to once per second. */
            static int64_t s_last_tx_fail_log_us = 0;
            s_tx_fail_cnt++;
            s_last_errno = errno;
            const int64_t now_us = esp_timer_get_time();
            if (now_us - s_last_tx_fail_log_us > 1000000LL)
            {
                ESP_LOGW(TAG, "video send fail: errno=%d (%s), fail_cnt=%lu",
                         s_last_errno, strerror(s_last_errno),
                         (unsigned long)s_tx_fail_cnt);
                s_last_tx_fail_log_us = now_us;
            }
            return false;
        }
        if (n == 0) return true;
        bytes_sent += (size_t)n;
        f.off += (size_t)n;
    }
    s_stat_v_frames++;
    s_stat_v_bytes += f.len;
    esp_camera_fb_return(f.owner);
    s_video_inflight = {};
    return true;
}

/* Start a new frame: fill the 8-byte header (magic+fid+len), then pump_video sends it round by round within budget. */
static void start_video_frame(const video_frame_t &frame)
{
    const uint32_t len = (uint32_t)frame.len;
    s_video_inflight.owner = frame.owner;
    s_video_inflight.buf = frame.buf;
    s_video_inflight.len = len;
    s_video_inflight.off = 0;
    s_video_inflight.hdr_off = 0;
    s_video_inflight.hdr_done = false;
    const uint16_t fid = ++s_frame_id;
    s_video_inflight.hdr[0] = (uint8_t)(PEO_NET_VIDEO_MAGIC >> 8);
    s_video_inflight.hdr[1] = (uint8_t)(PEO_NET_VIDEO_MAGIC & 0xFF);
    s_video_inflight.hdr[2] = (uint8_t)(fid >> 8);
    s_video_inflight.hdr[3] = (uint8_t)(fid & 0xFF);
    s_video_inflight.hdr[4] = (uint8_t)(len >> 24);
    s_video_inflight.hdr[5] = (uint8_t)(len >> 16);
    s_video_inflight.hdr[6] = (uint8_t)(len >> 8);
    s_video_inflight.hdr[7] = (uint8_t)(len & 0xFF);
}

void esp_net_video_flush(void)
{
    if (!s_video_send_mutex || !s_video_queue_mutex) return;
    /* Never wait on network sends inside a spinlock; always take send_mutex before queue_mutex to avoid lock-order inversion. */
    xSemaphoreTake(s_video_send_mutex, portMAX_DELAY);
    xSemaphoreTake(s_video_queue_mutex, portMAX_DELAY);
    if (s_video_pending.owner) esp_camera_fb_return(s_video_pending.owner);
    s_video_pending = {};
    if (s_video_inflight.owner)     /* When invalidating an in-flight frame, reset the connection;
                                       a possibly half-frame byte stream cannot be reused. */
    {
        if (s_video_fd >= 0) close(s_video_fd);
        s_video_fd = -1;
        esp_camera_fb_return(s_video_inflight.owner);
        s_video_inflight = {};
        ESP_LOGW(TAG, "video connection reset (half-frame discarded)");
    }
    xSemaphoreGive(s_video_queue_mutex);
    xSemaphoreGive(s_video_send_mutex);
    esp_net_wake();  /* camera may have closed the fd while select was waiting. */
}

static bool video_pending(void)
{
    xSemaphoreTake(s_video_queue_mutex, portMAX_DELAY);
    bool pending = s_video_pending.buf != NULL;
    xSemaphoreGive(s_video_queue_mutex);
    return pending;
}

/* True means bytes sent or frame/connection state advanced; cooldown renewal alone
   is not progress. Used only by the event-driven loop's no-progress backoff. */
static bool send_pending_video(void)
{
    xSemaphoreTake(s_video_send_mutex, portMAX_DELAY);
    size_t bytes_sent = 0;
    bool state_changed = false;
    /* 1) Continue the in-flight frame within the per-round dual budget;
          cooldown does not stop an in-flight frame from advancing. */
    if (!pump_video(bytes_sent))
    {
        state_changed = true;
        esp_link_request_stop();   /* Video send hard error = video link lost: latch STOP immediately (user decision 2026-11-17). */
        ESP_LOGW(TAG, "video socket error; closing video client");
        esp_camera_fb_return(s_video_inflight.owner);
        s_video_inflight = {};
        close(s_video_fd);
        s_video_fd = -1;
    }
    if (s_video_inflight.owner)
    {
        xSemaphoreGive(s_video_send_mutex);
        return bytes_sent != 0 || state_changed;   /* In-flight frame not finished: do not start a new one. */
    }
    /* 2) Take the pending frame to start a new one (no client / in cooldown ->
          dropped; the 1-deep queue only sends the latest frame). */
    xSemaphoreTake(s_video_queue_mutex, portMAX_DELAY);
    video_frame_t frame = s_video_pending;
    s_video_pending = {};
    xSemaphoreGive(s_video_queue_mutex);
    if (frame.owner)
    {
        state_changed = true;
        if (s_video_fd >= 0 && esp_timer_get_time() >= s_video_cool_until_us)
            start_video_frame(frame);
        else
            esp_camera_fb_return(frame.owner);
    }
    xSemaphoreGive(s_video_send_mutex);
    return bytes_sent != 0 || state_changed;
}

/* ============================================================================
   4. Main task
   ============================================================================ */
void esp_net_queues_init(void)
{
    if (!esp_net_rx_queue) esp_net_rx_queue = xQueueCreate(32, sizeof(esp_net_rx_message_t));
    if (!esp_net_ack_queue) esp_net_ack_queue = xQueueCreate(PEO_NET_ACK_QUEUE_DEPTH, sizeof(char *));
    if (!s_video_queue_mutex) s_video_queue_mutex = xSemaphoreCreateMutex();
    if (!s_video_send_mutex) s_video_send_mutex = xSemaphoreCreateMutex();
    if (!s_conn_eg) s_conn_eg = xEventGroupCreate();
    configASSERT(esp_net_rx_queue && esp_net_ack_queue &&
                 s_video_queue_mutex && s_video_send_mutex && s_conn_eg);
    init_net_wake();  /* Nonfatal failure leaves s_wake_fd == -1 and legacy polling. */
}

/* One bounded pending byte stream per client; never discard a partial JSON line. */
#define NET_TX_LIMIT 32768        /* Per-client pending-send backlog cap: leaves a drain window for
                                       burst air-interface stalls of the 20 Hz telemetry. */
#define NET_TX_TIMEOUT_MS 5000   /* 1000 was too aggressive: a 100-300 ms WiFi jitter / Windows
                                       background scan pause would already disconnect. */
#define NET_TX_DRAIN_BATCH_MAX 8 /* Max ACK lines dequeued per round, so RX/STOP never starve. */

/* append_tx failure reason: distinguishes "backlog over limit" from "realloc
   OOM" — both used to share one "tx backlog limit" log, hiding the root cause. */
typedef enum {
    TX_APPEND_OK = 0,
    TX_APPEND_BACKLOG,   /* Backlog exceeds NET_TX_LIMIT. */
    TX_APPEND_ALLOC      /* realloc failed (out of memory). */
} tx_append_result_t;
struct net_client_t {
    int fd;
    char *buf;
    size_t len;
    uint32_t generation;
    char *tx;
    size_t tx_len, tx_off;
    TickType_t tx_since;
};

static bool nonblocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    return flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

static bool selectable_socket(int fd, const char *role)
{
    if (fd < 0) return false;
    if (fd < LWIP_SOCKET_OFFSET || fd >= FD_SETSIZE || fd == s_wake_fd)
    {
        ESP_LOGE(TAG, "%s fd=%d outside socket range [%d,%d), wake=%d",
                 role, fd, LWIP_SOCKET_OFFSET, FD_SETSIZE, s_wake_fd);
        errno = EINVAL;
        return false;
    }
    return true;
}

/* Full network reset after disconnect (user request: return all memory, back to
   the "no client" initial state): clear the telemetry mailbox / ACK broadcast
   queue, free the client lines still left in the RX queue (lines cmd_parser is
   still consuming are unaffected — queue ops are atomic, this only drains what
   remains), return unsent video frames, reset cooldown/frame-id/fail counters. */
static void esp_net_cleanup_all(void)
{
    char *out;
    free(s_tel_latest.exchange(nullptr));
    while (xQueueReceive(esp_net_ack_queue, &out, 0) == pdTRUE) free(out);
    esp_net_rx_message_t msg;
    while (xQueueReceive(esp_net_rx_queue, &msg, 0) == pdTRUE)
    {
        esp_net_rx_release(msg.line_bytes);
        free(msg.line);
    }
    esp_net_video_flush();
    if (s_video_fd >= 0)
    {
        close(s_video_fd);
        s_video_fd = -1;
    }
    s_video_cool_until_us = 0;
    s_frame_id = 0;
    s_tx_fail_cnt = 0;
    s_last_errno = 0;
}

static void drop_client(net_client_t *clients, net_client_t &c, int slot, const char *why,
                        bool stop_already_requested = false)
{
    if (c.fd < 0) return;
    ESP_LOGW(TAG, "drop client %d: %s", slot, why ? why : "?");
    /* Latch STOP (safe stop) BEFORE reclaiming resources: invalidate the old
       command generation the moment the link is lost, instead of being delayed
       by close/free/cleanup/video_flush (Codex review R-1, severe). The heartbeat
       branch already requested STOP, so stop_already_requested avoids a duplicate
       generation bump here. */
    if (!stop_already_requested) post_disconnect();
    /* Scene diagnostics: pending / EAGAIN counts / max backlog / heap headroom,
       to tell "backlog over limit" from "OOM". */
    ESP_LOGW(TAG, "  diag: pending=%u limit=%u video_eagain=%lu tx_eagain=%lu "
                  "max_pending=%u free_heap=%u largest=%u",
             (unsigned)(c.tx_len - c.tx_off), (unsigned)NET_TX_LIMIT,
             (unsigned long)s_video_eagain_cnt, (unsigned long)s_telemetry_eagain_cnt,
             (unsigned)s_max_tx_pending,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    close(c.fd);
    free(c.buf);
    free(c.tx);
    c = {};
    c.fd = -1;
    portENTER_CRITICAL(&s_activity_lock);
    s_client_activity[slot] = {};
    portEXIT_CRITICAL(&s_activity_lock);
    bool have_client = false;
    for (int i = 0; i < PEO_NET_MAX_CLIENTS; ++i)
        have_client |= clients[i].fd >= 0;
    if (!have_client)
    {
        esp_net_video_set_enabled(false);   /* Also clears the VIDEO event bit. */
        if (s_conn_eg)
            xEventGroupClearBits(s_conn_eg, PEO_NET_EVT_CLIENT);
        esp_net_cleanup_all();              /* Full network reset to the initial state. */
    }
    esp_net_video_flush();
}

static tx_append_result_t append_tx(net_client_t &c, const char *out, size_t len)
{
    size_t pending = c.tx_len - c.tx_off;
    bool newline = len == 0 || out[len - 1] != '\n';
    if (len + newline > NET_TX_LIMIT - pending) return TX_APPEND_BACKLOG;
    if (c.tx_off && pending) memmove(c.tx, c.tx + c.tx_off, pending);
    c.tx_len = pending;
    c.tx_off = 0;
    char *tx = (char *)realloc(c.tx, pending + len + newline);
    if (!tx) return TX_APPEND_ALLOC;
    c.tx = tx;
    memcpy(c.tx + pending, out, len);
    if (newline) c.tx[pending + len] = '\n';
    c.tx_len += len + newline;
    if (c.tx_len - c.tx_off > s_max_tx_pending)
        s_max_tx_pending = c.tx_len - c.tx_off;
    if (!pending) c.tx_since = xTaskGetTickCount();
    return TX_APPEND_OK;
}

static bool flush_tx(net_client_t &c)
{
    if (c.tx_off == c.tx_len) return true;
    if ((TickType_t)(xTaskGetTickCount() - c.tx_since) >= pdMS_TO_TICKS(NET_TX_TIMEOUT_MS))
        return false;
    int n = send(c.fd, c.tx + c.tx_off, c.tx_len - c.tx_off, 0);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) s_telemetry_eagain_cnt++;
        return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
    }
    if (!n) return false;
    c.tx_off += (size_t)n;
    s_stat_t_bytes += (size_t)n;        /* Command/telemetry bytes sent (network-layer stats). */
    c.tx_since = xTaskGetTickCount();   /* Any progress renews the deadline: the timeout
                                            targets "stalled", not "backlog age". */
    if (c.tx_off == c.tx_len) {
        free(c.tx);
        c.tx = NULL;
        c.tx_off = c.tx_len = 0;
    }
    return true;
}

void esp_net_task(void *arg)
{
    xSemaphoreTake(s_video_queue_mutex, portMAX_DELAY);
    s_net_task = xTaskGetCurrentTaskHandle();
    xSemaphoreGive(s_video_queue_mutex);
    /* app_main already created the queues before task creation; this task connects
       to the hotspot in STA mode and runs select(). */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_sta();
    s_stat_last_us = esp_timer_get_time();   /* Stats baseline: the first window starts at task boot. */

    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    int one = 1;
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(PEO_NET_TCP_PORT);
    if (!selectable_socket(listen_fd, "command listener") ||
        setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) != 0 ||
        bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(listen_fd, PEO_NET_MAX_CLIENTS) != 0 || !nonblocking(listen_fd))
    {
        ESP_LOGE(TAG, "TCP startup failed: %d", errno);
        if (listen_fd >= 0) close(listen_fd);
        xSemaphoreTake(s_video_queue_mutex, portMAX_DELAY);
        s_net_task = NULL;
        xSemaphoreGive(s_video_queue_mutex);
        esp_net_video_flush();
        esp_link_request_stop();
        vTaskDelete(NULL);
        return;
    }

    int video_listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    addr.sin_port = htons(PEO_NET_VIDEO_PORT);
    if (!selectable_socket(video_listen_fd, "video listener") ||
        setsockopt(video_listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) != 0 ||
        bind(video_listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(video_listen_fd, 1) != 0 || !nonblocking(video_listen_fd))
    {
        if (video_listen_fd >= 0) close(video_listen_fd);
        video_listen_fd = -1;
        ESP_LOGE(TAG, "video TCP listener startup failed");
    }
    s_video_listen_fd = video_listen_fd;
    ESP_LOGI(TAG, "network fds: wake=%d command_listen=%d video_listen=%d socket_start=%d FD_SETSIZE=%d",
             s_wake_fd, listen_fd, video_listen_fd, LWIP_SOCKET_OFFSET, FD_SETSIZE);

    net_client_t clients[PEO_NET_MAX_CLIENTS] = {};
    for (int i = 0; i < PEO_NET_MAX_CLIENTS; i++) clients[i].fd = -1;
    bool backoff_next = false;
    while (1)
    {
        const bool event_driven = PEO_NET_USE_EVENTFD && s_wake_fd >= 0;
        const bool backoff = event_driven && backoff_next;
        backoff_next = false;
        if (backoff)
        {
            /* A positive select timeout alone cannot yield when an fd stays ready.
               Delay outside every application lock; +1 tick guarantees at least 1ms. */
            vTaskDelay(pdMS_TO_TICKS(1) + 1);
            ++s_stat_select.backoff;
        }
        fd_set rfds, wfds;
        int maxfd = listen_fd;
        FD_ZERO(&rfds);
        FD_ZERO(&wfds);
        FD_SET(listen_fd, &rfds);
        if (event_driven)
        {
            FD_SET(s_wake_fd, &rfds);  /* Never add eventfd to wfds: it is always writable. */
            if (s_wake_fd > maxfd) maxfd = s_wake_fd;
        }
        for (int i = 0; i < PEO_NET_MAX_CLIENTS; i++)
        {
            if (clients[i].fd >= 0)
            {
                FD_SET(clients[i].fd, &rfds);
                if (event_driven && clients[i].tx_off != clients[i].tx_len)
                    FD_SET(clients[i].fd, &wfds);
                if (clients[i].fd > maxfd) maxfd = clients[i].fd;
            }
        }
        if (s_video_listen_fd >= 0)
        {
            FD_SET(s_video_listen_fd, &rfds);
            if (s_video_listen_fd > maxfd) maxfd = s_video_listen_fd;
        }
        xSemaphoreTake(s_video_send_mutex, portMAX_DELAY);
        const bool inflight = s_video_inflight.owner != NULL;
        const int selected_video_fd = s_video_fd;
        if (selected_video_fd >= 0)
        {
            FD_SET(selected_video_fd, &rfds);
            if (event_driven && inflight) FD_SET(selected_video_fd, &wfds);  /* Even during cooldown. */
            if (selected_video_fd > maxfd) maxfd = selected_video_fd;
        }
        xSemaphoreGive(s_video_send_mutex);  /* During select, camera flush may close the connection. */
        /* Preserve the complete 0/5/20ms, read-only select fallback.
           Notifications still get consumed, but only eventfd can interrupt select. */
        xTaskNotifyWait(0, UINT32_MAX, NULL, 0);
        const bool pending = video_pending();
        const int64_t cool_remaining_us = s_video_cool_until_us - esp_timer_get_time();
        const bool cooling = cool_remaining_us > 0;
        int timeout_us = pending && !cooling ? 0 :
                         (pending || cooling || inflight ? 5000 : 20000);
        if (event_driven)
        {
            timeout_us = 100000;
            if (pending && !cooling) timeout_us = 0;
            else if (pending && cooling)
                timeout_us = (int)(cool_remaining_us > 100000 ? 100000 : cool_remaining_us);
            if (backoff && timeout_us < 1000) timeout_us = 1000;
        }
        struct timeval tv = { 0, timeout_us };
        ++s_stat_select.calls;
        int n = select(maxfd + 1, &rfds, event_driven ? &wfds : NULL, NULL, &tv);
        const int select_errno = n < 0 ? errno : 0;
        if (n == 0) ++s_stat_select.timeout;
        else if (n < 0) ++s_stat_select.error;
        /* Clear the coalesced wake BEFORE draining any producer queues.
           IDF reads return 8 bytes even at zero: never loop until EAGAIN here. */
        if (n > 0 && event_driven && FD_ISSET(s_wake_fd, &rfds))
        {
            uint64_t notifications = 0;
            (void)read(s_wake_fd, &notifications, sizeof(notifications));
            FD_CLR(s_wake_fd, &rfds);
            ++s_stat_select.eventfd;
        }
        if (n > 0)
        {
            bool readable = false, writable = false;
            for (int fd = 0; fd <= maxfd; ++fd)
            {
                readable |= FD_ISSET(fd, &rfds) != 0;
                writable |= event_driven && FD_ISSET(fd, &wfds) != 0;
            }
            if (readable) ++s_stat_select.readable;
            if (writable) ++s_stat_select.writable;
        }
        const bool video_writable = n > 0 && event_driven && selected_video_fd >= 0 &&
                                    FD_ISSET(selected_video_fd, &wfds);
        /* Heartbeat supervision is independent of socket I/O progress; only
           clients that have submitted control commands are supervised.
           Read now inside the lock so a cross-core renewal later than a previous
           sample cannot underflow the age. Logging and STOP stay outside the
           lock to avoid blocking or nesting the safety lock. */
        for (int i = 0; i < PEO_NET_MAX_CLIENTS; ++i)
        {
            if (clients[i].fd < 0) continue;  /* A disconnected slot no longer supervises stale activity. */
            portENTER_CRITICAL(&s_activity_lock);
            const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            const uint32_t age_ms = (uint32_t)(now_ms - s_client_activity[i].last_cmd_ms);
            const bool expired = s_client_activity[i].has_commanded &&
                s_client_activity[i].last_cmd_ms != 0 && age_ms > PEO_NET_HEARTBEAT_TIMEOUT_MS;
            const bool request_stop = expired && !s_client_activity[i].stop_latched;
            if (request_stop) s_client_activity[i].stop_latched = true;
            portEXIT_CRITICAL(&s_activity_lock);
            if (request_stop)
            {
                ESP_LOGW(TAG, "client %d heartbeat timeout (%lu ms); latching STOP",
                         i, (unsigned long)age_ms);
                esp_link_request_stop();
                drop_client(clients, clients[i], i, "heartbeat timeout", true);
            }
        }
        if (n < 0)
        {
            if (select_errno != EINTR) ESP_LOGW(TAG, "select err %d", select_errno);
            send_pending_video();
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (FD_ISSET(listen_fd, &rfds))
        {
            sockaddr_in peer = {};
            socklen_t peer_len = sizeof(peer);
            int cfd = accept(listen_fd, (sockaddr *)&peer, &peer_len);
            if (cfd >= 0)
            {
                int slot = -1;
                for (int i = 0; i < PEO_NET_MAX_CLIENTS; i++)
                    if (clients[i].fd < 0) { slot = i; break; }
                if (slot < 0 || !selectable_socket(cfd, "command client") || !nonblocking(cfd))
                {
                    if (slot < 0)
                        ESP_LOGW(TAG, "reject connection: 1:1 mode, client already connected");
                    close(cfd);
                }
                else
                {
                    if (setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) != 0)
                        ESP_LOGW(TAG, "command TCP_NODELAY failed: %d", errno);
                    const int tos = 0xB8;  /* IP precedence 5 -> ESP32-S3 AC_VI. */
                    if (setsockopt(cfd, IPPROTO_IP, IP_TOS, &tos, sizeof(tos)) != 0)
                        ESP_LOGW(TAG, "command IP_TOS failed: %d", errno);
                    FD_CLR(cfd, &rfds);
                    FD_CLR(cfd, &wfds);  /* A recycled fd must not inherit old select results. */
                    clients[slot].fd = cfd;
                    portENTER_CRITICAL(&s_activity_lock);
                    s_client_activity[slot] = {};  /* A new connection must not inherit the old session's activity/timeout latch. */
                    s_client_activity[slot].fd = cfd;
                    portEXIT_CRITICAL(&s_activity_lock);
                    ESP_LOGI(TAG, "client %d connected: fd=%d wake=%d", slot, cfd, s_wake_fd);
                    /* 8888 command connection established: the on-car NVS PID is authoritative; TX restores it asynchronously. */
                    esp_link_request_pid_push();
                    if (s_conn_eg)
                        xEventGroupSetBits(s_conn_eg, PEO_NET_EVT_CLIENT);
                }
            }
        }
        if (s_video_listen_fd >= 0 && FD_ISSET(s_video_listen_fd, &rfds))
        {
            int vfd = accept(s_video_listen_fd, NULL, NULL);
            if (vfd >= 0)
            {
                if (!selectable_socket(vfd, "video client") || !nonblocking(vfd))
                {
                    close(vfd);
                }
                else
                {
                    if (setsockopt(vfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) != 0)
                        ESP_LOGW(TAG, "video TCP_NODELAY failed: %d", errno);
                    /* ESP-IDF/lwIP does not support SOL_SOCKET/SO_SNDBUF:
                       errno 109 = ENOPROTOOPT, not a peer-close race.
                       The send buffer is set by CONFIG_LWIP_TCP_SND_BUF_DEFAULT;
                       we keep the current value, and partial writes are handled by
                       the in-flight offset plus EAGAIN retries. */
                    xSemaphoreTake(s_video_send_mutex, portMAX_DELAY);
                    if (s_video_fd >= 0)
                    {
                        FD_CLR(s_video_fd, &rfds);
                        FD_CLR(s_video_fd, &wfds);
                        close(s_video_fd);
                    }
                    if (s_video_inflight.owner)   /* New connection takes over: the old in-flight frame is voided. */
                    {
                        esp_camera_fb_return(s_video_inflight.owner);
                        s_video_inflight = {};
                    }
                    s_video_fd = vfd;
                    FD_CLR(vfd, &rfds);  /* The fd may be reused; old select results do not mean the new connection is ready. */
                    FD_CLR(vfd, &wfds);
                    xSemaphoreGive(s_video_send_mutex);
                    ESP_LOGI(TAG, "video client connected: fd=%d wake=%d", vfd, s_wake_fd);
                }
            }
        }
        /* Mutually exclusive with camera flush: check, non-blocking recv, and close must all use the same connection. */
        xSemaphoreTake(s_video_send_mutex, portMAX_DELAY);
        if (selected_video_fd >= 0 && s_video_fd != selected_video_fd)
        {
            FD_CLR(selected_video_fd, &rfds);
            FD_CLR(selected_video_fd, &wfds);
        }
        if (s_video_fd >= 0 && s_video_fd == selected_video_fd && FD_ISSET(s_video_fd, &rfds))
        {
            /* Video channel is one-way (S3 -> upper computer): readable = peer closed/error. */
            char tmp[16];
            int r = recv(s_video_fd, tmp, sizeof(tmp), 0);
            /* EBADF is skipped idempotently: never double-close the fd or double-clear the in-flight frame. */
            if (r <= 0 && !(r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR || errno == EBADF)))
            {
                esp_link_request_stop();   /* Video peer closed = video link lost: latch STOP immediately (user decision 2026-11-17). */
                ESP_LOGW(TAG, "video client closed");
                close(s_video_fd);
                s_video_fd = -1;
                if (s_video_inflight.buf)
                {
                    esp_camera_fb_return(s_video_inflight.owner);
                    s_video_inflight = {};
                }
            }
        }
        xSemaphoreGive(s_video_send_mutex);
        for (int i = 0; i < PEO_NET_MAX_CLIENTS; i++)
        {
            net_client_t &c = clients[i];
            if (c.fd < 0 || !FD_ISSET(c.fd, &rfds)) continue;
            char tmp[256];
            uint32_t received_generation = esp_link_command_generation();
            int r = recv(c.fd, tmp, sizeof(tmp), 0);
            if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) continue;
            if (r <= 0) { drop_client(clients, c, i, r == 0 ? "peer closed" : "recv error"); continue; }
            for (int k = 0; k < r; k++)
            {
                char ch = tmp[k];
                if (c.len == 0) c.generation = received_generation;
                if (ch == '\0' || (ch != '\n' && c.len == PEO_NET_MAX_LINE))
                {
                    drop_client(clients, c, i, "oversized/NUL line");  /* Reject entire oversized/NUL-containing line. */
                    break;
                }
                if (!c.buf)
                {
                    c.buf = (char *)malloc(PEO_NET_MAX_LINE + 1);
                    if (!c.buf) { drop_client(clients, c, i, "line buffer alloc"); break; }
                }
                if (ch == '\n')
                {
                    const uint32_t received_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                    c.buf[c.len] = '\0';
                    size_t len = c.len;
                    c.len = 0;
                    if (len)
                    {
                        /* The complete line is enqueued with its original generation;
                           the parser decides the STOP/one-way-heartbeat exemptions. */
                        esp_net_rx_message_t msg = {strdup(c.buf), c.generation, (uint8_t)i,
                                                    received_ms, (uint32_t)len};
                        if (!msg.line)
                        {
                            drop_client(clients, c, i, "rx line alloc");
                            break;
                        }
                        if (!esp_net_rx_reserve(msg.line_bytes))
                        {
                            free(msg.line);
                            drop_client(clients, c, i, "rx byte budget exceeded");
                            break;
                        }
                        /* Enqueue with a 50 ms wait: when esp_net (prio 5) floods lines
                           in a batch, give cmd_parser (prio 4) time to consume them
                           (batch commands / fast clicks / algorithm command streams all
                           burst multiple lines). Only a consumer truly stuck for 50 ms
                           counts as full -> disconnect. */
                        if (xQueueSend(esp_net_rx_queue, &msg, pdMS_TO_TICKS(50)) != pdTRUE)
                        {
                            esp_net_rx_release(msg.line_bytes);
                            free(msg.line);
                            drop_client(clients, c, i, "rx queue full");
                            break;
                        }
                    }
                }
                else c.buf[c.len++] = ch;
            }
        }
        char *out;
        /* ACKs have priority with a batch cap; then at most one latest telemetry line is consumed. */
        for (int batch = 0; batch < NET_TX_DRAIN_BATCH_MAX &&
             xQueueReceive(esp_net_ack_queue, &out, 0) == pdTRUE; ++batch)
        {
            if (!out) continue;
            size_t len = strlen(out);
            for (int i = 0; i < PEO_NET_MAX_CLIENTS; i++)
            {
                if (clients[i].fd < 0) continue;
                tx_append_result_t r = append_tx(clients[i], out, len);
                if (r == TX_APPEND_BACKLOG)
                    drop_client(clients, clients[i], i, "tx backlog limit");
                else if (r == TX_APPEND_ALLOC)
                    drop_client(clients, clients[i], i, "tx alloc failed (OOM)");
            }
            free(out);
        }
        if (char *json = s_tel_latest.exchange(nullptr))
        {
            const size_t len = strlen(json);
            for (int i = 0; i < PEO_NET_MAX_CLIENTS; i++)
            {
                if (clients[i].fd < 0) continue;
                /* Pending ACK/data alone is not a reason to skip telemetry. */
                tx_append_result_t r = append_tx(clients[i], json, len);
                if (r == TX_APPEND_BACKLOG)
                    s_tel_drop_busy.fetch_add(1);  /* Keep the existing byte stream intact. */
                else if (r == TX_APPEND_ALLOC)
                {
                    s_tel_drop_append_fail.fetch_add(1);
                    drop_client(clients, clients[i], i, "tx alloc failed (OOM)");
                }
            }
            free(json);
        }
        for (int i = 0; i < PEO_NET_MAX_CLIENTS; i++)
            if (clients[i].fd >= 0 && !flush_tx(clients[i])) drop_client(clients, clients[i], i, "tx stall");
        const bool video_progress = send_pending_video();  /* Always pump, regardless of wfds. */
        backoff_next = event_driven && !video_progress && (timeout_us == 0 || video_writable);
        /* A single eventfd read may cover more than one ACK drain batch. */
        if (event_driven && uxQueueMessagesWaiting(esp_net_ack_queue) != 0) esp_net_wake();

        /* Periodic stats (1 s window, single-line in-place refresh): EAGAIN/tel_drop
           use cumulative-snapshot deltas, select counts zero per window; ready
           reasons can overlap and readable excludes eventfd. Other components'
           logs interrupting this line is expected. */
        const int64_t stat_now = esp_timer_get_time();
        if (stat_now - s_stat_last_us >= 1000000LL)
        {
            /* Do not print the dashboard while WiFi is down (all-zero data is
               meaningless and would disturb reconnect-log observation); the window /
               snapshots still advance, and printing resumes next window once connected. */
            const bool sta_up = s_conn_eg &&
                (xEventGroupGetBits(s_conn_eg) & PEO_NET_EVT_STA) != 0;
            const float secs = (float)(stat_now - s_stat_last_us) / 1000000.0f;
            const uint32_t ve = s_video_eagain_cnt - s_stat_eagain_v_snap;
            const uint32_t te = s_telemetry_eagain_cnt - s_stat_eagain_t_snap;
            const uint32_t tel_overwrite = s_tel_drop_overwrite.load();
            const uint32_t tel_busy = s_tel_drop_busy.load();
            const uint32_t tel_alloc = s_tel_drop_append_fail.load();
            /* All fields use fixed-width placeholders: columns stay put when numbers
               grow/shrink (shorter values are covered by trailing spaces), so the line
               width is constant; two trailing spaces are the fallback. teldrop order is
               fixed = overwrite/busy/alloc; select = select calls per second in this
               window (idle ~10/s; hundreds/s means busy-wait, detailed in the error log). */
            const char *last_cmd = esp_link_last_command_name();
            if (sta_up)
            {
            printf("\rnet: video %3lu fps %6.1f KB/s | telemetry %5.1f KB/s | eagain +%4lu +%4lu"
                   " | teldrop +%4lu/+%4lu/+%4lu | select %4lu/s | cmd %-15s  ",
                   (unsigned long)s_stat_v_frames,
                   (double)((float)s_stat_v_bytes / secs / 1024.0f),
                   (double)((float)s_stat_t_bytes / secs / 1024.0f),
                   (unsigned long)ve, (unsigned long)te,
                   (unsigned long)(tel_overwrite - s_stat_tel_overwrite_snap),
                   (unsigned long)(tel_busy - s_stat_tel_busy_snap),
                   (unsigned long)(tel_alloc - s_stat_tel_alloc_snap),
                   (unsigned long)s_stat_select.calls,
                   last_cmd ? last_cmd : "-");
            fflush(stdout);
            /* Busy-wait/error breakdown does not go into the resident line: print one
               diagnostic line when it happens (1 s throttling comes free from the window). */
            if (s_stat_select.backoff != 0 || s_stat_select.error != 0)
            {
                ESP_LOGW(TAG, "select diag: timeout=%lu eventfd=%lu readable=%lu writable=%lu"
                         " error=%lu backoff=%lu",
                         (unsigned long)s_stat_select.timeout, (unsigned long)s_stat_select.eventfd,
                         (unsigned long)s_stat_select.readable, (unsigned long)s_stat_select.writable,
                         (unsigned long)s_stat_select.error, (unsigned long)s_stat_select.backoff);
            }
            }
            s_stat_v_frames = 0;
            s_stat_v_bytes = 0;
            s_stat_t_bytes = 0;
            s_stat_eagain_v_snap = s_video_eagain_cnt;
            s_stat_eagain_t_snap = s_telemetry_eagain_cnt;
            s_stat_tel_overwrite_snap = tel_overwrite;
            s_stat_tel_busy_snap = tel_busy;
            s_stat_tel_alloc_snap = tel_alloc;
            s_stat_select = {};
            s_stat_last_us = stat_now;
        }
    }
}
