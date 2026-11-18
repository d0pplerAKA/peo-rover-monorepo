/**
 * @file    esp_net.cpp
 * @brief   PEO Rover - WiFi STA + TCP 8888 命令/遥测 + TCP 8890 视频流
 *
 * 单任务（select 多路复用）管理：
 *   - AP 启动（esp_netif + esp_wifi）
 *   - TCP 监听 + <=4 客户端（每客户端独立行缓冲）
 *   - JSON 行 RX -> rx_queue；ACK 优先于遥测广播；断连 -> STOP_ALL
 *   - camera 任务零拷贝入队，TCP 定界（8B 头+JPEG）发送与帧归还由本任务完成
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
#include <nvs_flash.h>
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
    /* VFS fd 全局唯一分配（socket 也走 VFS），不存在与 lwip socket 撞号区；
       唯一硬约束是 select 位图上限。 */
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
    /* 用剩余额度比较，避免 bytes 很大时相加发生整数回绕。 */
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

/* 网络任务监督，解析器跨核续期；仅保存本地活动信息，不改变 JSON。 */
static struct {
    uint32_t last_cmd_ms;
    bool has_commanded;
    bool stop_latched;          /* 同一次活动超时只触发一次 STOP */
    int fd = -1;                /* 网络任务维护连接镜像，供跨核查询；空槽必须为 -1 */
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

static EventGroupHandle_t s_conn_eg = NULL; /* CLIENT/VIDEO 事件组（camera/telemetry 门控） */

EventGroupHandle_t esp_net_conn_event(void)
{
    return s_conn_eg;
}

static int s_video_listen_fd = -1;   /* TCP 视频监听 socket（8890） */
static int s_video_fd       = -1;    /* 已接受的视频客户端 */
static portMUX_TYPE s_video_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_video_enabled = false;
static uint16_t s_frame_id = 0;     /* 仅 esp_net 任务发送时更新 */
struct video_frame_t {
    const uint8_t *buf;
    size_t len;
    camera_fb_t *owner;        /* fb_return 要求原始描述符，不能传图像数据地址 */
};
static video_frame_t s_video_pending = {};   /* 1 深，覆盖时归还旧帧 */
static camera_fb_t *s_video_producer_frame = NULL; /* 暂存登记，入队成功前仍属 camera */
static SemaphoreHandle_t s_video_queue_mutex = NULL;
static SemaphoreHandle_t s_video_send_mutex = NULL; /* flush 等待发送中帧归还后才能 deinit */
static TaskHandle_t s_net_task = NULL;
static const uint32_t VIDEO_NOTIFY_BIT = 1u;

/* ============================================================================
   1. WiFi STA 初始化（笔记本热点，2026-11-16 用户定案）
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
        esp_link_request_stop();   /* WiFi 断开 = 失联，立即锁 STOP（用户拍板 2026-11-17） */
        ESP_LOGW(TAG, "wifi sta: disconnected; reconnecting...");
        if (s_conn_eg)
            xEventGroupClearBits(s_conn_eg, PEO_NET_EVT_STA);
        esp_wifi_connect();   /* 自动重连：每次断连事件触发一次连接尝试（失败会再次触发本事件，天然节流） */
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "wifi sta: got IP " IPSTR " (mDNS: %s.local)",
                 IP2STR(&ev->ip_info.ip), PEO_NET_MDNS_HOSTNAME);
    }
}

/* STA 模式：吞吐/稳定性优于 softAP（官方实测），笔记本射频前端更强；
   自动重连 + 省电关闭（遥控实时性优先）+ mDNS 解析（热点 DHCP 地址可变）。 */
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

    /* mDNS：上位机按 peo_rover.local 连接，不依赖 DHCP 分配的 IP */
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
   3. TCP 视频入队与发送（camera 单生产者，esp_net 单消费者）
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
        return false;  /* 未接管当前帧，由调用方归还 */
    }
    if (s_video_pending.owner) esp_camera_fb_return(s_video_pending.owner);
    s_video_pending = {jpeg, len, owner};
    /* 入队与通知在同一互斥区内完成；消费者始终以队列内容为准。 */
    xTaskNotify(s_net_task, VIDEO_NOTIFY_BIT, eSetBits);
    xSemaphoreGive(s_video_queue_mutex);
    esp_net_wake();
    return true;
}

/* 视频拥塞冷却：send 返回 EAGAIN 说明发送缓冲满（空口/对端消化不过来），
   在飞帧必须发完（TCP 定界不可中断），但冷却期内不启新帧——1 深队列
   本就只发最新帧，让链路排水；冷却结束再启新帧，成功即恢复流。
   15fps 下 50ms 冷却约影响 1 帧（取决于入队相位）；只挡新帧，在飞帧继续按预算推进。 */
#define VIDEO_COOLDOWN_US 50000
#define VIDEO_PUMP_MAX_SENDS 3   /* 每轮 pump 的 send 调用预算，头部和 JPEG 体共用 */
#ifndef VIDEO_PUMP_MAX_BYTES
#define VIDEO_PUMP_MAX_BYTES 8192
#endif
static_assert(VIDEO_PUMP_MAX_BYTES > 0, "video byte budget must be positive");

static int64_t s_video_cool_until_us = 0;   /* 冷却截止时刻（见 VIDEO_COOLDOWN_US） */
static uint32_t s_tx_fail_cnt = 0;          /* 视频 send 失败累计（诊断；断连全量复位） */
static int s_last_errno = 0;

/* TCP 视频在飞帧：TCP 字节流一旦开始发就必须发完（否则对端定界错乱）。
   分帧 = 8B 头（magic+fid+len 大端）+ JPEG 体；每轮 select 循环按预算推进，
   非阻塞 send，绝不阻塞主任务（命令/遥测通道保持响应）。 */
struct video_inflight_t {
    camera_fb_t *owner;
    const uint8_t *buf;
    size_t len, off;
    uint8_t hdr[8];            /* magic(2)+fid(2)+len(4) 大端 */
    size_t hdr_off;
    bool hdr_done;
};
/* 发送诊断（Codex 审查 R-1：先让断连根因可区分）——
   视频/遥测各自的 EAGAIN 次数、观察到的最大积压；drop_client 时打印现场。
   ⚠️ 必须在 pump_video/flush_tx 使用之前定义（pump_video 位于本文件前段）。 */
static uint32_t s_video_eagain_cnt = 0;
static uint32_t s_telemetry_eagain_cnt = 0;
static size_t   s_max_tx_pending = 0;

/* 周期统计窗口（1s 打印后清零；EAGAIN 计数保持累计供 diag 用）——
   camera 侧统计已整体移除，视频/遥测的发送信息统一在此（网络应用层）统计。 */
static uint32_t s_stat_v_frames = 0;   /* 本窗口发送完成帧数 */
static uint64_t s_stat_v_bytes  = 0;   /* 本窗口视频字节 */
static uint64_t s_stat_t_bytes  = 0;   /* 本窗口命令/遥测字节 */
static uint32_t s_stat_eagain_v_snap = 0;  /* EAGAIN 累计快照：窗口做差 = 本窗口新增 */
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

/* 每轮同时限制 send 次数和成功发送字节（含头部）；预算用尽留待下一轮 select。
   返回 false = 连接错误（对端关闭），true = 未完或完成。 */
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
                return true;   /* 缓冲满：下一轮继续（在飞帧必须发完） */
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
            /* 诊断：1s 节流 */
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

/* 起新帧：填 8B 头（magic+fid+len），交由 pump_video 逐轮按预算发送 */
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
    /* 不在自旋锁内等待网络发送；统一先 send_mutex 再 queue_mutex，避免锁序反转。 */
    xSemaphoreTake(s_video_send_mutex, portMAX_DELAY);
    xSemaphoreTake(s_video_queue_mutex, portMAX_DELAY);
    if (s_video_pending.owner) esp_camera_fb_return(s_video_pending.owner);
    s_video_pending = {};
    if (s_video_inflight.owner)     /* 作废在飞帧时重置连接，不能复用可能残留半帧的字节流 */
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
    /* 1) 按每轮双预算接续在飞帧；冷却不会阻止在飞帧推进。 */
    if (!pump_video(bytes_sent))
    {
        state_changed = true;
        esp_link_request_stop();   /* 视频发送硬错误 = 图传失联，立即锁 STOP（用户拍板 2026-11-17） */
        ESP_LOGW(TAG, "video socket error; closing video client");
        esp_camera_fb_return(s_video_inflight.owner);
        s_video_inflight = {};
        close(s_video_fd);
        s_video_fd = -1;
    }
    if (s_video_inflight.owner)
    {
        xSemaphoreGive(s_video_send_mutex);
        return bytes_sent != 0 || state_changed;   /* 在飞帧未发完：不启新帧 */
    }
    /* 2) 取 pending 起新帧（无客户端/冷却期内丢弃，1 深队列只发最新帧） */
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
   4. 主任务
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
#define NET_TX_LIMIT 32768        /* 每客户端待发积压上限：给 20Hz 遥测的突发空口中断留排水窗口。 */
#define NET_TX_TIMEOUT_MS 5000   /* 1000 太激进：WiFi 抖动/Windows 扫网 100-300ms 停顿即触发断联 */
#define NET_TX_DRAIN_BATCH_MAX 8 /* ACK 队列单轮最多取出的行数，避免饿死 RX/STOP。 */

/* append_tx 失败原因：区分「积压超限」与「realloc OOM」——
   二者此前共用一条 "tx backlog limit" 日志，无法定位根因。 */
typedef enum {
    TX_APPEND_OK = 0,
    TX_APPEND_BACKLOG,   /* 积压超过 NET_TX_LIMIT */
    TX_APPEND_ALLOC      /* realloc 失败（内存不足） */
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

/* 断连后网络全量复位（用户要求：归还全部内存，回到「无客户端」初始状态）：
   清空遥测邮箱/ACK 广播队列、释放残留在 RX 队列中的客户端行（cmd_parser 仍在消费的
   不受影响——queue 操作原子，此处只收走剩余项）、归还未发送视频帧、
   复位发送冷却/帧号/失败计数。 */
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
    /* 先锁 STOP（安全停车）再回收资源：失联第一时间失效旧命令代次，
       避免被 close/free/cleanup/video_flush 等资源回收拖延（Codex 审查 R-1 严重项）。
       心跳分支已先请求 STOP，此处凭 stop_already_requested 免重复递增代次。 */
    if (!stop_already_requested) post_disconnect();
    /* 现场诊断：pending / EAGAIN 计数 / 最大积压 / 堆余量，用于区分「积压超限 vs OOM」。 */
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
        esp_net_video_set_enabled(false);   /* 同时清 VIDEO 事件位 */
        if (s_conn_eg)
            xEventGroupClearBits(s_conn_eg, PEO_NET_EVT_CLIENT);
        esp_net_cleanup_all();              /* 网络部分全量复位到初始状态 */
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
    s_stat_t_bytes += (size_t)n;        /* 命令/遥测发送字节（网络层统计） */
    c.tx_since = xTaskGetTickCount();   /* 有进展即续期：超时只针对“停滞”，不是“积压年龄” */
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
    /* app_main 已在任务创建前统一建队；本任务以 STA 模式连接热点并运行 select。 */
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_sta();
    s_stat_last_us = esp_timer_get_time();   /* 统计基线：首窗口从任务启动起算 */

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
        xSemaphoreGive(s_video_send_mutex);  /* select 期间允许 camera flush 关闭连接 */
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
        /* 心跳独立于 socket 读写进展；仅监督已提交控制命令/宏的客户端。
           在锁内取 now，避免跨核续期时间晚于先前采样而导致年龄下溢。
           日志与 STOP 均在锁外，避免阻塞或嵌套安全锁。 */
        for (int i = 0; i < PEO_NET_MAX_CLIENTS; ++i)
        {
            if (clients[i].fd < 0) continue;  /* 已断开的槽位不再监督陈旧活动 */
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
                    FD_CLR(cfd, &wfds);  /* A recycled fd cannot inherit old select results. */
                    clients[slot].fd = cfd;
                    portENTER_CRITICAL(&s_activity_lock);
                    s_client_activity[slot] = {};  /* 新连接不继承旧会话活动/超时锁存 */
                    s_client_activity[slot].fd = cfd;
                    portEXIT_CRITICAL(&s_activity_lock);
                    ESP_LOGI(TAG, "client %d connected: fd=%d wake=%d", slot, cfd, s_wake_fd);
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
                    /* ESP-IDF/lwIP 不支持 SOL_SOCKET/SO_SNDBUF：
                       errno 109 = ENOPROTOOPT，并非对端关闭竞态。
                       发送缓冲由 CONFIG_LWIP_TCP_SND_BUF_DEFAULT 配置；
                       沿用现有值，部分写入由在飞帧偏移与 EAGAIN 重试处理。 */
                    xSemaphoreTake(s_video_send_mutex, portMAX_DELAY);
                    if (s_video_fd >= 0)
                    {
                        FD_CLR(s_video_fd, &rfds);
                        FD_CLR(s_video_fd, &wfds);
                        close(s_video_fd);
                    }
                    if (s_video_inflight.owner)   /* 新连接接管：旧在飞帧作废 */
                    {
                        esp_camera_fb_return(s_video_inflight.owner);
                        s_video_inflight = {};
                    }
                    s_video_fd = vfd;
                    FD_CLR(vfd, &rfds);  /* fd 可能复用；旧 select 结果不代表新连接就绪 */
                    FD_CLR(vfd, &wfds);
                    xSemaphoreGive(s_video_send_mutex);
                    ESP_LOGI(TAG, "video client connected: fd=%d wake=%d", vfd, s_wake_fd);
                }
            }
        }
        /* 与 camera flush 互斥：检查、非阻塞 recv、关闭必须使用同一连接。 */
        xSemaphoreTake(s_video_send_mutex, portMAX_DELAY);
        if (selected_video_fd >= 0 && s_video_fd != selected_video_fd)
        {
            FD_CLR(selected_video_fd, &rfds);
            FD_CLR(selected_video_fd, &wfds);
        }
        if (s_video_fd >= 0 && s_video_fd == selected_video_fd && FD_ISSET(s_video_fd, &rfds))
        {
            /* 视频通道单向（S3→上位机）：可读 = 对端关闭/出错 */
            char tmp[16];
            int r = recv(s_video_fd, tmp, sizeof(tmp), 0);
            /* EBADF 幂等跳过，不重复关闭 fd 或清理在飞帧。 */
            if (r <= 0 && !(r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR || errno == EBADF)))
            {
                esp_link_request_stop();   /* 视频对端关闭 = 图传失联，立即锁 STOP（用户拍板 2026-11-17） */
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
                        /* 完整行保留原代次入队；STOP/单向心跳 豁免由解析器统一判定。 */
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
                        /* 入队带 50ms 等待：esp_net(prio5) 批量灌行时给 cmd_parser(prio4)
                           消化时间（批量命令/快速点击/算法命令流均会突发多行）。
                           只有消费者真卡死 50ms 才视为满 → 断开。 */
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
        /* ACK 优先且有批次上限；随后最多消费一条最新遥测。 */
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

        /* 周期统计（1s 窗口，单行原地刷新）：EAGAIN/tel_drop 用累计快照做差，
           select 次数按窗口清零；就绪原因可重叠，readable 不含 eventfd。
           其他组件日志会打断本行，属预期。 */
        const int64_t stat_now = esp_timer_get_time();
        if (stat_now - s_stat_last_us >= 1000000LL)
        {
            const float secs = (float)(stat_now - s_stat_last_us) / 1000000.0f;
            const uint32_t ve = s_video_eagain_cnt - s_stat_eagain_v_snap;
            const uint32_t te = s_telemetry_eagain_cnt - s_stat_eagain_t_snap;
            const uint32_t tel_overwrite = s_tel_drop_overwrite.load();
            const uint32_t tel_busy = s_tel_drop_busy.load();
            const uint32_t tel_alloc = s_tel_drop_append_fail.load();
            /* 全字段固定宽度占位：数字变长/变短时列位置不动，短值由空格自然覆盖，
               行宽恒定；尾部留 2 空格兜底。
               teldrop 顺序固定 = 覆盖/忙/alloc；select = 本窗口 select 调用次数/秒
               （空闲 ~10/s；若达数百/s 说明忙等，细分类在异常日志中另打）。 */
            printf("\rnet: video %3lu fps %6.1f KB/s | telemetry %5.1f KB/s | eagain +%4lu +%4lu"
                   " | teldrop +%4lu/+%4lu/+%4lu | select %4lu/s  ",
                   (unsigned long)s_stat_v_frames,
                   (double)((float)s_stat_v_bytes / secs / 1024.0f),
                   (double)((float)s_stat_t_bytes / secs / 1024.0f),
                   (unsigned long)ve, (unsigned long)te,
                   (unsigned long)(tel_overwrite - s_stat_tel_overwrite_snap),
                   (unsigned long)(tel_busy - s_stat_tel_busy_snap),
                   (unsigned long)(tel_alloc - s_stat_tel_alloc_snap),
                   (unsigned long)s_stat_select.calls);
            fflush(stdout);
            /* 忙等/异常细分类不进常驻行：出现时打一行诊断（1s 节流已由窗口天然保证） */
            if (s_stat_select.backoff != 0 || s_stat_select.error != 0)
            {
                ESP_LOGW(TAG, "select diag: timeout=%lu eventfd=%lu readable=%lu writable=%lu"
                         " error=%lu backoff=%lu",
                         (unsigned long)s_stat_select.timeout, (unsigned long)s_stat_select.eventfd,
                         (unsigned long)s_stat_select.readable, (unsigned long)s_stat_select.writable,
                         (unsigned long)s_stat_select.error, (unsigned long)s_stat_select.backoff);
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
