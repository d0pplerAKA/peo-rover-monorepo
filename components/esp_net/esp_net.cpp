/**
 * @file    esp_net.cpp
 * @brief   PEO Rover - WiFi AP + TCP 8888 服务器 + UDP 8889 视频流
 *
 * 单任务（select 多路复用）管理：
 *   - AP 启动（esp_netif + esp_wifi）
 *   - TCP 监听 + <=4 客户端（每客户端独立行缓冲）
 *   - JSON 行 RX -> rx_queue；ACK 优先于遥测广播；断连 -> STOP_ALL
 *   - camera 任务零拷贝入队，UDP 分片发送与帧归还由本任务完成
 */

#include "esp_net.h"
#include "esp_link.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include <esp_camera.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <nvs_flash.h>
#include <fcntl.h>
#include <errno.h>
#include <lwip/sockets.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>

static const char *TAG = "esp_net";

QueueHandle_t esp_net_rx_queue = NULL;
QueueHandle_t esp_net_tx_queue = NULL;
QueueHandle_t esp_net_ack_queue = NULL;

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

static int s_udp_fd       = -1;
static portMUX_TYPE s_video_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_video_enabled = false;
static sockaddr_in s_video_peers[PEO_NET_MAX_CLIENTS] = {};
static uint16_t s_frame_id = 0;     /* 仅 esp_net 任务发送时更新 */
struct video_frame_t {
    const uint8_t *buf;
    size_t len;
    camera_fb_t *owner;            /* fb_return 要求原始描述符，不能传图像数据地址 */
};
static video_frame_t s_video_pending = {};   /* 1 深，覆盖时归还旧帧 */
static camera_fb_t *s_video_producer_frame = NULL; /* 暂存登记，入队成功前仍属 camera */
static SemaphoreHandle_t s_video_queue_mutex = NULL;
static SemaphoreHandle_t s_video_send_mutex = NULL; /* flush 等待发送中帧归还后才能 deinit */
static TaskHandle_t s_net_task = NULL;
static const uint32_t VIDEO_NOTIFY_BIT = 1u;

/* ============================================================================
   1. WiFi AP 初始化
   ============================================================================ */
/* ----------------------------------------------------------------------------
   1a. AP 信道自动选择：起 AP 前用 STA 接口扫描，选「干扰最小」的信道。

   ESP-IDF 限制：esp_wifi_scan_start() 仅在 STA 或 APSTA 模式可用，纯 AP 不能扫
   （官方 issue #13371）。故临时创建 STA netif + 切 STA 模式扫描，扫完 stop 再
   切回纯 AP。STA netif 之后保持 idle（不 connect、不占空口）。

   评分：对每个候选信道累加所有扫描到 AP 的干扰权重
        权重 = 重叠系数(0~5) × 信号强度因子
        - 重叠系数 = 5 - |AP信道 - 候选信道|，间隔>=5 视为不重叠
          （20MHz 带宽、5MHz 信道间隔）
        - 信号强度因子 = 100 + rssi（rssi 为负：-30dBm→70 强干扰，-90dBm→10 弱）
   选得分最低（最干净）的信道。全信道主动扫描引入约 1~1.6s 启动延迟，
   换取视频传输更稳的空口——开机后上位机才连，可接受。
   ---------------------------------------------------------------------------- */
static uint8_t pick_least_congested(const wifi_ap_record_t *aps, uint16_t num)
{
    uint8_t best_ch = 1;
    int best_score = INT_MAX;
    for (int ch = 1; ch <= 11; ch++)   /* 2.4G 合法信道 1-11（北美/中国交集） */
    {
        int score = 0;
        for (uint16_t i = 0; i < num; i++)
        {
            int gap = (int)aps[i].primary - ch;
            if (gap < 0) gap = -gap;
            if (gap < 5)
            {
                int strength = 100 + (int)aps[i].rssi;   /* -30dBm→70, -90dBm→10 */
                if (strength < 0) strength = 0;
                score += (5 - gap) * strength;
            }
        }
        if (score < best_score)
        {
            best_score = score;
            best_ch = (uint8_t)ch;
        }
    }
    return best_ch;
}

static uint8_t scan_select_channel(void)
{
    uint8_t best_ch = 1;
    uint16_t scanned = 0;

    esp_netif_t *sta = esp_netif_create_default_wifi_sta();   /* 临时 STA netif，仅扫描用 */
    if (!sta)
    {
        ESP_LOGW(TAG, "channel scan: STA netif create failed, fallback ch1");
        return best_ch;
    }
    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK ||
        esp_wifi_start() != ESP_OK)
    {
        ESP_LOGW(TAG, "channel scan: enter STA mode failed, fallback ch1");
        return best_ch;
    }

    wifi_scan_config_t scan_cfg;
    memset(&scan_cfg, 0, sizeof(scan_cfg));
    scan_cfg.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    scan_cfg.scan_time.active.min = 100;   /* 每信道最短驻留，压缩启动延迟 */
    scan_cfg.scan_time.active.max = 150;

    if (esp_wifi_scan_start(&scan_cfg, true) == ESP_OK)
    {
        uint16_t num = 0;
        esp_wifi_scan_get_ap_num(&num);
        scanned = num;
        if (num > 0)
        {
            wifi_ap_record_t *aps = (wifi_ap_record_t *)calloc(num, sizeof(wifi_ap_record_t));
            if (aps)
            {
                if (esp_wifi_scan_get_ap_records(&num, aps) == ESP_OK)
                    best_ch = pick_least_congested(aps, num);
                free(aps);
            }
            else
            {
                esp_wifi_clear_ap_list();   /* 无法分配记录缓冲，清掉 driver 扫描列表避免残留 */
            }
        }
    }
    else
    {
        ESP_LOGW(TAG, "channel scan: scan start failed, fallback ch1");
    }

    esp_wifi_stop();
    ESP_LOGI(TAG, "channel scan: %u APs -> cleanest ch%u", (unsigned)scanned, (unsigned)best_ch);
    return best_ch;
}

static void wifi_init_ap(void)
{
    wifi_config_t wcfg;
    memset(&wcfg, 0, sizeof(wcfg));

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    strlcpy((char *)wcfg.ap.ssid, PEO_NET_AP_SSID, sizeof(wcfg.ap.ssid));
    strlcpy((char *)wcfg.ap.password, PEO_NET_AP_PASS, sizeof(wcfg.ap.password));
    wcfg.ap.ssid_len       = strlen(PEO_NET_AP_SSID);
    wcfg.ap.max_connection = PEO_NET_MAX_CLIENTS;
    wcfg.ap.authmode       = WIFI_AUTH_WPA2_PSK;
    wcfg.ap.channel        = scan_select_channel();   /* 扫描选最干净信道（原硬编码 1） */

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP up: %s @ %s ch=%u", PEO_NET_AP_SSID, PEO_NET_AP_IP, (unsigned)wcfg.ap.channel);
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
   3. UDP 视频入队与发送（camera 单生产者，esp_net 单消费者）
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
    bool available = s_video_enabled && s_udp_fd >= 0;
    bool have_peer = false;
    for (int i = 0; i < PEO_NET_MAX_CLIENTS; ++i)
        have_peer |= s_video_peers[i].sin_family == AF_INET;
    portEXIT_CRITICAL(&s_video_lock);
    if (!available || !have_peer || !s_net_task || !owner || owner->buf != jpeg ||
        !jpeg || len == 0 || len > owner->len ||
        len > (uint32_t)UINT16_MAX * PEO_NET_VIDEO_MAX_PKT) {
        xSemaphoreGive(s_video_queue_mutex);
        return false;  /* 未接管当前帧，由调用方归还 */
    }
    if (s_video_pending.owner) esp_camera_fb_return(s_video_pending.owner);
    s_video_pending = {jpeg, len, owner};
    /* 入队与通知在同一互斥区内完成；消费者始终以队列内容为准。 */
    xTaskNotify(s_net_task, VIDEO_NOTIFY_BIT, eSetBits);
    xSemaphoreGive(s_video_queue_mutex);
    return true;
}

void esp_net_video_flush(void)
{
    if (!s_video_send_mutex || !s_video_queue_mutex) return;
    /* 不在自旋锁内等待网络发送；统一先 send_mutex 再 queue_mutex，避免锁序反转。 */
    xSemaphoreTake(s_video_send_mutex, portMAX_DELAY);
    xSemaphoreTake(s_video_queue_mutex, portMAX_DELAY);
    if (s_video_pending.owner) esp_camera_fb_return(s_video_pending.owner);
    s_video_pending = {};
    s_video_producer_frame = NULL;  /* 登记不代表接管，不能归还尚未入队的帧 */
    xSemaphoreGive(s_video_queue_mutex);
    xSemaphoreGive(s_video_send_mutex);
}

/* 视频拥塞冷却：sendto 失败说明空口发送队列已满，继续逐帧硬撞只会延长队列
   消化时间。失败后 200ms 内直接丢弃新帧（1 深队列本就只发最新帧），
   让队列排水；冷却结束再试，成功即恢复流。 */
#define VIDEO_COOLDOWN_US 200000

static int64_t s_video_cool_until_us = 0;   /* 冷却截止时刻（见 VIDEO_COOLDOWN_US） */
static uint32_t s_tx_fail_cnt = 0;          /* 视频 sendto 失败累计（诊断；断连全量复位） */
static int s_last_errno = 0;
static bool send_video_now(const uint8_t *jpeg, uint32_t len)
{
    static uint8_t pkt[PEO_NET_VIDEO_MAX_PKT + 8];

    sockaddr_in peers[PEO_NET_MAX_CLIENTS];
    portENTER_CRITICAL(&s_video_lock);
    int fd = s_udp_fd;
    bool enabled = s_video_enabled;
    memcpy(peers, s_video_peers, sizeof(peers));
    portEXIT_CRITICAL(&s_video_lock);
    if (!enabled || fd < 0 || !jpeg || len == 0 ||
        len > (uint32_t)UINT16_MAX * PEO_NET_VIDEO_MAX_PKT)
        return false;
    bool have_peer = false;
    for (int i = 0; i < PEO_NET_MAX_CLIENTS; ++i)
        have_peer |= peers[i].sin_family == AF_INET;
    if (!have_peer)
    {
        /* 诊断：tx 失败高发时先区分「无 TCP 客户端」与「sendto 失败」。1s 节流。 */
        static int64_t s_last_nopeer_log_us = 0;
        const int64_t now_us = esp_timer_get_time();
        if (now_us - s_last_nopeer_log_us > 1000000LL)
        {
            ESP_LOGW(TAG, "video drop: no TCP client (have_peer=false), fd=%d", fd);
            s_last_nopeer_log_us = now_us;
        }
        return false;
    }

    uint16_t total = (uint16_t)((len + PEO_NET_VIDEO_MAX_PKT - 1) / PEO_NET_VIDEO_MAX_PKT);
    uint16_t fid   = ++s_frame_id;

    for (uint16_t i = 0; i < total; i++)
    {
        uint32_t off   = (uint32_t)i * PEO_NET_VIDEO_MAX_PKT;
        uint32_t chunk = (len - off < PEO_NET_VIDEO_MAX_PKT)
                       ? (len - off) : PEO_NET_VIDEO_MAX_PKT;

        /* 8B 大端头：magic(2) + frame_id(2) + packet_id(2) + total_pkt(2) */
        pkt[0] = (uint8_t)(PEO_NET_VIDEO_MAGIC >> 8);
        pkt[1] = (uint8_t)(PEO_NET_VIDEO_MAGIC & 0xFF);
        pkt[2] = (uint8_t)(fid >> 8);
        pkt[3] = (uint8_t)(fid & 0xFF);
        pkt[4] = (uint8_t)(i >> 8);
        pkt[5] = (uint8_t)(i & 0xFF);
        pkt[6] = (uint8_t)(total >> 8);
        pkt[7] = (uint8_t)(total & 0xFF);
        memcpy(&pkt[8], &jpeg[off], chunk);

        for (int j = 0; j < PEO_NET_MAX_CLIENTS; ++j)
        {
            if (peers[j].sin_family != AF_INET) continue;
            bool duplicate = false;
            for (int k = 0; k < j; ++k)
                if (peers[k].sin_family == AF_INET &&
                    peers[k].sin_addr.s_addr == peers[j].sin_addr.s_addr)
                    duplicate = true;
            if (!duplicate && sendto(fd, pkt, 8 + chunk, 0,
                    (const sockaddr *)&peers[j], sizeof(peers[j])) != (int)(8 + chunk))
            {
                /* 首个失败即中止本帧：上位机严格组装器会整帧丢弃缺包帧，继续发余下
                   分包纯属浪费空口时间；立刻退出把带宽留给下一帧。 */
                /* 诊断：errno 分类（ENOBUFS=空口饱和；EINVAL/EMFILE=配置/资源；
                   ENXIO/EHOSTUNREACH=路由/ARP 失败）。1s 节流。 */
                static int64_t s_last_tx_fail_log_us = 0;
                s_tx_fail_cnt++;
                s_last_errno = errno;
                const int64_t now_us = esp_timer_get_time();
                if (now_us - s_last_tx_fail_log_us > 1000000LL)
                {
                    ESP_LOGW(TAG, "video sendto fail: errno=%d (%s), fail_cnt=%lu, free_heap=%lu (min=%lu)",
                             s_last_errno, strerror(s_last_errno),
                             (unsigned long)s_tx_fail_cnt,
                             (unsigned long)esp_get_free_heap_size(),
                             (unsigned long)esp_get_minimum_free_heap_size());
                    s_last_tx_fail_log_us = now_us;
                }
                s_video_cool_until_us = now_us + VIDEO_COOLDOWN_US;   /* 进入冷却，放队列排水 */
                return false;
            }
        }
        /* 整帧连续 sendto，不增加包间延时。 */
    }
    return true;
}

static bool video_pending(void)
{
    xSemaphoreTake(s_video_queue_mutex, portMAX_DELAY);
    bool pending = s_video_pending.owner != NULL;
    xSemaphoreGive(s_video_queue_mutex);
    return pending;
}

static void send_pending_video(void)
{
    xSemaphoreTake(s_video_send_mutex, portMAX_DELAY);
    xSemaphoreTake(s_video_queue_mutex, portMAX_DELAY);
    video_frame_t frame = s_video_pending;
    s_video_pending = {};
    xSemaphoreGive(s_video_queue_mutex);
    if (frame.owner) {
        /* 无 peer、视频已关闭、冷却期或任意 sendto 失败，均走同一归还路径。 */
        if (esp_timer_get_time() >= s_video_cool_until_us)
            send_video_now(frame.buf, (uint32_t)frame.len);
        esp_camera_fb_return(frame.owner);
    }
    xSemaphoreGive(s_video_send_mutex);
}

/* ============================================================================
   4. 主任务
   ============================================================================ */
void esp_net_queues_init(void)
{
    if (!esp_net_rx_queue) esp_net_rx_queue = xQueueCreate(32, sizeof(esp_net_rx_message_t));
    if (!esp_net_tx_queue) esp_net_tx_queue = xQueueCreate(8, sizeof(char *));
    if (!esp_net_ack_queue) esp_net_ack_queue = xQueueCreate(PEO_NET_ACK_QUEUE_DEPTH, sizeof(char *));
    if (!s_video_queue_mutex) s_video_queue_mutex = xSemaphoreCreateMutex();
    if (!s_video_send_mutex) s_video_send_mutex = xSemaphoreCreateMutex();
    if (!s_conn_eg) s_conn_eg = xEventGroupCreate();
    configASSERT(esp_net_rx_queue && esp_net_tx_queue && esp_net_ack_queue &&
                 s_video_queue_mutex && s_video_send_mutex && s_conn_eg);
}

/* One bounded pending byte stream per client; never discard a partial JSON line. */
#define NET_TX_LIMIT 32768        /* 每客户端待发积压上限：8KB 在遥测 25Hz 下只扛 ~2s；
                                     放宽到 32KB 给突发空口中断留自愈窗口（1 客户端 32KB，代价可忽略） */
#define NET_TX_TIMEOUT_MS 5000   /* 1000 太激进：WiFi 抖动/Windows 扫网 100-300ms 停顿即触发断联 */
#define NET_TX_DRAIN_BATCH_MAX 8 /* 每个发送队列单轮最多取出的行数，避免饿死 RX/STOP。 */
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

/* 断连后网络全量复位（用户要求：归还全部内存，回到「无客户端」初始状态）：
   清空遥测/ACK 广播队列、释放残留在 RX 队列中的客户端行（cmd_parser 仍在消费的
   不受影响——queue 操作原子，此处只收走剩余项）、归还未发送视频帧、
   复位发送冷却/帧号/失败计数。 */
static void esp_net_cleanup_all(void)
{
    char *out;
    while (xQueueReceive(esp_net_tx_queue, &out, 0) == pdTRUE) free(out);
    while (xQueueReceive(esp_net_ack_queue, &out, 0) == pdTRUE) free(out);
    esp_net_rx_message_t msg;
    while (xQueueReceive(esp_net_rx_queue, &msg, 0) == pdTRUE)
    {
        esp_net_rx_release(msg.line_bytes);
        free(msg.line);
    }
    esp_net_video_flush();
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
    close(c.fd);
    free(c.buf);
    free(c.tx);
    c = {};
    c.fd = -1;
    portENTER_CRITICAL(&s_activity_lock);
    s_client_activity[slot] = {};
    portEXIT_CRITICAL(&s_activity_lock);
    portENTER_CRITICAL(&s_video_lock);
    s_video_peers[slot] = {};
    portEXIT_CRITICAL(&s_video_lock);
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
    /* 心跳分支已先请求 STOP；仍执行全部回收，但同一事件不能再次递增代次。 */
    if (!stop_already_requested) post_disconnect();
}

static bool append_tx(net_client_t &c, const char *out, size_t len)
{
    size_t pending = c.tx_len - c.tx_off;
    bool newline = len == 0 || out[len - 1] != '\n';
    if (len + newline > NET_TX_LIMIT - pending) return false;
    if (c.tx_off && pending) memmove(c.tx, c.tx + c.tx_off, pending);
    c.tx_len = pending;
    c.tx_off = 0;
    char *tx = (char *)realloc(c.tx, pending + len + newline);
    if (!tx) return false;
    c.tx = tx;
    memcpy(c.tx + pending, out, len);
    if (newline) c.tx[pending + len] = '\n';
    c.tx_len += len + newline;
    if (!pending) c.tx_since = xTaskGetTickCount();
    return true;
}

static bool flush_tx(net_client_t &c)
{
    if (c.tx_off == c.tx_len) return true;
    if ((TickType_t)(xTaskGetTickCount() - c.tx_since) >= pdMS_TO_TICKS(NET_TX_TIMEOUT_MS))
        return false;
    int n = send(c.fd, c.tx + c.tx_off, c.tx_len - c.tx_off, 0);
    if (n < 0) return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
    if (!n) return false;
    c.tx_off += (size_t)n;
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
    /* app_main 已在任务创建前统一建队；本机无 BLE，本任务独立启动 AP 并运行 select。 */
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_ap();

    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    int one = 1;
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(PEO_NET_TCP_PORT);
    if (listen_fd < 0 ||
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

    int udp_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    addr.sin_port = htons(PEO_NET_UDP_PORT);
    if (udp_fd >= 0 &&
        (bind(udp_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0 || !nonblocking(udp_fd)))
    {
        close(udp_fd);
        udp_fd = -1;
    }
    portENTER_CRITICAL(&s_video_lock);
    s_udp_fd = udp_fd;
    portEXIT_CRITICAL(&s_video_lock);
    if (udp_fd < 0) ESP_LOGE(TAG, "UDP startup failed");

    net_client_t clients[PEO_NET_MAX_CLIENTS] = {};
    for (int i = 0; i < PEO_NET_MAX_CLIENTS; i++) clients[i].fd = -1;
    while (1)
    {
        fd_set rfds;
        int maxfd = listen_fd;
        FD_ZERO(&rfds);
        FD_SET(listen_fd, &rfds);
        for (int i = 0; i < PEO_NET_MAX_CLIENTS; i++)
        {
            if (clients[i].fd >= 0)
            {
                FD_SET(clients[i].fd, &rfds);
                if (clients[i].fd > maxfd) maxfd = clients[i].fd;
            }
        }
        /* 每轮先查队列再 select；通知只作提示，不能清掉尚未处理的帧。
           xTaskNotify 不会中断 lwIP select；并发入队最迟在原有 20ms 超时后处理。 */
        uint32_t notifications = 0;
        xTaskNotifyWait(0, UINT32_MAX, &notifications, 0);
        bool pending = video_pending();
        struct timeval tv = { 0, pending || (notifications & VIDEO_NOTIFY_BIT) ? 0 : 20000 };
        int n = select(maxfd + 1, &rfds, NULL, NULL, &tv);
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
            if (errno != EINTR) ESP_LOGW(TAG, "select err %d", errno);
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
                if (slot < 0 || !nonblocking(cfd))
                {
                    if (slot < 0)
                        ESP_LOGW(TAG, "reject connection: 1:1 mode, client already connected");
                    close(cfd);
                }
                else
                {
                    clients[slot].fd = cfd;
                    portENTER_CRITICAL(&s_activity_lock);
                    s_client_activity[slot] = {};  /* 新连接不继承旧会话活动/超时锁存 */
                    s_client_activity[slot].fd = cfd;
                    portEXIT_CRITICAL(&s_activity_lock);
                    peer.sin_port = htons(PEO_NET_UDP_PORT);
                    portENTER_CRITICAL(&s_video_lock);
                    s_video_peers[slot] = peer;
                    portEXIT_CRITICAL(&s_video_lock);
                    ESP_LOGI(TAG, "client %d connected", slot);
                    if (s_conn_eg)
                        xEventGroupSetBits(s_conn_eg, PEO_NET_EVT_CLIENT);
                }
            }
        }
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
                        /* 完整行保留原代次入队；STOP/PING/abort 豁免由解析器统一判定。 */
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
        /* ACK/PING 优先；两个队列均有批次上限，广播与字符串所有权规则一致。 */
        for (int batch = 0; batch < NET_TX_DRAIN_BATCH_MAX &&
             xQueueReceive(esp_net_ack_queue, &out, 0) == pdTRUE; ++batch)
        {
            if (!out) continue;
            size_t len = strlen(out);
            for (int i = 0; i < PEO_NET_MAX_CLIENTS; i++)
                if (clients[i].fd >= 0 && !append_tx(clients[i], out, len))
                    drop_client(clients, clients[i], i, "tx backlog limit");
            free(out);
        }
        for (int batch = 0; batch < NET_TX_DRAIN_BATCH_MAX &&
             xQueueReceive(esp_net_tx_queue, &out, 0) == pdTRUE; ++batch)
        {
            if (!out) continue;
            size_t len = strlen(out);
            for (int i = 0; i < PEO_NET_MAX_CLIENTS; i++)
                if (clients[i].fd >= 0 && !append_tx(clients[i], out, len))
                    drop_client(clients, clients[i], i, "tx backlog limit");
            free(out);
        }
        for (int i = 0; i < PEO_NET_MAX_CLIENTS; i++)
            if (clients[i].fd >= 0 && !flush_tx(clients[i])) drop_client(clients, clients[i], i, "tx stall");
        send_pending_video();
    }
}
