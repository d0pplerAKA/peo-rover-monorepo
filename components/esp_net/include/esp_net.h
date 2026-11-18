#ifndef __ESP_NET_H__
#define __ESP_NET_H__

/**
 * @file    esp_net.h
 * @brief   PEO Rover - WiFi STA + TCP 8888 命令/遥测 + TCP 8890 视频服务器
 *
 * esp_net_task（优先级 5）：以 STA 模式连接配置的 WiFi 热点；
 * 通过 select 多路复用提供 TCP 8888 服务（单客户端）：
 *   - 完整 JSON 行及本地代次、槽位、接收时间、字节预算信息入 esp_net_rx_queue；
 *   - ACK 专用队列与遥测最新值邮箱分离，ACK 优先广播给所有客户端；
 *   - 客户端断开 -> 自动发 STOP_ALL（PROTOCOL 断连保护硬规则）。
 * TCP 8890：视频流（camera 零拷贝入队，esp_net 任务发送并归还帧，
 * 定界 = 8B 大端头 magic+fid+len + JPEG 体（TCP 自带完整性校验）。
 *
 * RX/ACK 队列由 app_main 在任务创建前统一初始化；遥测使用静态原子邮箱。
 * 遥测任务每 50ms（20Hz）peek 快照；网络任务负责优先广播 ACK，再广播遥测。
 * 视频通道单向（S3→上位机）；客户端断开即清理，上位机仅在视频启用期间自动重连。
 * 超长/含 NUL 行、RX OOM/队满/超字节预算和慢 TX 客户端均断开并锁 STOP。
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_camera.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

/* ============================================================================
   网络参数（PROTOCOL.md §5）
   ============================================================================ */
/* 笔记本热点 STA 模式（2026-11-16 用户定案：不再回 S3 软 AP）。
   SSID/密码 = 用户笔记本移动热点的名称与密码，烧录前必须按实际填写。 */
#define PEO_NET_STA_SSID        "Y9000X"
#define PEO_NET_STA_PASS        "66666666"
#define PEO_NET_MDNS_HOSTNAME   "peo_rover"      /* 上位机按 peo_rover.local 解析 */
#define PEO_NET_TCP_PORT        8888
#define PEO_NET_VIDEO_PORT      8890    /* TCP 视频流（2026-11-16 定案；UDP 8889 已退役） */
#define PEO_NET_MAX_CLIENTS     1      /* 1:1 模式：单客户端独占（TCP accept 层拒绝第二连接；
                                           STA 模式下仅笔记本热点是 AP，其他设备天然进不来） */
#define PEO_NET_HEARTBEAT_TIMEOUT_MS 2000   /* 发过控制命令的客户端：有效命令/单向安全心跳超时 */
#define PEO_NET_MAX_LINE        16384   /* JSON bytes excluding newline */
#define PEO_NET_RX_BUDGET_BYTES 65536   /* 完整 RX 行总字节预算，不含换行/字符串终止符。 */
#define PEO_NET_ACK_QUEUE_DEPTH 8       /* ACK 专用队列深度。 */
#define PEO_NET_VIDEO_MAGIC     0xCAFE

typedef struct {
    char *line;                /* Owned by receiver; free exactly once. */
    uint32_t generation;       /* Safety generation when the line began arriving. */
    uint8_t slot;              /* 本地 TCP 客户端槽位，不参与对外 JSON。 */
    uint32_t received_ms;      /* 完整行换行到达时的 FreeRTOS 单调毫秒时间。 */
    uint32_t line_bytes;       /* 该行字符串字节数，不含换行/字符串终止符；用于归还预算。 */
} esp_net_rx_message_t;
#include <freertos/event_groups.h>

extern QueueHandle_t esp_net_rx_queue;  /* esp_net_rx_message_t */
extern QueueHandle_t esp_net_ack_queue; /* char* JSON 行：ACK 专用，优先广播后 free */

/** 接管 malloc/free 兼容的完整 JSON 字符串，覆盖并释放尚未消费的旧帧。
 * 非 NULL 返回 true（仅表示接管，不保证送达）；NULL 返回 false，不改变邮箱。
 * 调用后生产者不得再访问/释放 json；不能重复发布同一指针。仅任务上下文调用。 */
bool esp_net_publish_telemetry(char *json);

/** 接管 ACK 字符串；成功入队后唤醒网络任务。失败也释放 json，调用者不得再 free。
 * 非阻塞；保留原有队满丢弃策略。返回 true 仅表示成功入队。 */
bool esp_net_send_ack(char *json);

/** 唤醒 select；仅任务上下文、应用锁外调用。禁用/初始化失败时为空操作。 */
void esp_net_wake(void);

/** 队列初始化（app_main 里、创建任务之前调用） */
void esp_net_queues_init(void);

void esp_net_task(void *arg);

/** 跨核查询在线 TCP 客户端；锁内检查连接槽位，未连接时返回 false。 */
bool esp_net_has_client(void);

/* 连接事件组（esp_net 任务维护）：camera/telemetry 用它做「无客户端即停止」门控，
   ws2812 用它做状态灯。
   PEO_NET_EVT_CLIENT = 有在线客户端（accept 置位 / 全部断开清零）；
   PEO_NET_EVT_VIDEO  = 视频已启用（esp_net_video_set_enabled 同步置/清）；
   PEO_NET_EVT_STA    = WiFi 已连上笔记本热点（STA_CONNECTED 置位 / DISCONNECTED 清零）。 */
#define PEO_NET_EVT_CLIENT    (1u << 0)
#define PEO_NET_EVT_VIDEO     (1u << 1)
#define PEO_NET_EVT_STA       (1u << 2)
EventGroupHandle_t esp_net_conn_event(void);

/** 解析器确认有效活动后续期；commanded 标记该客户端已提交MOTOR/MOTOR_CLOSED/PID 命令。 */
void esp_net_note_client_activity(uint8_t slot, uint32_t received_ms, bool commanded);

/** 完整 RX 行的跨核字节预算；成功 reserve 后，由出队消费者或入队失败路径 release。 */
bool esp_net_rx_reserve(uint32_t bytes);
void esp_net_rx_release(uint32_t bytes);

/* 视频全局开关：上电默认关闭，所有 TCP 客户端断开后自动关闭。 */
void esp_net_video_set_enabled(bool enabled);
bool esp_net_video_enabled(void);

/** camera 单生产者在 send_video 前登记原始描述符；登记本身不转移所有权。 */
void esp_net_video_set_frame_owner(camera_fb_t *fb);

/** 异步入队 JPEG。true：接管帧并通知网络任务；false：调用方必须归还帧。
 * 必须先登记 buf 所属的原始 camera_fb_t，不能把 buf 强转为帧描述符。
 */
bool esp_net_send_video(const uint8_t *jpeg, uint32_t len);

/** 清空待发帧；若有在飞帧则关闭视频连接并归还帧，防止半帧流复用。
 * camera 单生产者须在 deinit/退出前调用；仅有 pending 时保留连接。 */
void esp_net_video_flush(void);

#endif /* __ESP_NET_H__ */
