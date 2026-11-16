#ifndef __ESP_NET_H__
#define __ESP_NET_H__

/**
 * @file    esp_net.h
 * @brief   PEO Rover - WiFi AP + TCP/UDP 服务器
 *
 * esp_net_task（优先级 5）：启动 AP（PEO_Rover / 192.168.4.1）；
 * 通过 select 多路复用提供 TCP 8888 服务（单客户端）：
 *   - 完整 JSON 行及本地代次、槽位、接收时间、字节预算信息入 esp_net_rx_queue；
 *   - ACK/PING 与遥测分队列，ACK 优先广播给所有客户端；
 *   - 客户端断开 -> 自动发 STOP_ALL（PROTOCOL 断连保护硬规则）。
 * UDP 8889：视频流（camera 零拷贝入队，esp_net 任务发送并归还帧，
 * 8B 大端头 0xCAFE + JPEG 分片 <=1400B）。
 *
 * RX/ACK/遥测队列由 app_main 在任务创建前统一初始化。
 * 遥测任务每轮延时 40ms 后 peek 快照；网络任务负责优先广播 ACK，再广播遥测。
 * UDP 目标为在线 TCP 客户端的 IPv4 地址、端口 8889，无额外握手。
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
#define PEO_NET_AP_SSID         "PEO_Rover"
#define PEO_NET_AP_PASS         "12345678"
#define PEO_NET_AP_IP           "192.168.4.1"
#define PEO_NET_TCP_PORT        8888
#define PEO_NET_UDP_PORT        8889
#define PEO_NET_MAX_CLIENTS     1      /* 1:1 模式：单客户端独占（同时限制 AP 最大关联
                                          STA 数）。2026-11-15 用户定案（PROTOCOL §2.3）；
                                          accept 无空槽即拒绝第二连接。 */
#define PEO_NET_HEARTBEAT_TIMEOUT_MS 2000   /* 发过控制命令的客户端：有效命令/PING 心跳超时 */
#define PEO_NET_MAX_LINE        16384   /* JSON bytes excluding newline */
#define PEO_NET_RX_BUDGET_BYTES 65536   /* 完整 RX 行总字节预算，不含换行/字符串终止符。 */
#define PEO_NET_ACK_QUEUE_DEPTH 8       /* ACK/PING 专用队列深度。 */
#define PEO_NET_VIDEO_MAX_PKT   1400    /* JPEG 分片上限（避 MTU，PROTOCOL.md §3）。
                                           2026-11-15 实验 4096（IP 分片路径）已回退；
                                           上位机接收端不限包大小（64KB recvfrom）。 */
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
extern QueueHandle_t esp_net_tx_queue;  /* char* JSON 行：telemetry 生产 -> esp_net 广播后 free */
extern QueueHandle_t esp_net_ack_queue; /* char* JSON 行：ACK/PING 专用，优先广播后 free */

/** 队列初始化（app_main 里、创建任务之前调用） */
void esp_net_queues_init(void);

void esp_net_task(void *arg);

/** 跨核查询在线 TCP 客户端；锁内检查连接槽位，未连接时返回 false。 */
bool esp_net_has_client(void);

/* 连接事件组（esp_net 任务维护）：camera/telemetry 用它做「无客户端即停止」门控。
   PEO_NET_EVT_CLIENT = 有在线客户端（accept 置位 / 全部断开清零）；
   PEO_NET_EVT_VIDEO  = 视频已启用（esp_net_video_set_enabled 同步置/清）。 */
#define PEO_NET_EVT_CLIENT    (1u << 0)
#define PEO_NET_EVT_VIDEO     (1u << 1)
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
 * 必须先登记 buf 所属的原始 camera_fb_t，不能把 buf 强转为帧描述符。 */
bool esp_net_send_video(const uint8_t *jpeg, uint32_t len);

/** 清空待发帧并等待发送中帧归还；camera 单生产者须在 deinit/退出前调用。 */
void esp_net_video_flush(void);

#endif /* __ESP_NET_H__ */
