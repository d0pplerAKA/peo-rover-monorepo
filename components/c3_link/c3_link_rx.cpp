/**
 * @file    c3_link_rx.cpp
 * @brief   PEO Rover - ESP32-C3 链路 RX：UART 状态帧解析 + 失联锁 STOP
 *
 * 帧格式与 esp_link 同构（PROTOCOL.md §1.4）：
 *   SOF(0xAA 0x55) + LEN(1) + PAYLOAD + CRC16(2)；PAYLOAD = type(1) + body。
 * 逐字节状态机：SOF 同步、LEN 边界、CRC 校验、丢字节保留后缀重同步
 * （与 esp_link_rx_task 同模式，但 LEN 可变）。
 *
 * 安全语义：
 *   - 每收到有效 GAMEPAD_STATE 帧刷新 s_last_frame_ms（C3 存活心跳）。
 *   - 连续 C3_LINK_TIMEOUT_MS(500ms) 无帧 = C3 失联 -> esp_link_request_stop()
 *     （本地锁 STOP，TX 自动补发 STOP_ALL）+ 快照 connected=0；恢复收帧仅清失联标志。
 *   - 不在此处 resume：resume 由 c3_map_task 的新鲜回中逻辑负责。
 */

#include "c3_link.h"
#include "esp_link.h"
#include "PEO_PINMAP.h"

#include <Arduino.h>      /* millis()：与 c3_map_task 时间基准一致 */

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <string.h>

static const char *TAG = "c3_link";

QueueHandle_t xbox_state_queue = NULL;

/* ============================================================================
   CRC16-CCITT（多项式 0x1021，初始 0xFFFF，查表法）
   与 esp_link / C3 端 c3_uart 完全一致；独立实现避免依赖 esp_link 内部符号。
   ============================================================================ */
static uint16_t s_crc_table[256];
static bool     s_crc_ready = false;

static void crc_build_table(void)
{
    for (uint16_t i = 0; i < 256; i++)
    {
        uint16_t crc = (uint16_t)(i << 8);
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        s_crc_table[i] = crc;
    }
    s_crc_ready = true;
}

static uint16_t c3_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    if (!s_crc_ready)
        crc_build_table();
    while (len--)
        crc = (uint16_t)((crc << 8) ^ s_crc_table[((crc >> 8) ^ *data++) & 0xFFU]);
    return crc;
}

/* ============================================================================
   RX 状态机（有界滑动候选帧）
   ============================================================================ */
#define C3_RX_MAX_PAYLOAD       32U     /* 当前唯一类型 26B；留余量支持未来类型 */
#define C3_GAMEPAD_PAYLOAD_LEN  26U     /* type(1) + body(25) */

static uint8_t s_rx_buf[5 + C3_RX_MAX_PAYLOAD];
static size_t  s_rx_count = 0;

/* C3 存活心跳与通知时间追踪 */
static uint32_t s_last_frame_ms = 0;          /* 0 = 尚未收到任何帧（不判失联） */
static uint32_t s_last_seq = 0;
static uint32_t s_last_notification_ms = 0;
static bool     s_c3_lost = false;

static uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/** GAMEPAD_STATE body（25B，PROTOCOL.md §1.4 偏移表）-> xbox_state_t 快照 */
static void apply_gamepad(const uint8_t *b)
{
    const uint32_t now = millis();

    xbox_state_t st;
    memset(&st, 0, sizeof(st));
    st.notification_sequence = rd32(b + 0);
    st.disconnect_generation = rd32(b + 4);
    st.connected    = b[8];
    st.battery      = b[9];
    st.input_fresh  = b[10];
    st.joy_lx       = rd16(b + 11);
    st.joy_ly       = rd16(b + 13);
    st.joy_rx       = rd16(b + 15);
    st.joy_ry       = rd16(b + 17);
    st.trigger_lt   = rd16(b + 19);
    st.trigger_rt   = rd16(b + 21);
    st.buttons      = rd16(b + 23);
    st.snapshot_ms  = now;

    /* 有效 HID 通知时间：仅在 seq 变化时刷新（C3 端电池/短报告不递增 seq）。
       seq 透传自 C3 asukiaaa snapshot，跨重连递增。 */
    if (st.notification_sequence != s_last_seq) {
        s_last_notification_ms = now;
        s_last_seq = st.notification_sequence;
    }
    st.notification_ms = s_last_notification_ms;

    s_last_frame_ms = now;
    if (s_c3_lost) {
        ESP_LOGW(TAG, "C3 link recovered");
        s_c3_lost = false;
    }

    xQueueOverwrite(xbox_state_queue, &st);
}

/** 一帧完整（CRC 已过）：按 type 分派 */
static void handle_frame(const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len == 0)
        return;
    if (payload[0] == C3_TYPE_GAMEPAD_STATE) {
        if (payload_len != C3_GAMEPAD_PAYLOAD_LEN) {
            ESP_LOGW(TAG, "gamepad frame bad len %u (expect %u)",
                     payload_len, C3_GAMEPAD_PAYLOAD_LEN);
            return;
        }
        apply_gamepad(&payload[1]);
    } else {
        ESP_LOGW(TAG, "unknown frame type 0x%02x (len %u)", payload[0], payload_len);
    }
}

void c3_link_queues_init(void)
{
    if (xbox_state_queue == NULL)
        xbox_state_queue = xQueueCreate(1, sizeof(xbox_state_t));
    configASSERT(xbox_state_queue);
    crc_build_table();
}

void c3_link_rx_task(void *arg)
{
    ESP_LOGI(TAG, "rx task starting");

    /* UART 初始化（8-N-1，无流控，与 C3 端 c3_uart 一致） */
    uart_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.baud_rate  = PEO_PM_C3_BAUDRATE;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    esp_err_t err = uart_driver_install(PEO_PM_C3_UART, 2048, 0, 0, NULL, 0);
    if (err == ESP_OK) err = uart_param_config(PEO_PM_C3_UART, &cfg);
    if (err == ESP_OK)
        err = uart_set_pin(PEO_PM_C3_UART, PEO_PM_C3_TX, PEO_PM_C3_RX,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %s; link remains disabled", esp_err_to_name(err));
        esp_link_request_stop();
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "rx task up (UART%d, %u baud)", PEO_PM_C3_UART,
             (unsigned)PEO_PM_C3_BAUDRATE);

    while (1)
    {
        /* 监督有效状态帧，而不是 UART 是否有字节；持续坏流也必须检查超时。
           s_last_frame_ms == 0 表示尚未收到首帧，保留不监督的启动语义。 */
        const uint32_t now = millis();
        uint32_t wait_ms = 100;
        if (!s_c3_lost && s_last_frame_ms != 0) {
            const uint32_t age_ms = (uint32_t)(now - s_last_frame_ms);
            if (age_ms >= C3_LINK_TIMEOUT_MS) {
                s_c3_lost = true;
                ESP_LOGW(TAG, "C3 link lost (%lu ms no frame); latching STOP",
                         (unsigned long)age_ms);
                esp_link_request_stop();
                /* 快照标记离线（保留 seq/disc_gen 供重连后比较） */
                xbox_state_t st;
                if (xQueuePeek(xbox_state_queue, &st, 0) != pdTRUE)
                    memset(&st, 0, sizeof(st));
                st.connected = 0;
                xQueueOverwrite(xbox_state_queue, &st);
            } else {
                /* 静默时也在截止时间醒来，不再额外等满一次 100ms。 */
                const uint32_t remaining_ms = C3_LINK_TIMEOUT_MS - age_ms;
                if (remaining_ms < wait_ms) wait_ms = remaining_ms;
            }
        }

        uint8_t b;
        if (uart_read_bytes(PEO_PM_C3_UART, &b, 1, pdMS_TO_TICKS(wait_ms)) != 1) {
            /* 读超时/驱动错误：清候选帧；下一轮仍先检查有效帧截止时间。 */
            s_rx_count = 0;
            continue;
        }

        /* 有界滑动候选帧：CRC 失败保留后缀，防止载荷内嵌 AA 55 干扰重同步 */
        configASSERT(s_rx_count < sizeof(s_rx_buf));
        s_rx_buf[s_rx_count++] = b;
        while (s_rx_count != 0) {
            bool reject = s_rx_buf[0] != ESP_LINK_SOF0;
            if (!reject && s_rx_count >= 2) reject = s_rx_buf[1] != ESP_LINK_SOF1;
            if (!reject && s_rx_count >= 3) {
                const uint8_t payload_len = s_rx_buf[2];
                reject = (payload_len == 0) || (payload_len > C3_RX_MAX_PAYLOAD);
                if (!reject) {
                    const size_t frame_total = 3 + (size_t)payload_len + 2;
                    if (s_rx_count >= frame_total) {
                        const size_t crc_offset = 3 + (size_t)payload_len;
                        const uint16_t received = (uint16_t)s_rx_buf[crc_offset] |
                                                  (uint16_t)(s_rx_buf[crc_offset + 1] << 8);
                        if (c3_crc16(s_rx_buf, (uint16_t)crc_offset) == received) {
                            handle_frame(&s_rx_buf[3], payload_len);
                            s_rx_count = 0;
                            break;
                        }
                        reject = true;
                    }
                }
            }
            if (!reject) break;  /* 保留部分候选帧 */
            --s_rx_count;
            memmove(s_rx_buf, s_rx_buf + 1, s_rx_count);
        }
    }
}
