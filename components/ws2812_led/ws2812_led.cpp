#include "ws2812_led.h"
#include "PEO_PINMAP.h"
#include "esp_net.h"

#include <driver/rmt_tx.h>
#include <esp_err.h>
#include <esp_log.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>

static const char *TAG = "ws2812";

/* 状态灯配色（可按喜好改）：
   呼吸 = 蓝色（未连上热点），常亮 = 绿色（已连上），闪光 = 绿色快闪 3 次 */
#define WS2812_BREATH_R   0
#define WS2812_BREATH_G   100
#define WS2812_BREATH_B   255
#define WS2812_STEADY_R   0
#define WS2812_STEADY_G   255
#define WS2812_STEADY_B   0
#define WS2812_TICK_MS    50      /* 状态机节拍：50ms */
#define WS2812_FLASH_MS   150     /* 连接闪光亮/灭各 150ms */

/* 1 深 latest-wins：灯只关心最终颜色，旧命令被最新命令覆盖 */
static QueueHandle_t s_cmd_queue = NULL;
static rmt_channel_handle_t s_tx_chan = NULL;
static rmt_encoder_handle_t s_encoder = NULL;

void ws2812_queues_init(void)
{
    if (!s_cmd_queue)
        s_cmd_queue = xQueueCreate(1, sizeof(ws2812_cmd_t));
    if (!s_cmd_queue)
        ESP_LOGE(TAG, "cmd queue create failed");
}

/* 亮度 0-100% → 0-255 比例缩放 */
static inline uint8_t scale(uint8_t v, uint8_t brightness)
{
    return (uint8_t)(((uint16_t)v * (uint16_t)brightness) / 100u);
}

/* 发送一次（GRB 顺序 + 亮度缩放）。帧间低电平间隔由调用方节拍保证（>50us RESET）。 */
static void led_show(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
    uint8_t grb[3] = {
        scale(g, brightness),
        scale(r, brightness),
        scale(b, brightness),
    };
    rmt_transmit_config_t tx = {0};   /* loop_count=0：发送一次 */
    if (rmt_transmit(s_tx_chan, s_encoder, grb, sizeof(grb), &tx) == ESP_OK)
        rmt_tx_wait_all_done(s_tx_chan, pdMS_TO_TICKS(10));
}

/* RMT TX：10MHz 分辨率（1 tick = 100ns）。WS2812 标准时序（GRB，800kHz）：
   T0H 0.4us / T0L 0.9us，T1H 0.8us / T1L 0.5us，均落 ±150ns 容差内。 */
static bool rmt_init(void)
{
    rmt_tx_channel_config_t ch = {
        .gpio_num = PEO_PM_WS2812_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,      /* 80MHz APB */
        .resolution_hz = 10000000,
        .mem_block_symbols = 64,             /* 1 颗 WS2812 = 24 bit = 24 RMT symbols */
        .trans_queue_depth = 1,
    };
    rmt_bytes_encoder_config_t enc = {
        .bit0 = { .duration0 = 4, .level0 = 1, .duration1 = 9, .level1 = 0 },
        .bit1 = { .duration0 = 8, .level0 = 1, .duration1 = 5, .level1 = 0 },
        .flags = { .msb_first = 1 },          /* WS2812: G7..G0, R7..R0, B7..B0 */
    };
    if (rmt_new_tx_channel(&ch, &s_tx_chan) != ESP_OK)
        return false;
    if (rmt_new_bytes_encoder(&enc, &s_encoder) != ESP_OK)
    {
        rmt_del_channel(s_tx_chan);
        s_tx_chan = NULL;
        return false;
    }
    if (rmt_enable(s_tx_chan) != ESP_OK)
        return false;
    return true;
}

void ws2812_task(void *arg)
{
    (void)arg;
    if (!s_cmd_queue)
    {
        ESP_LOGE(TAG, "queue missing; task exits");
        vTaskDelete(NULL);
        return;
    }
    if (!rmt_init())
    {
        ESP_LOGE(TAG, "RMT init failed; task exits");
        vTaskDelete(NULL);
        return;
    }
    EventGroupHandle_t eg = esp_net_conn_event();
    if (!eg)
    {
        ESP_LOGE(TAG, "connection event group missing; task exits");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "task up (RMT GPIO%u, status LED)", (unsigned)PEO_PM_WS2812_GPIO);

    /* 呼吸正弦亮度表：32 步 × 50ms = 1.6s 一个周期 */
    static uint8_t breath_lut[32];
    for (int i = 0; i < 32; i++)
        breath_lut[i] = (uint8_t)(((sinf((float)i * 2.0f * (float)M_PI / 32.0f) + 1.0f) / 2.0f) * 100.0f);

    bool sta = false;
    uint8_t breath_i = 0;
    ws2812_cmd_t manual = {0};
    bool has_manual = false;

    for (;;)
    {
        /* 50ms 节拍：STA 位有变化立刻醒，否则到点醒 */
        xEventGroupWaitBits(eg, PEO_NET_EVT_STA, pdFALSE, pdFALSE,
                            pdMS_TO_TICKS(WS2812_TICK_MS));
        bool sta_now = (xEventGroupGetBits(eg) & PEO_NET_EVT_STA) != 0;

        if (sta_now && !sta)
        {
            /* 连接沿：绿色快闪 3 次 → 常亮 */
            ESP_LOGI(TAG, "hotspot connected; flash then steady");
            for (int k = 0; k < 3; k++)
            {
                led_show(WS2812_STEADY_R, WS2812_STEADY_G, WS2812_STEADY_B, 100);
                vTaskDelay(pdMS_TO_TICKS(WS2812_FLASH_MS));
                led_show(0, 0, 0, 100);
                vTaskDelay(pdMS_TO_TICKS(WS2812_FLASH_MS));
            }
            has_manual = false;
        }
        else if (!sta_now && sta)
        {
            ESP_LOGI(TAG, "hotspot lost; back to breathing");
            has_manual = false;   /* 断连回呼吸：放弃手动颜色 */
        }
        sta = sta_now;

        /* 手动命令（仅连接后由上位机发）：latest-wins 覆盖，直到连接状态变化 */
        ws2812_cmd_t cmd;
        if (xQueueReceive(s_cmd_queue, &cmd, 0) == pdTRUE)
        {
            manual = cmd;
            has_manual = true;
        }

        if (sta_now)
        {
            if (has_manual)
                led_show(manual.r, manual.g, manual.b, manual.brightness);
            else
                led_show(WS2812_STEADY_R, WS2812_STEADY_G, WS2812_STEADY_B, 100);
        }
        else
        {
            led_show(WS2812_BREATH_R, WS2812_BREATH_G, WS2812_BREATH_B, breath_lut[breath_i]);
            breath_i = (uint8_t)((breath_i + 1) & 31);
        }
    }
}

bool ws2812_set_color(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
    if (!s_cmd_queue)
        return false;
    ws2812_cmd_t cmd = {
        .r = r, .g = g, .b = b,
        .brightness = (uint8_t)(brightness > 100 ? 100 : brightness),
    };
    /* 1 深队列 + overwrite：满时覆盖为最新值，调用方永不阻塞 */
    return xQueueOverwrite(s_cmd_queue, &cmd) == pdTRUE;
}
