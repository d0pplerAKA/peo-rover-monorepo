#include "ws2812_led.h"
#include "PEO_PINMAP.h"

#include <driver/rmt_tx.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

static const char *TAG = "ws2812";

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
    ESP_LOGI(TAG, "task up (RMT GPIO%u)", (unsigned)PEO_PM_WS2812_GPIO);

    for (;;)
    {
        ws2812_cmd_t cmd;
        if (xQueueReceive(s_cmd_queue, &cmd, portMAX_DELAY) != pdTRUE)
            continue;

        /* WS2812 数据顺序为 GRB */
        uint8_t grb[3] = {
            scale(cmd.g, cmd.brightness),
            scale(cmd.r, cmd.brightness),
            scale(cmd.b, cmd.brightness),
        };
        rmt_transmit_config_t tx = {0};   /* loop_count=0：发送一次 */
        if (rmt_transmit(s_tx_chan, s_encoder, grb, sizeof(grb), &tx) == ESP_OK)
            rmt_tx_wait_all_done(s_tx_chan, pdMS_TO_TICKS(10));

        /* 命令节奏天然 >50us（WS2812 RESET code 要求帧间低电平），
           补 1ms 保守保证。 */
        vTaskDelay(pdMS_TO_TICKS(1));
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
