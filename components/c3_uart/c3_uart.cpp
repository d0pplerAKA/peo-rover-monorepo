/**
 * @file    c3_uart.cpp
 * @brief   PEO Rover - ESP32-C3 UART 链路实现：CRC16 + 状态帧打包发送
 *
 * CRC16-CCITT（多项式 0x1021，初始 0xFFFF，查表法）与 esp_link 完全一致，
 * 标准测试向量 crc("123456789") == 0x29B1。
 * 帧格式：SOF(0xAA 0x55) + LEN(1) + PAYLOAD + CRC16(2)，小端，CRC 低字节在前。
 */

#include "c3_uart.h"
#include "PEO_PINMAP.h"

#include <driver/uart.h>
#include <esp_log.h>
#include <string.h>

static const char *TAG = "c3_uart";

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

void c3_uart_init(void)
{
    static_assert(sizeof(c3_gamepad_payload_t) == 26,
                  "gamepad payload must be 26B (PROTOCOL.md §1.4)");

    /* UART 初始化（8-N-1，无流控，与 F407 端 USART3 一致） */
    uart_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.baud_rate  = PEO_PM_C3_BAUDRATE;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_driver_install(PEO_PM_C3_UART, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PEO_PM_C3_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(PEO_PM_C3_UART, PEO_PM_C3_TX, PEO_PM_C3_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    crc_build_table();
    ESP_LOGI(TAG, "uart up (UART%d, %u baud)", PEO_PM_C3_UART,
             (unsigned)PEO_PM_C3_BAUDRATE);
}

bool c3_uart_send_gamepad(const c3_gamepad_payload_t *p)
{
    if (p == NULL)
        return false;

    uint8_t frame[3 + 1 + sizeof(c3_gamepad_payload_t) + 2];   /* 32B: SOF2+LEN1+type1+body26+CRC2 */
    frame[0] = C3_FRAME_SOF0;
    frame[1] = C3_FRAME_SOF1;
    frame[2] = (uint8_t)(1 + sizeof(c3_gamepad_payload_t));    /* LEN = 27 = type+body */
    frame[3] = C3_TYPE_GAMEPAD_STATE;
    memcpy(&frame[4], p, sizeof(c3_gamepad_payload_t));

    const uint16_t crc = c3_crc16(frame, (uint16_t)(4 + sizeof(c3_gamepad_payload_t)));
    frame[sizeof(frame) - 2] = (uint8_t)crc;         /* frame[30] CRC 低字节 */
    frame[sizeof(frame) - 1] = (uint8_t)(crc >> 8);  /* frame[31] CRC 高字节 */

    const int sent = uart_write_bytes(PEO_PM_C3_UART, frame, sizeof(frame));
    const esp_err_t drained = uart_wait_tx_done(PEO_PM_C3_UART, pdMS_TO_TICKS(100));
    return sent == (int)sizeof(frame) && drained == ESP_OK;
}
