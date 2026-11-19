/**
 * @file  c3_uart.cpp
 * @brief UART link implementation: CRC16 calculation + gamepad frame packing/sending.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

/**
 * CRC16-CCITT (polynomial 0x1021, init 0xFFFF, table-driven) matches esp_link
 * exactly. Standard test vector: crc("123456789") == 0x29B1.
 * Frame format: SOF(0xAA 0x55) + LEN(1) + PAYLOAD + CRC16(2), little-endian,
 * CRC low byte first.
 */

#include "c3_uart.h"
#include "PEO_PINMAP.h"

#include <driver/uart.h>
#include <esp_log.h>
#include <string.h>

/** Log tag shown in the serial console for messages from this file. */
static const char *TAG = "c3_uart";

/** Pre-computed CRC lookup table (filled lazily by crc_build_table). */
static uint16_t s_crc_table[256];
/** True once the lookup table has been built. */
static bool     s_crc_ready = false;

/**
 * @brief  Build the 256-entry CRC lookup table (called once, lazily).
 *
 * Each entry is the CRC remainder of an 8-bit value shifted into the top byte,
 * which turns the bit-by-bit CRC into a fast byte-at-a-time table lookup.
 */
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

/**
 * @brief  Compute the CRC16-CCITT of a byte buffer.
 * @param  data  Pointer to the bytes to checksum (may be NULL if len == 0).
 * @param  len   Number of bytes to checksum.
 * @retval The 16-bit CRC value (CCITT-FALSE / IBM-3740 variant).
 */
static uint16_t c3_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    if (!s_crc_ready)
        crc_build_table();

    while (len--)
        crc = (uint16_t)((crc << 8) ^ s_crc_table[((crc >> 8) ^ *data++) & 0xFFU]);
    return crc;
}

/**
 * @brief  Initialise the UART peripheral and pre-build the CRC table.
 *
 * Configures the link UART as 8-N-1 with no flow control (matching the F407
 * USART3 settings), installs the driver and routes the TX/RX pins from
 * PEO_PINMAP.h.
 */
void c3_uart_init(void)
{
    // Compile-time guarantee: the packed payload must be exactly 26 bytes,
    // otherwise the frame layout would no longer match PROTOCOL.md §1.4.
    static_assert(sizeof(c3_gamepad_payload_t) == 26,
                  "gamepad payload must be 26B (PROTOCOL.md §1.4)");

    // UART settings: 8 data bits, no parity, 1 stop bit, no flow control.
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

/**
 * @brief  Pack and send one GAMEPAD_STATE frame over the UART.
 *
 * Builds the full 32-byte frame (SOF2 + LEN1 + type1 + body26 + CRC2), computes
 * the CRC over SOF..body, then writes it and waits for the UART TX buffer to drain.
 *
 * @param  p  Pointer to the payload to send (must not be NULL).
 * @retval true   All 32 bytes written and the UART TX buffer flushed.
 * @retval false  Send failed (NULL payload, short write, or TX drain timed out).
 */
bool c3_uart_send_gamepad(const c3_gamepad_payload_t *p)
{
    if (p == NULL)
        return false;

    uint8_t frame[3 + 1 + sizeof(c3_gamepad_payload_t) + 2];   /* 32B: SOF2 + LEN1 + type1 + body26 + CRC2 */
    frame[0] = C3_FRAME_SOF0;
    frame[1] = C3_FRAME_SOF1;
    frame[2] = (uint8_t)(1 + sizeof(c3_gamepad_payload_t));    /* LEN = 27 = type + body */
    frame[3] = C3_TYPE_GAMEPAD_STATE;
    memcpy(&frame[4], p, sizeof(c3_gamepad_payload_t));

    const uint16_t crc = c3_crc16(frame, (uint16_t)(4 + sizeof(c3_gamepad_payload_t)));
    frame[sizeof(frame) - 2] = (uint8_t)crc;         /* frame[30] = CRC low byte */
    frame[sizeof(frame) - 1] = (uint8_t)(crc >> 8);  /* frame[31] = CRC high byte */

    const int sent = uart_write_bytes(PEO_PM_C3_UART, frame, sizeof(frame));
    const esp_err_t drained = uart_wait_tx_done(PEO_PM_C3_UART, pdMS_TO_TICKS(100));
    return sent == (int)sizeof(frame) && drained == ESP_OK;
}
