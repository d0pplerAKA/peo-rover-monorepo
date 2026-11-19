/**
 * @file  c3_uart.h
 * @brief UART link that sends Xbox controller state frames from the C3 to the F407.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

#ifndef __C3_UART_H__
#define __C3_UART_H__

/**
 * The C3 talks to the F407 over a wired UART (direct to F407 USART3,
 * PROTOCOL.md §1.4).
 *
 * The frame format is identical to the esp_link protocol:
 *   SOF(0xAA 0x55) + LEN(1 byte) + PAYLOAD + CRC16-CCITT(2 bytes)
 * All multi-byte values are little-endian. The CRC covers everything from SOF
 * through PAYLOAD, and is appended with the low byte first.
 *
 * PAYLOAD = type(1 byte) + body. The only type today is GAMEPAD_STATE (0x10):
 *   body = c3_gamepad_payload_t (26 bytes) -> payload 27 bytes, frame 32 bytes total.
 * The frame is sent at 100 Hz and doubles as the C3 heartbeat (the F407 stops the
 * rover if it sees no frame for 500 ms = C3 link lost).
 */

#include <stdint.h>
#include <stdbool.h>

/** First byte of every frame's start-of-frame marker. */
#define C3_FRAME_SOF0   0xAA
/** Second byte of every frame's start-of-frame marker. */
#define C3_FRAME_SOF1   0x55

/** Payload type for a gamepad/controller state frame. */
#define C3_TYPE_GAMEPAD_STATE   0x10

/**
 * @brief  Body of a GAMEPAD_STATE frame (26 bytes, packed to match the
 *         PROTOCOL.md §1.4 offset table byte-for-byte).
 */
typedef struct __attribute__((packed))
{
    uint32_t seq;        /**< Valid HID notification counter (increments across reconnects; battery / short reports do NOT increment it). */
    uint32_t disc_gen;   /**< Disconnect generation (increments each time the BLE link drops). */
    uint8_t  connected;  /**< 0 = not connected, 1 = connected and has data. */
    uint8_t  battery;    /**< Battery level, 0..100 (percent). */
    uint8_t  input_fresh;/**< 0/1: a valid HID notification arrived within the last 5 s (judged locally on the C3; always 0 when disconnected). */
    uint16_t joy_lx;     /**< Left stick horizontal, 0..65535, center 32768. */
    uint16_t joy_ly;     /**< Left stick vertical, 0..65535, center 32768. */
    uint16_t joy_rx;     /**< Right stick horizontal, 0..65535, center 32768. */
    uint16_t joy_ry;     /**< Right stick vertical, 0..65535, center 32768. */
    uint16_t trigger_lt; /**< Left trigger, 0..1023. */
    uint16_t trigger_rt; /**< Right trigger, 0..1023. */
    uint16_t buttons;    /**< Button bitmap (XB_BTN_* bits, see xbox_service.h). */
    uint8_t  lock;       /**< Control lock: 0 = closed (manual stick control), 1 = open (autonomous, upper computer controls). */
} c3_gamepad_payload_t;

/**
 * @brief  Initialise the UART (8-N-1, no flow control; baud rate and pins are
 *         defined in PEO_PINMAP.h).
 */
void c3_uart_init(void);

/**
 * @brief  Pack and send one GAMEPAD_STATE frame over the UART.
 * @param  p  Pointer to the payload to send (must not be NULL).
 * @retval true   All 32 bytes written and the UART TX buffer flushed.
 * @retval false  Send failed (NULL payload, short write, or TX drain timed out).
 */
bool c3_uart_send_gamepad(const c3_gamepad_payload_t *p);

#endif /* __C3_UART_H__ */
