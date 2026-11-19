/**
 * @file  usart.h
 * @brief Expose the polling debug-console interface on USART1.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * One UART is provided by this component:
 *   1. DEBUG console  - USART1, PA9/PA10, external USB-TTL adapter
 *                       (blocking, printf-retarget friendly)
 *
 * All pins/bauds come from PEO_PINMAP.H macros - do not hardcode them here.
 */
#ifndef __USART_H__
#define __USART_H__

#include "stm32f4xx.h"
#include "COMPONENTS/PEO_PINMAP.H"

/* ============================================================================
   DEBUG CONSOLE (USART1, PA9/PA10)
   ============================================================================ */
/**
 * @brief Prepare the USB-to-TTL debug console on USART1.
 *
 * Called by main after RCC clock setup. Uses PA9 TX and PA10 RX, eight data bits, no
 * parity, one stop bit, and polling I/O.
 * @param baud Requested serial baud rate in bits/s, normally 115200.
 * @retval None This function does not return a value.
 */
void    USART_DBG_Init(uint32_t baud);                       /* e.g. 115200 */
/**
 * @brief Place one byte into the debug serial transmitter when it is ready.
 *
 * Used by the string/buffer helpers and printf retargeting. Blocks until the transmit
 * data register is empty, but does not wait for the final stop bit on the wire.
 * @param ch Byte value to transmit (0..255).
 * @retval None This function does not return a value.
 */
void    USART_DBG_SendByte(uint8_t ch);                      /* block on TXE   */
/**
 * @brief Send an exact number of bytes to the debug console.
 *
 * Available to application diagnostics; calls the blocking byte helper for each element,
 * including zero-valued bytes.
 * @param buf Readable buffer containing at least len bytes.
 * @param len Number of bytes to transmit; zero sends nothing.
 * @retval None This function does not return a value.
 */
void    USART_DBG_SendBuf(const uint8_t *buf, uint16_t len); /* block, raw bytes */
/**
 * @brief Send readable text up to its terminating zero character.
 *
 * Available to application diagnostics; calls the byte helper until the end of the C
 * string.
 * @param str Non-NULL, zero-terminated text; the terminating zero is not transmitted.
 * @retval None This function does not return a value.
 */
void    USART_DBG_SendStr(const char *str);                  /* block, C string  */
/**
 * @brief Check for one incoming debug-console byte without waiting.
 *
 * Used by fgetc and available to polling application code. Reads RXNE once and leaves
 * the output unchanged if no byte is ready.
 * @param ch Non-NULL writable pointer receiving the byte on success.
 * @retval 1 One byte was read and stored.
 * @retval 0 No byte was ready; ch was unchanged.
 */
uint8_t USART_DBG_GetByte(uint8_t *ch);                      /* 1 = byte got (poll RXNE) */

#endif /* __USART_H__ */
