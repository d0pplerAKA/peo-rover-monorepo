/**
 * @file  usart_dbg.c
 * @brief Send diagnostics and standard-library console I/O through USART1.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * External USB-TTL adapter on 3V3 logic, 115200-8-N-1 by default.
 * All pin macros come from PEO_PINMAP.H (section 3).
 *
 * printf() RETARGET:
 *   fputc is redirected to this UART. With Keil, use the MicroLIB option
 *   (Options -> Target -> "Use MicroLIB") for a small footprint; without
 *   MicroLIB the standard stdio also works but pulls in more code.
 *   Enable by just calling printf() anywhere - fputc below handles it.
 */
#include "include/usart.h"

#include <stdio.h>

/* ----------------------------------------------------------------------------
 * Public: initialize USART1 (GPIO + peripheral)
 * -------------------------------------------------------------------------- */
/**
 * @brief Prepare the USB-to-TTL debug console on USART1.
 *
 * Called by main after RCC clock setup. Uses PA9 TX and PA10 RX, eight data bits, no
 * parity, one stop bit, and polling I/O.
 * @param baud Requested serial baud rate in bits/s, normally 115200.
 * @retval None This function does not return a value.
 */
void USART_DBG_Init(uint32_t baud)
{
    /* NOTE: peripheral clocks (GPIOA, USART1) are enabled centrally by
       RCC_Clock_Init() in COMPONENTS/RCC - call it in main() first. */
    GPIO_InitTypeDef  gpio;  /* GPIO setup record: selects pin routing, electrical drive, pulls, and switching speed. */
    USART_InitTypeDef usart;  /* Serial-port setup record: baud rate, byte format, and enabled directions. */

    /* PA9 = USART1_TX, PA10 = USART1_RX, AF7 */
    gpio.GPIO_Pin   = PEO_PM_DBG_TX_PIN | PEO_PM_DBG_RX_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;        /* RX idle high */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PEO_PM_DBG_TX_PORT, &gpio);
    GPIO_PinAFConfig(PEO_PM_DBG_TX_PORT, PEO_PM_DBG_TX_SRC, PEO_PM_DBG_AF);
    GPIO_PinAFConfig(PEO_PM_DBG_RX_PORT, PEO_PM_DBG_RX_SRC, PEO_PM_DBG_AF);

    /* 8-N-1, no flow control */
    usart.USART_BaudRate            = baud;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(PEO_PM_DBG_USART, &usart);
    USART_Cmd(PEO_PM_DBG_USART, ENABLE);
}

/* ----------------------------------------------------------------------------
 * Blocking transmit
 * -------------------------------------------------------------------------- */
/**
 * @brief Place one byte into the debug serial transmitter when it is ready.
 *
 * Used by the string/buffer helpers and printf retargeting. Blocks until the transmit
 * data register is empty, but does not wait for the final stop bit on the wire.
 * @param ch Byte value to transmit (0..255).
 * @retval None This function does not return a value.
 */
void USART_DBG_SendByte(uint8_t ch)
{
    while (USART_GetFlagStatus(PEO_PM_DBG_USART, USART_FLAG_TXE) == RESET)
        ;
    USART_SendData(PEO_PM_DBG_USART, ch);
}

/**
 * @brief Send an exact number of bytes to the debug console.
 *
 * Available to application diagnostics; calls the blocking byte helper for each element,
 * including zero-valued bytes.
 * @param buf Readable buffer containing at least len bytes.
 * @param len Number of bytes to transmit; zero sends nothing.
 * @retval None This function does not return a value.
 */
void USART_DBG_SendBuf(const uint8_t *buf, uint16_t len)
{
    while (len--)
    {
        USART_DBG_SendByte(*buf++);
    }
}

/**
 * @brief Send readable text up to its terminating zero character.
 *
 * Available to application diagnostics; calls the byte helper until the end of the C
 * string.
 * @param str Non-NULL, zero-terminated text; the terminating zero is not transmitted.
 * @retval None This function does not return a value.
 */
void USART_DBG_SendStr(const char *str)
{
    while (*str)
    {
        USART_DBG_SendByte((uint8_t)*str++);
    }
}

/* ----------------------------------------------------------------------------
 * Non-blocking polling receive: returns 1 if a byte was available
 * -------------------------------------------------------------------------- */
/**
 * @brief Check for one incoming debug-console byte without waiting.
 *
 * Used by fgetc and available to polling application code. Reads RXNE once and leaves
 * the output unchanged if no byte is ready.
 * @param ch Non-NULL writable pointer receiving the byte on success.
 * @retval 1 One byte was read and stored.
 * @retval 0 No byte was ready; ch was unchanged.
 */
uint8_t USART_DBG_GetByte(uint8_t *ch)
{
    if (USART_GetFlagStatus(PEO_PM_DBG_USART, USART_FLAG_RXNE) != RESET)
    {
        *ch = (uint8_t)USART_ReceiveData(PEO_PM_DBG_USART);
        return 1;
    }
    return 0;
}

/* ----------------------------------------------------------------------------
 * printf() retarget (ARMCC / GCC / IAR)
 * ----------------------------------------------------------------------------
 * fputc redirects stdout to the debug UART, so printf() prints on PA9/PA10.
 * fgetc redirects stdin, so scanf()/getchar() read from the same UART.
 *
 * Keil configuration:
 *   Options for Target -> Target tab -> check "Use MicroLIB"
 *   (with MicroLIB only fputc/fgetc are needed; without it the full stdio
 *    pulls in semihosting stubs and needs extra retarget symbols).
 * -------------------------------------------------------------------------- */
#if defined(__CC_ARM) || defined(__GNUC__) || defined(__ICCARM__)

/**
 * @brief Route a standard-library output character to the debug UART.
 *
 * Called by printf/stdio retargeting in the configured toolchain. Inserts a carriage
 * return before newline for serial terminals.
 * @param ch Character value to send; transmission uses its low eight bits.
 * @param f Standard-library stream argument; ignored because this firmware uses one
 *          console.
 * @retval value The original ch value after requesting transmission.
 */
int fputc(int ch, FILE *f)
{
    (void)f;
    if (ch == '\n')
    {
        USART_DBG_SendByte((uint8_t)'\r');  /* CRLF: keep terminals in line */
    }
    USART_DBG_SendByte((uint8_t)ch);
    return ch;
}

/**
 * @brief Wait for a standard-library input character from the debug UART.
 *
 * Called by getchar/scanf retargeting. Busy-waits until a byte arrives; use from
 * ordinary code, never from an interrupt handler.
 * @param f Standard-library stream argument; ignored because there is one debug console.
 * @retval value Received unsigned byte promoted to int (0..255).
 */
int fgetc(FILE *f)
{
    uint8_t ch;  /* One received console byte (0..255). */
    (void)f;
    while (USART_DBG_GetByte(&ch) == 0)
    {
        /* block until a byte arrives (scanf/getchar only - never call
           from interrupt context) */
    }
    return (int)ch;
}

#endif /* __CC_ARM || __GNUC__ || __ICCARM__ */
