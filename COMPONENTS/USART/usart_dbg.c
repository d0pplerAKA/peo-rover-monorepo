/**
 ******************************************************************************
 * @file    usart_dbg.c
 * @brief   PEO Rover - debug console driver (USART1, PA9/PA10, AF7)
 *
 * External USB-TTL adapter on 3V3 logic, 115200-8-N-1 by default.
 * All pin macros come from PEO_PINMAP.H (section 3).
 *
 * printf() RETARGET:
 *   fputc is redirected to this UART. With Keil, use the MicroLIB option
 *   (Options -> Target -> "Use MicroLIB") for a small footprint; without
 *   MicroLIB the standard stdio also works but pulls in more code.
 *   Enable by just calling printf() anywhere - fputc below handles it.
 ******************************************************************************
 */
#include "include/usart.h"

#include <stdio.h>

/* ----------------------------------------------------------------------------
 * Public: initialize USART1 (GPIO + peripheral)
 * -------------------------------------------------------------------------- */
void USART_DBG_Init(uint32_t baud)
{
    /* NOTE: peripheral clocks (GPIOA, USART1) are enabled centrally by
       RCC_Clock_Init() in COMPONENTS/RCC - call it in main() first. */
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;

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
void USART_DBG_SendByte(uint8_t ch)
{
    while (USART_GetFlagStatus(PEO_PM_DBG_USART, USART_FLAG_TXE) == RESET)
        ;
    USART_SendData(PEO_PM_DBG_USART, ch);
}

void USART_DBG_SendBuf(const uint8_t *buf, uint16_t len)
{
    while (len--)
    {
        USART_DBG_SendByte(*buf++);
    }
}

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

int fgetc(FILE *f)
{
    uint8_t ch;
    (void)f;
    while (USART_DBG_GetByte(&ch) == 0)
    {
        /* block until a byte arrives (scanf/getchar only - never call
           from interrupt context) */
    }
    return (int)ch;
}

#endif /* __CC_ARM || __GNUC__ || __ICCARM__ */
