/**
 ******************************************************************************
 * @file    usart.h
 * @brief   PEO Rover - USART component public API
 *
 * One UART is provided by this component:
 *   1. DEBUG console  - USART1, PA9/PA10, external USB-TTL adapter
 *                       (blocking, printf-retarget friendly)
 *
 * All pins/bauds come from PEO_PINMAP.H macros - do not hardcode them here.
 ******************************************************************************
 */
#ifndef __USART_H__
#define __USART_H__

#include "stm32f4xx.h"
#include "COMPONENTS/PEO_PINMAP.H"

/* ============================================================================
   DEBUG CONSOLE (USART1, PA9/PA10)
   ============================================================================ */
void    USART_DBG_Init(uint32_t baud);                       /* e.g. 115200 */
void    USART_DBG_SendByte(uint8_t ch);                      /* block on TXE   */
void    USART_DBG_SendBuf(const uint8_t *buf, uint16_t len); /* block, raw bytes */
void    USART_DBG_SendStr(const char *str);                  /* block, C string  */
uint8_t USART_DBG_GetByte(uint8_t *ch);                      /* 1 = byte got (poll RXNE) */

#endif /* __USART_H__ */
