/**
 ******************************************************************************
 * @file    usart.h
 * @brief   PEO Rover - USART component public API
 *
 * Two UARTs are provided by this component:
 *   1. DEBUG console  - USART1, PA9/PA10, external USB-TTL adapter
 *                       (blocking, printf-retarget friendly)
 *   2. ESP32 link     - USART2, PD5/PD6, 4.2 Mbps, DMA + IDLE-line framing
 *                       (see PEO_PINMAP.H section 4 + 15, COMPONENTS/NVIC/include/nvic.h)
 *
 * All pins/bauds come from PEO_PINMAP.H macros - do not hardcode them here.
 ******************************************************************************
 */
#ifndef __USART_H__
#define __USART_H__

#include "stm32f4xx.h"
#include "PEO_PINMAP.H"

/* ============================================================================
   DEBUG CONSOLE (USART1, PA9/PA10)
   ============================================================================ */
void    USART_DBG_Init(uint32_t baud);                       /* e.g. 115200 */
void    USART_DBG_SendByte(uint8_t ch);                      /* block on TXE   */
void    USART_DBG_SendBuf(const uint8_t *buf, uint16_t len); /* block, raw bytes */
void    USART_DBG_SendStr(const char *str);                  /* block, C string  */
uint8_t USART_DBG_GetByte(uint8_t *ch);                      /* 1 = byte got (poll RXNE) */

/* ============================================================================
   ESP32 LINK (USART2, PD5/PD6, DMA TX + circular DMA RX + IDLE framing)
   ----------------------------------------------------------------------------
   RX model:
     - DMA1_Stream5 in CIRCULAR mode fills a fixed-size ring buffer.
     - The USART2 IDLE interrupt marks frame boundaries: when the line goes
       idle, USART_COMM_FramePending() returns 1 and USART_COMM_FrameLen()
       reports how many bytes the frame contains. Drain with
       USART_COMM_Read() (non-blocking, copies up to `max` bytes).
   TX model:
     - DMA1_Stream6 in normal mode. USART_COMM_Send() returns 0 while the
       previous transfer is still running - check USART_COMM_TxBusy().
   ============================================================================ */
#define PEO_COMM_RX_RING_SIZE   (512U)      /* RX ring buffer in bytes */

void     USART_COMM_Init(void);
uint8_t  USART_COMM_Send(const uint8_t *buf, uint16_t len);  /* 1 = started, 0 = busy */
uint8_t  USART_COMM_TxBusy(void);
uint16_t USART_COMM_RxAvailable(void);      /* unread bytes in ring */
uint16_t USART_COMM_Read(uint8_t *dst, uint16_t max);         /* returns bytes copied */
uint8_t  USART_COMM_FramePending(void);     /* IDLE-line frame ready flag */
uint16_t USART_COMM_FrameLen(void);         /* byte count of the last frame */
void     USART_COMM_ClearFrame(void);

#endif /* __USART_H__ */
