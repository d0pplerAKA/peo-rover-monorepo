/**
 ******************************************************************************
 * @file    usart_comm.c
 * @brief   PEO Rover - ESP32 high-speed link driver
 *          (USART2, PD5/PD6, AF7, 4.2 Mbps, DMA)
 *
 * Design (see PEO_PINMAP.H section 4 + 15, COMPONENTS/NVIC/include/nvic.h):
 *   RX: DMA1_Stream5 / Channel 4, CIRCULAR mode, fixed ring buffer.
 *       No RX interrupt flooding: frame boundaries are detected by the
 *       USART2 IDLE-line interrupt. The stream TC interrupt only counts
 *       full loop rollovers so the absolute write position stays correct.
 *   TX: DMA1_Stream6 / Channel 4, normal mode, TC interrupt signals done.
 *
 * Baud check: USART2 on APB1 = 42 MHz. USARTDIV = 42e6 / 4.2e6 = 10 exactly
 *             -> 0% baud error (one reason this UART is the ESP32 link).
 ******************************************************************************
 */
#include "include/usart.h"

/* ----------------------------------------------------------------------------
 * RX ring buffer and absolute position counters
 * -------------------------------------------------------------------------- */
static uint8_t            rx_ring[PEO_COMM_RX_RING_SIZE];
static volatile uint32_t  rx_rollover;     /* +1 every full DMA loop (ISR)   */
static volatile uint32_t  rx_read_pos;     /* absolute bytes already drained */
static volatile uint16_t  rx_frame_len;    /* last IDLE frame byte count     */
static volatile uint8_t   rx_frame_pending;
static volatile uint8_t   tx_busy;

/* Internal: absolute write position = rollover*RING + (RING - NDTR).
   NOTE: call with interrupts disabled or only from IRQ-safe context when
   racing the DMA TC ISR. */
static uint32_t comm_get_write_pos(void)
{
    uint32_t ndtr = DMA_GetCurrDataCounter(PEO_DMA_COMM_RX_STREAM);
    return rx_rollover * (uint32_t)PEO_COMM_RX_RING_SIZE
         + ((uint32_t)PEO_COMM_RX_RING_SIZE - ndtr);
}

/* ----------------------------------------------------------------------------
 * Public: initialize USART2 + DMA1 Stream5/6 + NVIC
 * -------------------------------------------------------------------------- */
void USART_COMM_Init(void)
{
    /* NOTE: peripheral clocks (GPIOD, USART2, DMA1) and the NVIC channels
       are configured centrally - RCC_Clock_Init() (COMPONENTS/RCC) and
       NVIC_InitAll() (COMPONENTS/IRQ) must run in main() before this. */
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    DMA_InitTypeDef   dma;

    /* PD5 = USART2_TX, PD6 = USART2_RX, AF7 */
    gpio.GPIO_Pin   = PEO_PM_COMM_TX_PIN | PEO_PM_COMM_RX_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;         /* RX idle high */
    gpio.GPIO_Speed = GPIO_Speed_100MHz;    /* 4.2 Mbps needs fast slew */
    GPIO_Init(PEO_PM_COMM_TX_PORT, &gpio);
    GPIO_PinAFConfig(PEO_PM_COMM_TX_PORT, PEO_PM_COMM_TX_SRC, PEO_PM_COMM_AF);
    GPIO_PinAFConfig(PEO_PM_COMM_RX_PORT, PEO_PM_COMM_RX_SRC, PEO_PM_COMM_AF);

    /* 4.2 Mbps, 8-N-1, no flow control */
    usart.USART_BaudRate            = PEO_PM_COMM_BAUDRATE;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(PEO_PM_COMM_USART, &usart);

    /* ---- DMA1 Stream5: circular RX ---- */
    DMA_DeInit(PEO_DMA_COMM_RX_STREAM);
    dma.DMA_Channel            = PEO_DMA_COMM_RX_CH;      /* Channel 4 */
    dma.DMA_PeripheralBaseAddr = (uint32_t)&PEO_PM_COMM_USART->DR;
    dma.DMA_Memory0BaseAddr    = (uint32_t)rx_ring;
    dma.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize         = PEO_COMM_RX_RING_SIZE;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode               = DMA_Mode_Circular;
    dma.DMA_Priority           = DMA_Priority_High;
    dma.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
    dma.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(PEO_DMA_COMM_RX_STREAM, &dma);
    DMA_ITConfig(PEO_DMA_COMM_RX_STREAM, DMA_IT_TC, ENABLE);   /* rollover count */
    DMA_Cmd(PEO_DMA_COMM_RX_STREAM, ENABLE);

    /* ---- DMA1 Stream6: normal TX (armed per transfer) ---- */
    DMA_DeInit(PEO_DMA_COMM_TX_STREAM);
    dma.DMA_Channel            = PEO_DMA_COMM_TX_CH;      /* Channel 4 */
    dma.DMA_PeripheralBaseAddr = (uint32_t)&PEO_PM_COMM_USART->DR;
    dma.DMA_Memory0BaseAddr    = 0;                       /* set in Send() */
    dma.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    dma.DMA_BufferSize         = 0;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode               = DMA_Mode_Normal;
    dma.DMA_Priority           = DMA_Priority_High;
    dma.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
    dma.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(PEO_DMA_COMM_TX_STREAM, &dma);
    DMA_ITConfig(PEO_DMA_COMM_TX_STREAM, DMA_IT_TC, ENABLE);   /* TX done */

    /* USART DMA requests: RX + TX; IDLE interrupt for framing.
       The NVIC channels (DMA1_Stream5/6, USART2) are enabled by
       NVIC_InitAll() - peripheral-side IT bits stay here. */
    USART_DMACmd(PEO_PM_COMM_USART, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);
    USART_ITConfig(PEO_PM_COMM_USART, USART_IT_IDLE, ENABLE);

    rx_rollover      = 0;
    rx_read_pos      = 0;
    rx_frame_len     = 0;
    rx_frame_pending = 0;
    tx_busy          = 0;

    USART_Cmd(PEO_PM_COMM_USART, ENABLE);
}

/* ----------------------------------------------------------------------------
 * TX: returns 1 if the transfer started, 0 if the previous one still runs
 * -------------------------------------------------------------------------- */
uint8_t USART_COMM_Send(const uint8_t *buf, uint16_t len)
{
    if ((buf == 0) || (len == 0) || tx_busy)
    {
        return 0;
    }
    tx_busy = 1;

    DMA_Cmd(PEO_DMA_COMM_TX_STREAM, DISABLE);
    DMA_ClearITPendingBit(PEO_DMA_COMM_TX_STREAM, DMA_IT_TCIF6);
    PEO_DMA_COMM_TX_STREAM->M0AR = (uint32_t)buf;
    PEO_DMA_COMM_TX_STREAM->NDTR = len;
    DMA_Cmd(PEO_DMA_COMM_TX_STREAM, ENABLE);

    return 1;
}

uint8_t USART_COMM_TxBusy(void)
{
    return tx_busy;
}

/* ----------------------------------------------------------------------------
 * RX: available byte count / non-blocking read / frame handling
 * -------------------------------------------------------------------------- */
uint16_t USART_COMM_RxAvailable(void)
{
    uint32_t avail;
    __disable_irq();
    avail = comm_get_write_pos() - rx_read_pos;
    __enable_irq();
    return (avail > 0xFFFFUL) ? 0xFFFFU : (uint16_t)avail;
}

uint16_t USART_COMM_Read(uint8_t *dst, uint16_t max)
{
    uint16_t copied = 0;
    uint32_t avail, i;

    __disable_irq();
    avail = comm_get_write_pos() - rx_read_pos;
    if (avail > max)
    {
        avail = max;
    }
    for (i = 0; i < avail; i++)
    {
        dst[i] = rx_ring[(rx_read_pos + i) % PEO_COMM_RX_RING_SIZE];
    }
    rx_read_pos += avail;
    copied = (uint16_t)avail;
    __enable_irq();

    return copied;
}

uint8_t USART_COMM_FramePending(void)
{
    return rx_frame_pending;
}

uint16_t USART_COMM_FrameLen(void)
{
    return rx_frame_len;
}

void USART_COMM_ClearFrame(void)
{
    rx_frame_pending = 0;
    rx_frame_len     = 0;
}

/* ============================================================================
   INTERRUPT HANDLERS (names fixed by startup_stm32f40xx.s)
   ========================================================================== */

/* DMA1 Stream5 (USART2 RX) TC: one full ring loop finished -> rollover++ */
void DMA1_Stream5_IRQHandler(void)
{
    if (DMA_GetITStatus(PEO_DMA_COMM_RX_STREAM, DMA_IT_TCIF5) != RESET)
    {
        DMA_ClearITPendingBit(PEO_DMA_COMM_RX_STREAM, DMA_IT_TCIF5);
        rx_rollover++;
    }
}

/* DMA1 Stream6 (USART2 TX) TC: transfer complete -> line free */
void DMA1_Stream6_IRQHandler(void)
{
    if (DMA_GetITStatus(PEO_DMA_COMM_TX_STREAM, DMA_IT_TCIF6) != RESET)
    {
        DMA_ClearITPendingBit(PEO_DMA_COMM_TX_STREAM, DMA_IT_TCIF6);
        DMA_Cmd(PEO_DMA_COMM_TX_STREAM, DISABLE);
        tx_busy = 0;
    }
}

/* USART2: IDLE line -> frame boundary; ORE -> recover */
void USART2_IRQHandler(void)
{
    volatile uint32_t dummy;

    if (USART_GetITStatus(PEO_PM_COMM_USART, USART_IT_IDLE) != RESET)
    {
        /* Read SR then DR - the required sequence to clear the IDLE flag */
        dummy = PEO_PM_COMM_USART->SR;
        dummy = PEO_PM_COMM_USART->DR;
        (void)dummy;
        rx_frame_len     = (uint16_t)(comm_get_write_pos() - rx_read_pos);
        rx_frame_pending = 1;
    }

    if (USART_GetFlagStatus(PEO_PM_COMM_USART, USART_FLAG_ORE) != RESET)
    {
        /* Overrun: frame bytes were lost - drain, wait for the next IDLE */
        dummy = PEO_PM_COMM_USART->SR;
        dummy = PEO_PM_COMM_USART->DR;
        (void)dummy;
    }
}
