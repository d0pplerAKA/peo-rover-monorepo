/**
 ******************************************************************************
 * @file    gamepad.c
 * @brief   PEO Rover - GAMEPAD 实现：USART3 帧接收 + 锁仲裁 + 摇杆映射
 *
 * USART3_RX -> DMA1_Stream1(Ch4) 循环接收 32B 手柄帧 + USART IDLE 中断组帧
 * （SOF 0xAA55 + LEN + type + body + CRC16），校验通过后更新手柄状态快照。
 * GAMEPAD_Process()（主循环）做失联检测 + 锁仲裁 + 摇杆映射。
 ******************************************************************************
 */
#include "include/gamepad.h"

#include "COMPONENTS/CONTROL/include/control.h"
#include "COMPONENTS/TIMER/include/timer.h"

#include <string.h>

/* ============================================================================
   CRC16-CCITT（多项式 0x1021，初始 0xFFFF，查表法，与 esp_link/c3_uart 一致）
   ============================================================================ */
static uint16_t g_crc_table[256];
static uint8_t  g_crc_ready = 0;

static void CRC16_BuildTable(void)
{
    uint16_t i, j, crc;

    for (i = 0; i < 256; i++)
    {
        crc = (uint16_t)(i << 8);
        for (j = 0; j < 8; j++)
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        g_crc_table[i] = crc;
    }
    g_crc_ready = 1;
}

static uint16_t CRC16_Calc(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    if (!g_crc_ready)
        CRC16_BuildTable();

    while (len--)
        crc = (uint16_t)((crc << 8) ^ g_crc_table[((crc >> 8) ^ *data++) & 0xFFU]);
    return crc;
}

/* ============================================================================
   手柄状态快照（USART3 中断写入，GAMEPAD_Process 消费）
   ============================================================================ */
static volatile gamepad_payload_t g_pad;
static volatile uint32_t g_last_rx_ms = 0;
static volatile uint8_t  g_prev_lock  = GAMEPAD_LOCK_MANUAL;

/* ============================================================================
   DMA 循环接收（USART3_RX -> DMA1_Stream1 Ch4）
   ----------------------------------------------------------------------------
   DMA 循环搬运字节到 g_rx_dma_buf；USART IDLE 中断（总线空闲 = 一帧结束）里
   读 NDTR 算长度，一次性校验 + 解析整帧。CPU 从逐字节 RXNE 中断解放出来，
   只在每帧结束（100Hz）进一次中断。
   ============================================================================ */
#define GAMEPAD_RX_BUF_SIZE  64   /* DMA 循环缓冲，>= 帧长 32B，留余量 */

/* uint32 数组保证 4 字节对齐（DMA Memory0 地址要求） */
static uint32_t g_rx_dma_buf[GAMEPAD_RX_BUF_SIZE / 4];

/** 校验 + 解析一整帧（IDLE 中断里调）：SOF + LEN + type + CRC -> 状态快照。 */
static void process_frame(uint16_t len)
{
    uint8_t *buf = (uint8_t *)g_rx_dma_buf;
    gamepad_payload_t pad;
    uint16_t crc_calc, crc_rx;

    if (len != GAMEPAD_FRAME_LEN)                          /* 帧长必须 32B */
        return;
    if (buf[0] != GAMEPAD_SOF0 || buf[1] != GAMEPAD_SOF1)  /* SOF 0xAA55 */
        return;
    if (buf[2] != GAMEPAD_PAYLOAD_LEN)                     /* LEN = 27 */
        return;
    if (buf[3] != GAMEPAD_TYPE_STATE)                      /* type = 0x10 */
        return;

    /* CRC 校验：覆盖 SOF~PAYLOAD（30 字节 = 2+1+1+26），不含 CRC 本身。 */
    crc_calc = CRC16_Calc(buf, 30);
    crc_rx   = (uint16_t)(buf[30] | ((uint16_t)buf[31] << 8));
    if (crc_calc != crc_rx)
        return;

    /* body(26B) 从 buf[4] 开始，memcpy 到对齐的局部变量再赋值（避免未对齐访问）。 */
    memcpy(&pad, &buf[4], GAMEPAD_BODY_LEN);
    g_pad        = pad;
    g_last_rx_ms = TIMER_GetTick();
}

/** USART3 IDLE 中断（帧结束）。 */
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(PEO_PM_UART3_USART, USART_IT_IDLE) != RESET)
    {
        uint16_t len;

        /* 清 IDLE：先读 SR 再读 DR（RM0090 软件序列） */
        (void)PEO_PM_UART3_USART->SR;
        (void)PEO_PM_UART3_USART->DR;

        /* 本帧已收字节数 = 缓冲大小 - DMA 剩余计数 */
        len = (uint16_t)(GAMEPAD_RX_BUF_SIZE
                         - DMA_GetCurrDataCounter(PEO_DMA_GAMEPAD_RX_STREAM));

        process_frame(len);

        /* 重置 DMA 计数器，从缓冲头收下一帧（RM0090 重配置序列） */
        DMA_Cmd(PEO_DMA_GAMEPAD_RX_STREAM, DISABLE);
        while (DMA_GetCmdStatus(PEO_DMA_GAMEPAD_RX_STREAM) != DISABLE)
            ;
        DMA_ClearFlag(PEO_DMA_GAMEPAD_RX_STREAM,
                      DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1 | DMA_FLAG_TEIF1 |
                      DMA_FLAG_DMEIF1 | DMA_FLAG_FEIF1);
        DMA_SetCurrDataCounter(PEO_DMA_GAMEPAD_RX_STREAM, GAMEPAD_RX_BUF_SIZE);
        DMA_Cmd(PEO_DMA_GAMEPAD_RX_STREAM, ENABLE);
    }
}

/* ============================================================================
   摇杆映射（uint16 0..65535，中值 32768 -> 死区 ±500 -> 速度 ±4424）
   ============================================================================ */
static int32_t map_axis(uint16_t raw)
{
    int32_t v  = (int32_t)raw - 32768;   /* -32768 .. +32767 */
    int32_t dz = GAMEPAD_DEADZONE;

    if (v > -dz && v < dz)
        return 0;                        /* 死区内归零 */

    if (v >= dz)
        return (v - dz) * GAMEPAD_SPEED_MAX / (32767 - dz);
    else
        return (v + dz) * GAMEPAD_SPEED_MAX / (32768 - dz);
}

/* ============================================================================
   初始化
   ============================================================================ */
void GAMEPAD_Init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    DMA_InitTypeDef   dma;

    /* GPIO：PD8(TX)/PD9(RX)，AF7（RCC 时钟已由 RCC_Clock_Init 统一使能）。 */
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;      /* RX 上拉，空闲高电平 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin = PEO_PM_UART3_TX_PIN;
    GPIO_Init(PEO_PM_UART3_TX_PORT, &gpio);
    gpio.GPIO_Pin = PEO_PM_UART3_RX_PIN;
    GPIO_Init(PEO_PM_UART3_RX_PORT, &gpio);

    GPIO_PinAFConfig(PEO_PM_UART3_TX_PORT, PEO_PM_UART3_TX_SRC, PEO_PM_UART3_AF);
    GPIO_PinAFConfig(PEO_PM_UART3_RX_PORT, PEO_PM_UART3_RX_SRC, PEO_PM_UART3_AF);

    /* USART3：115200 8N1，无流控。 */
    usart.USART_BaudRate            = GAMEPAD_BAUDRATE;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(PEO_PM_UART3_USART, &usart);

    /* DMA 循环接收：USART3_RX -> DMA1_Stream1(Ch4) */
    DMA_DeInit(PEO_DMA_GAMEPAD_RX_STREAM);
    dma.DMA_Channel            = PEO_DMA_GAMEPAD_RX_CH;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&PEO_PM_UART3_USART->DR;
    dma.DMA_Memory0BaseAddr    = (uint32_t)g_rx_dma_buf;
    dma.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize         = GAMEPAD_RX_BUF_SIZE;
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
    DMA_Init(PEO_DMA_GAMEPAD_RX_STREAM, &dma);
    DMA_Cmd(PEO_DMA_GAMEPAD_RX_STREAM, ENABLE);

    /* USART 侧：DMA 请求（RX）+ IDLE 中断（帧结束检测），不开 RXNE 逐字节中断 */
    USART_DMACmd(PEO_PM_UART3_USART, USART_DMAReq_Rx, ENABLE);
    USART_ITConfig(PEO_PM_UART3_USART, USART_IT_IDLE, ENABLE);
    USART_Cmd(PEO_PM_UART3_USART, ENABLE);
}

/* ============================================================================
   Public API
   ============================================================================ */
uint8_t GAMEPAD_GetLock(void)
{
    return g_pad.lock;
}

uint8_t GAMEPAD_IsConnected(void)
{
    return g_pad.connected;
}

void GAMEPAD_Process(void)
{
    uint32_t now = TIMER_GetTick();
    uint8_t  lock = g_pad.lock;
    int32_t  throttle, steer, left, right;

    /* 锁切换瞬间停车（避免控制权切换时的速度突变）。 */
    if (lock != g_prev_lock)
    {
        CONTROL_StopAll();
        g_prev_lock = lock;
    }

    /* 锁开启（自主）：上位机命令接管，手柄映射跳过。 */
    if (lock == GAMEPAD_LOCK_AUTO)
        return;

    /* 锁关闭（手操）：C3 失联 / 手柄断连 / 急停键 -> 停车。 */
    if ((uint32_t)(now - g_last_rx_ms) >= GAMEPAD_STALE_MS)
    {
        CONTROL_StopAll();            /* C3 失联 */
        return;
    }
    if (!g_pad.connected || !g_pad.input_fresh)
    {
        CONTROL_StopAll();            /* 手柄断连 / 输入过期 */
        return;
    }
    if (g_pad.buttons & GAMEPAD_BTN_XBOX)
    {
        CONTROL_StopAll();            /* 急停键（双源，越过锁） */
        return;
    }

    /* 摇杆映射：左摇杆上下=前进/后退油门，右摇杆左右=单边（右轮）差速。 */
    throttle = map_axis(g_pad.joy_ly);
    steer    = map_axis(g_pad.joy_rx);

    left  = throttle;                 /* 左轮 = 油门 */
    right = throttle + steer;         /* 右轮 = 油门 + 差速 */

    if (left  >  GAMEPAD_SPEED_MAX) left  =  GAMEPAD_SPEED_MAX;
    if (left  < -GAMEPAD_SPEED_MAX) left  = -GAMEPAD_SPEED_MAX;
    if (right >  GAMEPAD_SPEED_MAX) right =  GAMEPAD_SPEED_MAX;
    if (right < -GAMEPAD_SPEED_MAX) right = -GAMEPAD_SPEED_MAX;

    /* M1/M3 = 左轮，M2/M4 = 右轮（MOTOR_M1..M4 = 0..3）。四轮原子一次写完。 */
    {
        const int32_t targets[4] = { left, right, left, right };
        CONTROL_SetTargetSpeedAll(targets);
    }
}
