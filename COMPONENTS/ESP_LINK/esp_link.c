/**
 ******************************************************************************
 * @file    esp_link.c
 * @brief   PEO Rover - ESP32 link (USART2) 实现：CRC16 + 帧收发 + 命令分发
 *
 * 方向1（F407 -> ESP32）：ESP_LINK_Task() 按 SENSOR_RATE_REPORT_HZ 打包上报
 *                         sensorhub（SensorData_t 112B）。
 * 方向2（ESP32 -> F407）：USART2_RX -> DMA1_Stream5 循环接收 + USART IDLE 中断，
 *                         整帧解析「手柄20B + 命令可选」，命令执行后回显 ACK。
 ******************************************************************************
 */
#include "include/esp_link.h"

#include "COMPONENTS/SENSOR_HUB/include/sensor_hub.h"
#include "COMPONENTS/TIMER/include/timer.h"
#include "COMPONENTS/BUILTIN_LED/builtin_led.h"
#include "COMPONENTS/CONTROL/include/control.h"
#include "COMPONENTS/MOTOR/include/motor.h"

#include <string.h>

/* ============================================================================
   1. CRC16-CCITT（多项式 0x1021，初始 0xFFFF，查表法）
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
        {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
        g_crc_table[i] = crc;
    }
    g_crc_ready = 1;
}

uint16_t ESP_LINK_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    if (!g_crc_ready)
        CRC16_BuildTable();

    while (len--)
    {
        crc = (uint16_t)((crc << 8) ^ g_crc_table[((crc >> 8) ^ *data++) & 0xFFU]);
    }
    return crc;
}

/* ============================================================================
   2. 发送（方向1：sensorhub 上报，DMA 非阻塞）
   ============================================================================ */
/* uint32 数组保证 4 字节对齐（DMA Memory0 地址要求） */
static uint32_t g_tx_buf[(ESP_LINK_FRAME_OVH + sizeof(SensorData_t) + 3) / 4];

/**
 * @brief  取 sensorhub 快照 -> 套帧（SOF+LEN+payload+CRC）-> 启动 DMA 发送。
 * @note   非阻塞：DMA 后台搬运，CPU 立即返回，不卡 1kHz 控制循环。
 */
static void ESP_LINK_SendHub(void)
{
    uint8_t     *buf  = (uint8_t *)g_tx_buf;
    SensorData_t hub;
    uint16_t     crc;
    uint16_t     len  = (uint16_t)sizeof(SensorData_t);
    uint16_t     flen = (uint16_t)(len + ESP_LINK_FRAME_OVH);

    SENSOR_GetHub(&hub);

    buf[0] = ESP_LINK_SOF0;
    buf[1] = ESP_LINK_SOF1;
    buf[2] = (uint8_t)len;
    memcpy(&buf[3], &hub, len);

    crc = ESP_LINK_CRC16(buf, (uint16_t)(3 + len));
    buf[3 + len]     = (uint8_t)(crc & 0xFFU);       /* CRC 低字节在前 */
    buf[3 + len + 1] = (uint8_t)(crc >> 8);

    /* 若上一次 DMA 发送未完成（Normal 模式发完 EN 位自动清），跳过本次，
       避免打断/截断上一帧。 */
    if (DMA_GetCmdStatus(PEO_DMA_ESP_TX_STREAM) != DISABLE)
        return;

    /* 启动 DMA 发送（单次模式，发 flen 字节后自动停） */
    DMA_Cmd(PEO_DMA_ESP_TX_STREAM, DISABLE);
    DMA_SetCurrDataCounter(PEO_DMA_ESP_TX_STREAM, flen);
    DMA_Cmd(PEO_DMA_ESP_TX_STREAM, ENABLE);
}

/* ============================================================================
   3. DMA 循环接收（方向2：手柄 + 命令）
   ----------------------------------------------------------------------------
   USART2_RX -> DMA1_Stream5(Ch4) 循环搬运字节到 g_rx_dma_buf；USART IDLE
   中断（总线空闲 = 一帧结束）里读 NDTR 算长度，一次性处理整帧。
   CPU 从逐字节 RXNE 中断解放出来，只在每帧结束时进一次中断。
   ============================================================================ */
#define ESP_LINK_RX_BUF_SIZE  128   /* DMA 循环缓冲，>= 最大帧长 */

/* uint32 数组保证 4 字节对齐（DMA Memory0 地址要求） */
static uint32_t g_rx_dma_buf[ESP_LINK_RX_BUF_SIZE / 4];

/* 最新手柄状态（供 CONTROL 读，遥控用） */
static volatile GamepadData_t g_gamepad;

/* 前置声明：HandleCmd 定义在 ProcessFrame 之后 */
static void ESP_LINK_HandleCmd(const uint8_t *cmd, uint8_t len);

/**
 * @brief  处理一帧完整 payload：前 20B 手柄，剩余字节命令（可选）。
 */
static void ESP_LINK_ProcessFrame(const uint8_t *payload, uint8_t len)
{
    uint8_t cmd_len;

    if (len < sizeof(GamepadData_t))
        return;   /* 连手柄都不完整，丢弃 */

    /* 前 20 字节 = 手柄 */
    memcpy((void *)&g_gamepad, payload, sizeof(GamepadData_t));

    /* 剩余 = 命令（0 字节 = 无命令） */
    cmd_len = (uint8_t)(len - sizeof(GamepadData_t));
    if (cmd_len > 0)
        ESP_LINK_HandleCmd(payload + sizeof(GamepadData_t), cmd_len);
}

/**
 * @brief  校验并处理一整帧（IDLE 中断里调）：SOF + LEN + CRC -> 手柄 + 命令。
 * @note   len 为 DMA 本帧已搬运到缓冲的字节数。
 */
static void ESP_LINK_ProcessRx(uint16_t len)
{
    uint8_t *buf = (uint8_t *)g_rx_dma_buf;
    uint8_t  payload_len;
    uint16_t crc_calc, crc_recv;

    if (len < ESP_LINK_FRAME_OVH)                          /* 不足 SOF+LEN+CRC */
        return;
    if (buf[0] != ESP_LINK_SOF0 || buf[1] != ESP_LINK_SOF1)
        return;                                            /* 帧头错 */
    payload_len = buf[2];
    if (payload_len > ESP_LINK_MAX_PAYLOAD)
        return;                                            /* 长度非法 */
    if (len != (uint16_t)(payload_len + ESP_LINK_FRAME_OVH))
        return;                                            /* 长度与 LEN 不符 */

    crc_calc = ESP_LINK_CRC16(buf, (uint16_t)(3 + payload_len));
    crc_recv = (uint16_t)buf[3 + payload_len]
             | (uint16_t)((uint16_t)buf[3 + payload_len + 1] << 8);
    if (crc_calc != crc_recv)
        return;                                            /* CRC 错 */

    ESP_LINK_ProcessFrame(&buf[3], payload_len);
}

/* ============================================================================
   4. 命令分发
   ============================================================================ */
static void ESP_LINK_HandleCmd(const uint8_t *cmd, uint8_t len)
{
    uint8_t  cmd_id    = cmd[0];
    const uint8_t *param = &cmd[1];
    uint8_t  param_len = (uint8_t)(len - 1);
    uint8_t  status    = ESP_CMD_OK;

    switch (cmd_id)
    {
        case ESP_CMD_LED_TOGGLE:
            BUILTIN_LED_Toggle();
            break;

        case ESP_CMD_SENSOR_RESET:
            /* TODO: 传感器复位（VL53L1X / MPU6050 复位流程） */
            status = ESP_CMD_OK;
            break;

        case ESP_CMD_MOTOR:
        {
            uint8_t motor;
            int16_t speed;
            if (param_len < 3) { status = ESP_CMD_ERR_BADLEN; break; }
            motor = param[0];
            speed = (int16_t)(param[1] | ((uint16_t)param[2] << 8));
            if (motor >= SENSOR_WHEEL_COUNT) { status = ESP_CMD_ERR_BADMOTOR; break; }
            MOTOR_SetSpeed(motor, speed);
            break;
        }

        case ESP_CMD_PID:
        {
            float kp, ki;
            if (param_len < 8) { status = ESP_CMD_ERR_BADLEN; break; }
            memcpy(&kp, param,     4);
            memcpy(&ki, param + 4, 4);
            CONTROL_SetGains(kp, ki);
            break;
        }

        case ESP_CMD_MOTOR_CLOSED:
        {
            uint8_t motor;
            int32_t speed_d100;
            if (param_len < 5) { status = ESP_CMD_ERR_BADLEN; break; }
            motor = param[0];
            memcpy(&speed_d100, param + 1, 4);
            if (motor >= SENSOR_WHEEL_COUNT) { status = ESP_CMD_ERR_BADMOTOR; break; }
            CONTROL_SetTargetSpeed(motor, speed_d100);
            break;
        }

        default:
            status = ESP_CMD_ERR_INVALID;
            break;
    }

    SENSOR_SetCmdAck(cmd_id, status);
}

/* ============================================================================
   5. USART2 IDLE 中断（帧结束）
   ============================================================================ */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(PEO_PM_ESP_USART, USART_IT_IDLE) != RESET)
    {
        uint16_t len;

        /* 清 IDLE：先读 SR 再读 DR（RM0090 软件序列） */
        (void)PEO_PM_ESP_USART->SR;
        (void)PEO_PM_ESP_USART->DR;

        /* 本帧已收字节数 = 缓冲大小 - DMA 剩余计数 */
        len = (uint16_t)(ESP_LINK_RX_BUF_SIZE
                         - DMA_GetCurrDataCounter(PEO_DMA_ESP_RX_STREAM));

        ESP_LINK_ProcessRx(len);

        /* 重置 DMA 计数器，从缓冲头收下一帧 */
        DMA_Cmd(PEO_DMA_ESP_RX_STREAM, DISABLE);
        DMA_SetCurrDataCounter(PEO_DMA_ESP_RX_STREAM, ESP_LINK_RX_BUF_SIZE);
        DMA_Cmd(PEO_DMA_ESP_RX_STREAM, ENABLE);
    }
}

/* ============================================================================
   6. 初始化 + 主循环任务
   ============================================================================ */
void ESP_LINK_Init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    DMA_InitTypeDef   dma;

    /* 预建 CRC 表 */
    CRC16_BuildTable();

    /* PD5 = USART2_TX, PD6 = USART2_RX, AF7 */
    gpio.GPIO_Pin   = PEO_PM_ESP_TX_PIN | PEO_PM_ESP_RX_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;          /* RX 空闲高 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PEO_PM_ESP_TX_PORT, &gpio);
    GPIO_PinAFConfig(PEO_PM_ESP_TX_PORT, PEO_PM_ESP_TX_SRC, PEO_PM_ESP_AF);
    GPIO_PinAFConfig(PEO_PM_ESP_RX_PORT, PEO_PM_ESP_RX_SRC, PEO_PM_ESP_AF);

    /* 8-N-1，无流控，RX+TX */
    usart.USART_BaudRate            = PEO_PM_ESP_BAUDRATE;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(PEO_PM_ESP_USART, &usart);

    /* DMA 循环接收：USART2_RX -> DMA1_Stream5(Ch4) */
    DMA_DeInit(PEO_DMA_ESP_RX_STREAM);
    dma.DMA_Channel            = PEO_DMA_ESP_RX_CH;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&PEO_PM_ESP_USART->DR;
    dma.DMA_Memory0BaseAddr    = (uint32_t)g_rx_dma_buf;
    dma.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize         = ESP_LINK_RX_BUF_SIZE;
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
    DMA_Init(PEO_DMA_ESP_RX_STREAM, &dma);
    DMA_Cmd(PEO_DMA_ESP_RX_STREAM, ENABLE);

    /* DMA 发送：USART2_TX <- DMA1_Stream6(Ch4)，单次模式（发送时设长度） */
    DMA_DeInit(PEO_DMA_ESP_TX_STREAM);
    dma.DMA_Channel            = PEO_DMA_ESP_TX_CH;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&PEO_PM_ESP_USART->DR;
    dma.DMA_Memory0BaseAddr    = (uint32_t)g_tx_buf;
    dma.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    dma.DMA_BufferSize         = 0;   /* 发送时用 DMA_SetCurrDataCounter 设实际长度 */
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
    DMA_Init(PEO_DMA_ESP_TX_STREAM, &dma);

    /* USART 侧：DMA 请求（RX + TX）+ IDLE 中断（帧结束检测），不开 RXNE 逐字节中断 */
    USART_DMACmd(PEO_PM_ESP_USART, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(PEO_PM_ESP_USART, USART_DMAReq_Tx, ENABLE);
    USART_ITConfig(PEO_PM_ESP_USART, USART_IT_IDLE, ENABLE);
    USART_Cmd(PEO_PM_ESP_USART, ENABLE);
}

/* ============================================================================
   6. TIM14 上报定时器
   ============================================================================ */
#define ESP_REPORT_TIM      PEO_PM_ESP_REPORT_TIM
#define ESP_REPORT_TIM_PSC  (PEO_PM_TIM_APB1_HZ / 10000UL - 1)     /* 84 MHz -> 10 kHz */
#define ESP_REPORT_TIM_ARR  (10000UL / SENSOR_RATE_REPORT_HZ - 1)  /* 10 kHz -> RATE Hz */

void ESP_LINK_StartReport(void)
{
    TIM_TimeBaseInitTypeDef tb;

    tb.TIM_Prescaler         = ESP_REPORT_TIM_PSC;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = ESP_REPORT_TIM_ARR;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(ESP_REPORT_TIM, &tb);

    TIM_ClearITPendingBit(ESP_REPORT_TIM, TIM_IT_Update);
    TIM_ITConfig(ESP_REPORT_TIM, TIM_IT_Update, ENABLE);
    TIM_Cmd(ESP_REPORT_TIM, ENABLE);
}

/**
 * @brief  TIM14 Update 中断 = sensorhub 上报（SENSOR_RATE_REPORT_HZ）。
 * @note   覆盖 startup 的 [WEAK] TIM8_TRG_COM_TIM14_IRQHandler（TIM8 纯编码器零中断）。
 */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
    if (TIM_GetITStatus(ESP_REPORT_TIM, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(ESP_REPORT_TIM, TIM_IT_Update);
        ESP_LINK_SendHub();   /* 打包 + 启动 DMA 发送（非阻塞） */
    }
}
