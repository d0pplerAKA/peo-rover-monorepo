/**
 * @file  esp_link.c
 * @brief Exchange checked command and telemetry frames with the ESP32-S3 WiFi bridge.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * F407 -> ESP32: send the 112-byte SensorData_t snapshot at SENSOR_RATE_REPORT_HZ.
 * The earlier ESP_LINK_Task description referred to this reporting role;
 * this implementation schedules it with TIM14 and sends through DMA.
 * ESP32 -> F407: USART2_RX feeds circular DMA1_Stream5; the USART IDLE interrupt
 * validates one complete command frame (command ID plus parameters). Execution
 * happens in main, and its acknowledgement is echoed in the sensor hub.
 */
#include "include/esp_link.h"

#include "COMPONENTS/SENSOR_HUB/include/sensor_hub.h"
#include "COMPONENTS/BUILTIN_LED/builtin_led.h"
#include "COMPONENTS/CONTROL/include/control.h"
#include "COMPONENTS/MOTOR/include/motor.h"
#include "COMPONENTS/TIMER/include/timer.h"
#include "COMPONENTS/GAMEPAD/include/gamepad.h"

#include <math.h>
#include <string.h>

/* 1. CRC16-CCITT: polynomial 0x1021, initial value 0xFFFF, table lookup. */
static uint16_t g_crc_table[256];  /* CRC result for each possible byte value; speeds up frame integrity checks. */
static uint8_t  g_crc_ready = 0;  /* Nonzero once the CRC lookup table has been initialized. */

/**
 * @brief Prepare a table that makes checking received serial data faster.
 *
 * Called by link initialization, or lazily by the CRC calculator if necessary. Each
 * entry replaces eight bit-by-bit polynomial steps.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
static void CRC16_BuildTable(void)
{
    uint16_t i, j, crc; /* i selects a table entry, j counts its eight bits, and crc holds the 16-bit remainder. */

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

/**
 * @brief Calculate the 16-bit check value used to detect damaged serial frames.
 *
 * Used when building telemetry frames and validating host command frames. Uses
 * CRC16-CCITT polynomial 0x1021 and seed 0xFFFF.
 * @param data Readable buffer containing at least len bytes; frame callers pass SOF
 *             through the end of payload.
 * @param len Number of bytes to include, excluding the stored CRC bytes.
 * @retval value Calculated 16-bit CRC; empty input returns the initial value 0xFFFF.
 */
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

/* 2. Transmit sensor-hub reports to ESP32 using nonblocking DMA. */
/* A uint32_t array provides four-byte alignment for the DMA memory address. */
/* Persistent aligned frame storage; DMA reads it until transmission completes. */
static uint32_t g_tx_buf[(ESP_LINK_FRAME_OVH + sizeof(SensorData_t) + 3) / 4];

/**
 * @brief Copy current telemetry into a framed packet and start its DMA transmission.
 *
 * Called by the TIM14 report interrupt. If the previous DMA transfer is still active,
 * skip this report without touching its buffer.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note DMA transfers in the background; the CPU returns without waiting for the serial
 *       bytes, allowing the 1 kHz control loop to continue.
 */
static void ESP_LINK_SendHub(void)
{
    uint8_t     *buf  = (uint8_t *)g_tx_buf;  /* Byte-oriented view of the aligned DMA buffer for constructing or checking frames. */
    SensorData_t hub;  /* Local consistent telemetry snapshot copied before framing and transmission. */
    uint16_t     crc;
    uint16_t     len  = (uint16_t)sizeof(SensorData_t);  /* Byte count for the current payload or completed DMA receive operation. */
    /* Complete transmit length in bytes, including SOF, LEN, payload, and CRC. */
    uint16_t     flen = (uint16_t)(len + ESP_LINK_FRAME_OVH);

    /* If DMA still owns the previous frame, skip this report before modifying any byte. */
    if (DMA_GetCmdStatus(PEO_DMA_ESP_TX_STREAM) != DISABLE)
        return;

    SENSOR_GetHub(&hub);

    buf[0] = ESP_LINK_SOF0;
    buf[1] = ESP_LINK_SOF1;
    buf[2] = (uint8_t)len;
    memcpy(&buf[3], &hub, len);

    crc = ESP_LINK_CRC16(buf, (uint16_t)(3 + len));
    buf[3 + len]     = (uint8_t)(crc & 0xFFU);       /* Send the CRC low byte first (little-endian). */
    buf[3 + len + 1] = (uint8_t)(crc >> 8);

    /* Start a normal-mode DMA transfer; it stops after flen bytes.
       RM0090 reconfiguration sequence: disable, wait for EN to clear, clear
       all flags, set the byte count, then enable. */
    DMA_Cmd(PEO_DMA_ESP_TX_STREAM, DISABLE);
    while (DMA_GetCmdStatus(PEO_DMA_ESP_TX_STREAM) != DISABLE)
        ;   /* Wait until the hardware EN bit actually clears. */
    DMA_ClearFlag(PEO_DMA_ESP_TX_STREAM,
                  DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6 |
                  DMA_FLAG_DMEIF6 | DMA_FLAG_FEIF6);
    DMA_SetCurrDataCounter(PEO_DMA_ESP_TX_STREAM, flen);
    DMA_Cmd(PEO_DMA_ESP_TX_STREAM, ENABLE);
}

/* 3. Receive command frames using circular DMA.
   USART2_RX -> DMA1_Stream5 (channel 4) copies bytes into g_rx_dma_buf.
   The USART IDLE interrupt treats a pause on the wire as a frame boundary,
   reads NDTR to calculate the byte count, and processes the whole frame.
   This avoids a separate RXNE CPU interrupt for every byte. */
#define ESP_LINK_RX_BUF_SIZE  128   /* Receive-buffer size in bytes; must fit a complete accepted command frame. */

/* A uint32_t array provides four-byte alignment for the DMA memory address. */
static uint32_t g_rx_dma_buf[ESP_LINK_RX_BUF_SIZE / 4];  /* Aligned DMA receive storage; interpreted as a byte array by the parser. */

/* Link-loss stop: timestamp of the latest CRC-valid frame, in SysTick ms.
   g_rx_seen=0 means no valid frame has arrived yet, so supervision is inactive.
   A timestamp of zero is valid. The S3 sends an empty heartbeat every 200 ms. */
static volatile uint32_t g_last_rx_ms = 0;  /* SysTick millisecond timestamp of the latest valid frame on this link. */
static volatile uint8_t  g_rx_seen = 0;  /* Records whether any valid S3 frame has arrived, enabling link-loss supervision. */

/* Forward declaration: HandleCmd is defined after ProcessFrame. */
/**
 * @brief Separate urgent STOP requests from ordinary queued commands.
 *
 * Called by the receive parser in USART2 interrupt context after frame validation. STOP
 * sets a separate flag; queue failures publish an error acknowledgement.
 * @param cmd Non-NULL payload starting with one command-ID byte, followed by its
 *            parameters.
 * @param len Payload length in bytes; must be at least one.
 * @retval None This function does not return a value.
 */
static void ESP_LINK_HandleCmd(const uint8_t *cmd, uint8_t len);

/**
 * @brief Forward a nonempty payload for command dispatch.
 *
 * Called after the USART2 parser validates a frame. Empty frames act only as heartbeats
 * and contain no command.
 * @param payload Payload bytes after the three-byte SOF/LEN header.
 * @param len Payload byte count; zero means heartbeat.
 * @retval None This function does not return a value.
 */
static void ESP_LINK_ProcessFrame(const uint8_t *payload, uint8_t len)
{
    if (len < 1)
        return;   /* Empty heartbeat frame: there is no command to execute. */

    ESP_LINK_HandleCmd(payload, len);
}

/**
 * @brief Check the received frame before allowing it to affect the rover.
 *
 * Called from the USART2 IDLE handler. Validates exact length, SOF, and CRC, refreshes
 * the link timestamp, then dispatches the payload.
 * @param len Number of received bytes currently stored in g_rx_dma_buf.
 * @retval None This function does not return a value.
 * @note len counts bytes DMA has already placed in this receive buffer.
 */
static void ESP_LINK_ProcessRx(uint16_t len)
{
    uint8_t *buf = (uint8_t *)g_rx_dma_buf;  /* Byte-oriented view of the aligned DMA buffer for constructing or checking frames. */
    uint8_t  payload_len;  /* Payload byte count advertised by the received LEN field. */
    uint16_t crc_calc, crc_recv; /* Calculated and received CRC values; a mismatch rejects the frame. */

    if (len < ESP_LINK_FRAME_OVH)                          /* Too short to contain SOF, LEN, and CRC. */
        return;
    if (buf[0] != ESP_LINK_SOF0 || buf[1] != ESP_LINK_SOF1)
        return;                                            /* Incorrect start-of-frame bytes. */
    payload_len = buf[2];
    if (payload_len > ESP_LINK_MAX_PAYLOAD)
        return;                                            /* Payload length exceeds the accepted limit. */
    if (len != (uint16_t)(payload_len + ESP_LINK_FRAME_OVH))
        return;                                            /* Received byte count does not match LEN plus framing overhead. */

    crc_calc = ESP_LINK_CRC16(buf, (uint16_t)(3 + payload_len));
    crc_recv = (uint16_t)buf[3 + payload_len]
             | (uint16_t)((uint16_t)buf[3 + payload_len + 1] << 8);
    if (crc_calc != crc_recv)
        return;                                            /* CRC mismatch: discard the corrupted frame. */

    g_last_rx_ms = TIMER_GetTick();   /* Every valid frame, including an empty heartbeat, refreshes link supervision. */
    g_rx_seen = 1;
    ESP_LINK_ProcessFrame(&buf[3], payload_len);
}

/* 4. Command dispatch. */
/* The interrupt enqueues commands; the main loop executes their actions. */
#define ESP_CMD_QUEUE_SIZE  8   /* Ring buffer with one empty slot to distinguish full from empty: seven usable entries. */

/* Pending host commands copied by the receive interrupt for execution in main. */
static ESP_Cmd_t          g_cmd_queue[ESP_CMD_QUEUE_SIZE];
static volatile uint8_t   g_cmd_head = 0;   /* Next write position, owned by the USART IDLE interrupt. */
static volatile uint8_t   g_cmd_tail = 0;   /* Next read position, owned by the main loop. */
static volatile uint8_t   g_stop_requested = 0;   /* Separate STOP flag; does not consume a queue slot. */

/**
 * @brief Separate urgent STOP requests from ordinary queued commands.
 *
 * Called by the receive parser in USART2 interrupt context after frame validation. STOP
 * sets a separate flag; queue failures publish an error acknowledgement.
 * @param cmd Non-NULL payload starting with one command-ID byte, followed by its
 *            parameters.
 * @param len Payload length in bytes; must be at least one.
 * @retval None This function does not return a value.
 */
static void ESP_LINK_HandleCmd(const uint8_t *cmd, uint8_t len)
{
    uint8_t cmd_id    = cmd[0];  /* First payload byte, identifying which host action is requested. */
    uint8_t param_len = (uint8_t)(len - 1);  /* Command parameter byte count, excluding the command-ID byte. */

    if (cmd_id == ESP_CMD_STOP_ALL)
    {
        g_stop_requested = 1;
        return;
    }

    /* Only report an error here if queuing fails (full queue or too many parameter bytes). */
    if (!ESP_LINK_PushCommand(cmd_id, &cmd[1], param_len))
        SENSOR_SetCmdAck(cmd_id, ESP_CMD_ERR_INVALID);
}

/**
 * @brief Check whether an urgent STOP request is waiting for main.
 *
 * Main checks this before removing ordinary commands, so a full command queue cannot
 * prevent a STOP request.
 * @note No arguments are required.
 * @retval value Nonzero if STOP is pending; zero otherwise.
 */
uint8_t ESP_LINK_StopPending(void)
{
    return g_stop_requested;
}

/**
 * @brief Acknowledge consumption of the independent STOP request.
 *
 * Main clears the flag immediately before calling CONTROL_StopAll and publishing the
 * acknowledgement.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void ESP_LINK_ClearStop(void)
{
    g_stop_requested = 0;
}

/**
 * @brief Save one command for later execution by the main loop.
 *
 * Producer API used from the USART2 interrupt; the ring holds seven commands. It copies
 * the parameter bytes, so the receive buffer can be reused.
 * @param id Command identifier from ESP_CMD_*.
 * @param param Readable parameter buffer; may be NULL only when param_len is zero.
 * @param param_len Parameter count in bytes, at most ESP_CMD_MAX_PARAM.
 * @retval 1 The command was copied into the queue.
 * @retval 0 Queue full or parameters too long; nothing was enqueued.
 */
uint8_t ESP_LINK_PushCommand(uint8_t id, const uint8_t *param, uint8_t param_len)
{
    /* Candidate ring-buffer write position; matching tail means the queue is full. */
    uint8_t next = (uint8_t)((g_cmd_head + 1) % ESP_CMD_QUEUE_SIZE);

    if (next == g_cmd_tail)
        return 0;                              /* Queue is full. */
    if (param_len > ESP_CMD_MAX_PARAM)
        return 0;                              /* Too many parameter bytes. */

    g_cmd_queue[g_cmd_head].id        = id;
    g_cmd_queue[g_cmd_head].param_len = param_len;
    if (param_len > 0 && param != 0)
        memcpy(g_cmd_queue[g_cmd_head].param, param, param_len);
    g_cmd_head = next;
    return 1;
}

/**
 * @brief Take the oldest waiting command out of the queue.
 *
 * Consumer API called by main; on success main passes the copied command to
 * ESP_LINK_ExecCommand.
 * @param out Non-NULL writable command structure receiving the queue item.
 * @retval 1 A command was copied to out.
 * @retval 0 The queue was empty; out was not changed.
 */
uint8_t ESP_LINK_PopCommand(ESP_Cmd_t *out)
{
    if (g_cmd_head == g_cmd_tail)
        return 0;                              /* Queue is empty. */

    *out = g_cmd_queue[g_cmd_tail];
    g_cmd_tail = (uint8_t)((g_cmd_tail + 1) % ESP_CMD_QUEUE_SIZE);
    return 1;
}

/**
 * @brief Apply a host command and record whether it was accepted.
 *
 * Called by main after dequeuing. Manual ownership blocks host motor and PI-tuning
 * requests; other commands include LED control, odometry reset, STOP, and system reset.
 * @param cmd Non-NULL queued command with its ID and parameter bytes.
 * @retval None This function does not return a value.
 */
void ESP_LINK_ExecCommand(const ESP_Cmd_t *cmd)
{
    uint8_t status = ESP_CMD_OK;  /* Acknowledgement result for this command; starts at ESP_CMD_OK. */

    /* Manual ownership rejects host movement and gain changes. STOP and
       unrelated commands remain available regardless of the ownership lock. */
    if ((cmd->id == ESP_CMD_MOTOR || cmd->id == ESP_CMD_PID ||
         cmd->id == ESP_CMD_MOTOR_CLOSED) &&
        GAMEPAD_GetLock() == GAMEPAD_LOCK_MANUAL)
    {
        SENSOR_SetCmdAck(cmd->id, ESP_CMD_ERR_INVALID);
        return;
    }

    switch (cmd->id)
    {
        case ESP_CMD_LED_TOGGLE:
            BUILTIN_LED_Toggle();
            break;

        case ESP_CMD_LED_ON:
            BUILTIN_LED_On();
            break;

        case ESP_CMD_LED_OFF:
            BUILTIN_LED_Off();
            break;

        case ESP_CMD_MOTOR:
        {
#if PEO_ENABLE_MOTOR
            uint8_t motor;  /* Zero-based wheel index, 0..3 in M1..M4 order. */
            int16_t speed;  /* Signed direct PWM request in per-mille; this is not a measured speed. */
            if (cmd->param_len < 3) { status = ESP_CMD_ERR_BADLEN; break; }
            motor = cmd->param[0];
            speed = (int16_t)(cmd->param[1] | ((uint16_t)cmd->param[2] << 8));
            if (motor >= SENSOR_WHEEL_COUNT) { status = ESP_CMD_ERR_BADMOTOR; break; }
#if PEO_ENABLE_CONTROL
            CONTROL_SetMode(CONTROL_MODE_OPEN);   /* Switch only when the TIM6 controller exists, so PI will not overwrite direct PWM. */
#endif
            MOTOR_SetSpeed(motor, speed);
#else
            status = ESP_CMD_ERR_INVALID;   /* Motors are disabled in this build; reject movement commands. */
#endif
            break;
        }

        case ESP_CMD_PID:
        {
            float kp, ki; /* Shared PI gains decoded from two four-byte floats in the command. */
            if (cmd->param_len < 8) { status = ESP_CMD_ERR_BADLEN; break; }
            memcpy(&kp, cmd->param,     4);
            memcpy(&ki, cmd->param + 4, 4);
            if (!isfinite(kp) || !isfinite(ki) || kp < 0.0f || ki < 0.0f)
            {
                status = ESP_CMD_ERR_INVALID;
                break;
            }
            CONTROL_SetGains(kp, ki);
            break;
        }

        case ESP_CMD_MOTOR_CLOSED:
        {
#if PEO_ENABLE_MOTOR && PEO_ENABLE_CONTROL
            uint8_t motor;  /* Zero-based wheel index, 0..3 in M1..M4 order. */
            int32_t speed_d100;  /* Signed closed-loop target in 0.01 cm/s decoded from the command. */
            if (cmd->param_len < 5) { status = ESP_CMD_ERR_BADLEN; break; }
            motor = cmd->param[0];
            memcpy(&speed_d100, cmd->param + 1, 4);
            if (motor >= SENSOR_WHEEL_COUNT) { status = ESP_CMD_ERR_BADMOTOR; break; }
            if (speed_d100 > GAMEPAD_SPEED_MAX || speed_d100 < -GAMEPAD_SPEED_MAX)
            {
                status = ESP_CMD_ERR_INVALID;
                break;
            }
            CONTROL_SetMode(CONTROL_MODE_CLOSED);   /* Return to closed-loop speed control. */
            CONTROL_SetTargetSpeed(motor, speed_d100);
#else
            status = ESP_CMD_ERR_INVALID;   /* Motors or CONTROL are disabled in this build; reject this movement command. */
#endif
            break;
        }

        case ESP_CMD_STOP_ALL:
            CONTROL_StopAll();   /* Emergency stop: clear speed targets and PI memory, then coast all four wheels. */
            break;

        case ESP_CMD_ODOMETER_RESET:
            CONTROL_ResetOdometer();   /* Reset the accumulated encoder counts for all four wheels. */
            break;

        case ESP_CMD_SYS_RESET:
            /* Software-reset the entire F407 to its startup state: sensor state,
               odometry, PWM, and queues are reinitialized. The ESP32 link then
               resumes; the host SDK is expected to detect fresh data returning
               and resend PID/resolution settings. */
            for (volatile uint32_t d = 0; d < 200000U; d++) { }   /* Short busy wait intended to let USART2 finish transmitting. */
            NVIC_SystemReset();
            break;

        default:
            status = ESP_CMD_ERR_INVALID;
            break;
    }

    SENSOR_SetCmdAck(cmd->id, status);
}

/* 5. USART2 IDLE interrupt: a pause marks the end of a frame. */
/**
 * @brief Handle the pause that marks the end of an ESP32-S3 command frame.
 *
 * Called by the processor for USART2 interrupts. Clears IDLE, calculates the DMA byte
 * count, validates the frame, and rearms reception at buffer offset zero.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(PEO_PM_ESP_USART, USART_IT_IDLE) != RESET)
    {
        uint16_t len;  /* Byte count for the current payload or completed DMA receive operation. */

        /* Clear IDLE by reading SR, then DR, as specified by RM0090. */
        (void)PEO_PM_ESP_USART->SR;
        (void)PEO_PM_ESP_USART->DR;

        /* Bytes received = buffer capacity minus the DMA remaining-byte count. */
        len = (uint16_t)(ESP_LINK_RX_BUF_SIZE
                         - DMA_GetCurrDataCounter(PEO_DMA_ESP_RX_STREAM));

        ESP_LINK_ProcessRx(len);

        /* Reset DMA using the RM0090 sequence so the next frame starts at buffer byte zero. */
        DMA_Cmd(PEO_DMA_ESP_RX_STREAM, DISABLE);
        while (DMA_GetCmdStatus(PEO_DMA_ESP_RX_STREAM) != DISABLE)
            ;   /* Wait until the hardware EN bit actually clears. */
        DMA_ClearFlag(PEO_DMA_ESP_RX_STREAM,
                      DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_TEIF5 |
                      DMA_FLAG_DMEIF5 | DMA_FLAG_FEIF5);
        DMA_SetCurrDataCounter(PEO_DMA_ESP_RX_STREAM, ESP_LINK_RX_BUF_SIZE);
        DMA_Cmd(PEO_DMA_ESP_RX_STREAM, ENABLE);
    }
}

/* 6. Link initialization and support for main-loop command processing. */
/**
 * @brief Prepare the USART2 host link, CRC table, and transmit/receive DMA.
 *
 * Called by main after RCC/NVIC setup. Uses PD5/PD6 at PEO_PM_ESP_BAUDRATE;
 * ESP_LINK_StartReport starts periodic telemetry separately.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void ESP_LINK_Init(void)
{
    GPIO_InitTypeDef  gpio;  /* GPIO setup record: selects pin routing, electrical drive, pulls, and switching speed. */
    USART_InitTypeDef usart;  /* Serial-port setup record: baud rate, byte format, and enabled directions. */
    DMA_InitTypeDef   dma;  /* DMA setup record: peripheral/memory addresses, byte count, and transfer behavior. */

    /* Build the CRC lookup table before interrupts need it. */
    CRC16_BuildTable();

    /* PD5 = USART2_TX, PD6 = USART2_RX, AF7 */
    gpio.GPIO_Pin   = PEO_PM_ESP_TX_PIN | PEO_PM_ESP_RX_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;          /* The receive line is high while idle. */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PEO_PM_ESP_TX_PORT, &gpio);
    GPIO_PinAFConfig(PEO_PM_ESP_TX_PORT, PEO_PM_ESP_TX_SRC, PEO_PM_ESP_AF);
    GPIO_PinAFConfig(PEO_PM_ESP_RX_PORT, PEO_PM_ESP_RX_SRC, PEO_PM_ESP_AF);

    /* 8 data bits, no parity, 1 stop bit; no flow control; receive and transmit enabled. */
    usart.USART_BaudRate            = PEO_PM_ESP_BAUDRATE;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(PEO_PM_ESP_USART, &usart);

    /* Circular DMA receive: USART2_RX -> DMA1_Stream5 (channel 4). */
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

    /* Normal-mode DMA transmit: DMA1_Stream6 (channel 4) -> USART2_TX.
       The length is supplied separately for each transmission. */
    DMA_DeInit(PEO_DMA_ESP_TX_STREAM);
    dma.DMA_Channel            = PEO_DMA_ESP_TX_CH;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&PEO_PM_ESP_USART->DR;
    dma.DMA_Memory0BaseAddr    = (uint32_t)g_tx_buf;
    dma.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    dma.DMA_BufferSize         = 0;   /* DMA_SetCurrDataCounter supplies the actual byte count when sending. */
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

    /* Enable USART DMA requests for RX/TX and the IDLE frame-boundary
       interrupt; do not enable per-byte RXNE interrupts. */
    USART_DMACmd(PEO_PM_ESP_USART, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(PEO_PM_ESP_USART, USART_DMAReq_Tx, ENABLE);
    USART_ITConfig(PEO_PM_ESP_USART, USART_IT_IDLE, ENABLE);
    USART_Cmd(PEO_PM_ESP_USART, ENABLE);
}

/* 7. TIM14 periodic sensor-report timer. */
#define ESP_REPORT_TIM      PEO_PM_ESP_REPORT_TIM  /* Timer peripheral scheduling periodic sensor-hub transmissions. */
#define ESP_REPORT_TIM_PSC  (PEO_PM_TIM_APB1_HZ / 10000UL - 1)     /* 84 MHz -> 10 kHz */
#define ESP_REPORT_TIM_ARR  (10000UL / SENSOR_RATE_REPORT_HZ - 1)  /* 10 kHz -> RATE Hz */

/**
 * @brief Start the timer that sends regular telemetry to ESP32-S3.
 *
 * Called by main after ESP_LINK_Init. TIM14 triggers at SENSOR_RATE_REPORT_HZ, normally
 * 50 reports per second.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void ESP_LINK_StartReport(void)
{
    TIM_TimeBaseInitTypeDef tb;  /* Timer setup record containing prescaler and period counts for the requested rate. */

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
 * @brief Send telemetry and check for loss of the ESP32-S3 heartbeat.
 *
 * The processor calls this shared vector on TIM14 updates; TIM8 encoder operation does
 * not request interrupts. After the first valid frame, a gap longer than
 * ESP_LINK_DEADMAN_MS stops all motors.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note Replaces the weak startup TIM8_TRG_COM_TIM14_IRQHandler. TIM8 is used only for
 *       encoder counting and generates no interrupts here.
 */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
    if (TIM_GetITStatus(ESP_REPORT_TIM, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(ESP_REPORT_TIM, TIM_IT_Update);
        ESP_LINK_SendHub();   /* Build the frame and start nonblocking DMA transmission. */

        /* Link-loss stop: if the S3 loses power or hangs, empty heartbeat frames
           stop arriving. Stop after ESP_LINK_DEADMAN_MS without a valid frame.
           Supervision starts only after the first valid frame (g_rx_seen != 0). */
        if (g_rx_seen &&
            (uint32_t)(TIMER_GetTick() - g_last_rx_ms) > ESP_LINK_DEADMAN_MS)
        {
            CONTROL_StopAll();
        }
    }
}


