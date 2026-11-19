/**
 * @file  gamepad.c
 * @brief Receive ESP32-C3 controller frames and map manual input to rover movement.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * USART3_RX -> circular DMA1_Stream1 (channel 4) receives 32-byte gamepad
 * frames. USART IDLE marks a frame boundary. Frames contain SOF 0xAA55,
 * LEN, type, body, and CRC16; valid frames replace the input snapshot.
 * GAMEPAD_Process() in main checks link loss, ownership, and joystick mapping.
 */
#include "include/gamepad.h"

#include "COMPONENTS/CONTROL/include/control.h"
#include "COMPONENTS/MOTOR/include/motor.h"
#include "COMPONENTS/TIMER/include/timer.h"
#include "COMPONENTS/SENSOR_HUB/include/sensor_hub.h"
#include "COMPONENTS/TOF400C/include/tof400c.h"
#include "COMPONENTS/INA3221/include/ina3221.h"
#include "COMPONENTS/MPU6050/include/mpu6050.h"

#include <string.h>

/* CRC16-CCITT: polynomial 0x1021, initial value 0xFFFF, table lookup.
   The format matches esp_link and c3_uart. */
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
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        g_crc_table[i] = crc;
    }
    g_crc_ready = 1;
}

/**
 * @brief Calculate the integrity check for a C3 gamepad frame.
 *
 * Called by process_frame before accepting a controller snapshot; uses the same
 * CRC16-CCITT convention as the S3 link.
 * @param data Readable input bytes; normally the first 30 frame bytes.
 * @param len Number of bytes included in the checksum.
 * @retval value Calculated 16-bit CRC, compared with the low-byte-first received CRC.
 */
static uint16_t CRC16_Calc(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    if (!g_crc_ready)
        CRC16_BuildTable();

    while (len--)
        crc = (uint16_t)((crc << 8) ^ g_crc_table[((crc >> 8) ^ *data++) & 0xFFU]);
    return crc;
}

/* Gamepad snapshot: written by the USART3 interrupt and consumed by GAMEPAD_Process(). */
static volatile gamepad_payload_t g_pad;  /* Latest CRC-valid controller snapshot, shared between USART3 interrupt and main. */
static volatile uint32_t g_last_rx_ms = 0;  /* SysTick millisecond timestamp of the latest valid frame on this link. */
/* Ownership state processed on the previous main-loop pass; detects handovers. */
static volatile uint8_t  g_prev_lock  = GAMEPAD_LOCK_MANUAL;
static uint16_t g_prev_buttons = 0;   /* Previous button bitmap, used to detect newly pressed buttons. */
#if PEO_ENABLE_TOF400C
static uint8_t  g_vl53_on = 0;        /* Whether VL53 ranging is running; off at startup, toggled by Y. */
#endif
#if PEO_ENABLE_INA3221
static uint8_t  g_ina_on  = 0;        /* Whether INA3221 polling is running; off at startup, toggled by X. */
#endif
#if PEO_ENABLE_MPU6050
static uint8_t  g_dmp_on  = 0;        /* Whether DMP attitude sampling is running; off at startup, toggled by A. */
#endif

/* Circular DMA reception: USART3_RX -> DMA1_Stream1, channel 4.
   DMA copies bytes into g_rx_dma_buf. A USART IDLE interrupt treats a pause
   as the frame boundary, reads NDTR for the byte count, then validates and
   parses the full frame. This replaces per-byte RXNE interrupts with one
   interrupt per frame (nominally 100 Hz). */
#define GAMEPAD_RX_BUF_SIZE  64   /* DMA receive-buffer capacity in bytes; leaves headroom above the 32-byte frame size. */

/* A uint32_t array provides four-byte alignment for the DMA memory address. */
static uint32_t g_rx_dma_buf[GAMEPAD_RX_BUF_SIZE / 4];  /* Aligned DMA receive storage; interpreted as a byte array by the parser. */

/**
 * @brief Validate a complete C3 frame and replace the latest controller snapshot.
 *
 * Called by USART3_IRQHandler. Incorrect length, SOF, message type, or CRC leaves the
 * previous snapshot and timestamp unchanged.
 * @param len Number of DMA-received bytes; only the exact 32-byte frame length is
 *            accepted.
 * @retval None This function does not return a value.
 */
static void process_frame(uint16_t len)
{
    uint8_t *buf = (uint8_t *)g_rx_dma_buf;  /* Byte-oriented view of the controller DMA receive buffer. */
    gamepad_payload_t pad;  /* Local controller snapshot used for validation or one main-loop mapping pass. */
    uint16_t crc_calc, crc_rx; /* Calculated and received CRC values for the gamepad frame. */

    if (len != GAMEPAD_FRAME_LEN)                          /* A gamepad frame must contain exactly 32 bytes. */
        return;
    if (buf[0] != GAMEPAD_SOF0 || buf[1] != GAMEPAD_SOF1)  /* SOF 0xAA55 */
        return;
    if (buf[2] != GAMEPAD_PAYLOAD_LEN)                     /* LEN = 27 */
        return;
    if (buf[3] != GAMEPAD_TYPE_STATE)                      /* type = 0x10 */
        return;

    /* CRC covers SOF through payload: 30 bytes = 2 + 1 + 1 + 26; exclude the CRC itself. */
    crc_calc = CRC16_Calc(buf, 30);
    crc_rx   = (uint16_t)(buf[30] | ((uint16_t)buf[31] << 8));
    if (crc_calc != crc_rx)
        return;

    /* The 26-byte body starts at buf[4]. Copy it into a local payload object
       before assigning fields, avoiding casts of raw bytes to typed pointers. */
    memcpy(&pad, &buf[4], GAMEPAD_BODY_LEN);
    g_pad        = pad;
    g_last_rx_ms = TIMER_GetTick();
}

/**
 * @brief Collect a complete controller frame when the serial line becomes idle.
 *
 * Called by the processor for USART3. Clears IDLE, validates DMA data, then restarts DMA
 * at the beginning of the receive buffer.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(PEO_PM_UART3_USART, USART_IT_IDLE) != RESET)
    {
        uint16_t len;  /* Number of bytes received before the USART IDLE event. */

        /* Clear IDLE by reading SR and then DR, following the RM0090 software sequence. */
        (void)PEO_PM_UART3_USART->SR;
        (void)PEO_PM_UART3_USART->DR;

        /* Bytes received = buffer capacity minus the DMA remaining-byte count. */
        len = (uint16_t)(GAMEPAD_RX_BUF_SIZE
                         - DMA_GetCurrDataCounter(PEO_DMA_GAMEPAD_RX_STREAM));

        process_frame(len);

        /* Reset DMA using the RM0090 sequence so the next frame starts at buffer byte zero. */
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

/* Joystick mapping: unsigned input 0..65535, center 32768, dead zone
   within +/-500 counts, output limited to +/-4424 in 0.01 cm/s. */
/**
 * @brief Turn a joystick position into a signed movement request with a quiet center.
 *
 * Called by GAMEPAD_Process for throttle and steering. Small center noise produces zero;
 * the remaining travel is scaled linearly.
 * @param raw Unsigned stick reading 0..65535; center is 32768.
 * @retval value Signed request in 0.01 cm/s, within +/-GAMEPAD_SPEED_MAX.
 */
static int32_t map_axis(uint16_t raw)
{
    int32_t v  = (int32_t)raw - 32768;   /* -32768 .. +32767 */
    int32_t dz = GAMEPAD_DEADZONE;  /* Joystick dead-zone half-width in raw input counts. */

    if (v > -dz && v < dz)
        return 0;                        /* Treat small movements within the dead zone as zero. */

    if (v >= dz)
        return (v - dz) * GAMEPAD_SPEED_MAX / (32767 - dz);
    else
        return (v + dz) * GAMEPAD_SPEED_MAX / (32768 - dz);
}

/* Trigger mapping: 0..1023 raw counts (released=0) maps linearly to
   0..4424 in 0.01 cm/s; more pressure requests more movement. */
/**
 * @brief Turn trigger pressure into a nonnegative sideways-speed contribution.
 *
 * Called by GAMEPAD_Process for LT and RT; readings above the defined maximum are
 * clamped.
 * @param raw Trigger reading in raw counts, normally 0..1023; zero means released.
 * @retval value Speed contribution from zero to GAMEPAD_SPEED_MAX, in 0.01 cm/s.
 */
static int32_t map_trigger(uint16_t raw)
{
    if (raw > GAMEPAD_TRIGGER_MAX)
        raw = GAMEPAD_TRIGGER_MAX;
    return (int32_t)raw * GAMEPAD_SPEED_MAX / GAMEPAD_TRIGGER_MAX;
}

/* Initialization. */
/**
 * @brief Prepare the separate ESP32-C3 controller link on USART3.
 *
 * Called by main after RCC/NVIC initialization. Receives controller frames with circular
 * DMA and IDLE interrupts; movement mapping remains in main.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void GAMEPAD_Init(void)
{
    GPIO_InitTypeDef  gpio;  /* GPIO setup record: selects pin routing, electrical drive, pulls, and switching speed. */
    USART_InitTypeDef usart;  /* Serial-port setup record: baud rate, byte format, and enabled directions. */
    DMA_InitTypeDef   dma;  /* DMA setup record: peripheral/memory addresses, byte count, and transfer behavior. */

    /* GPIO: PD8 (TX) and PD9 (RX), AF7. RCC_Clock_Init() has already enabled the clocks. */
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;      /* Pull RX high to match its idle electrical level. */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin = PEO_PM_UART3_TX_PIN;
    GPIO_Init(PEO_PM_UART3_TX_PORT, &gpio);
    gpio.GPIO_Pin = PEO_PM_UART3_RX_PIN;
    GPIO_Init(PEO_PM_UART3_RX_PORT, &gpio);

    GPIO_PinAFConfig(PEO_PM_UART3_TX_PORT, PEO_PM_UART3_TX_SRC, PEO_PM_UART3_AF);
    GPIO_PinAFConfig(PEO_PM_UART3_RX_PORT, PEO_PM_UART3_RX_SRC, PEO_PM_UART3_AF);

    /* USART3: 115200 baud, 8 data bits, no parity, 1 stop bit, no flow control. */
    usart.USART_BaudRate            = GAMEPAD_BAUDRATE;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(PEO_PM_UART3_USART, &usart);

    /* Circular DMA receive: USART3_RX -> DMA1_Stream1 (channel 4). */
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

    /* Enable RX DMA requests and IDLE frame-boundary interrupts;
       per-byte RXNE interrupts are not used. */
    USART_DMACmd(PEO_PM_UART3_USART, USART_DMAReq_Rx, ENABLE);
    USART_ITConfig(PEO_PM_UART3_USART, USART_IT_IDLE, ENABLE);
    USART_Cmd(PEO_PM_UART3_USART, ENABLE);
}

/* ============================================================================
   Public API
   ============================================================================ */
/**
 * @brief Read the driving-owner field from the latest valid controller frame.
 *
 * Used by host command arbitration and telemetry. This getter returns the saved field
 * and does not itself check frame freshness.
 * @note No arguments are required.
 * @retval value Protocol lock field: 0=manual/controller, 1=autonomous/host.
 */
uint8_t GAMEPAD_GetLock(void)
{
    return g_pad.lock;
}

/**
 * @brief Read the controller connection flag in the latest saved frame.
 *
 * Available to application status displays; it does not independently time out old
 * frames.
 * @note No arguments are required.
 * @retval value Saved connection flag; nonzero means C3 reported a connected controller.
 */
uint8_t GAMEPAD_IsConnected(void)
{
    return g_pad.connected;
}

/**
 * @brief Toggle ranging requests for all three distance sensors.
 *
 * Called by GAMEPAD_Process on a new Y press in valid manual mode. Does nothing when
 * TOF400C is disabled at build time.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
static void toggle_vl53(void)
{
#if PEO_ENABLE_TOF400C
    g_vl53_on = !g_vl53_on;
    if (g_vl53_on)
    {
        TOF400C_StartRanging_Interrupt(TOF400C_FRONT);
        TOF400C_StartRanging_Interrupt(TOF400C_LEFT);
        TOF400C_StartRanging_Interrupt(TOF400C_RIGHT);
    }
    else
    {
        TOF400C_StopRanging_Interrupt(TOF400C_FRONT);
        TOF400C_StopRanging_Interrupt(TOF400C_LEFT);
        TOF400C_StopRanging_Interrupt(TOF400C_RIGHT);
    }
#endif
}

/**
 * @brief Toggle periodic motor-current polling.
 *
 * Called by GAMEPAD_Process on a new X press in valid manual mode. Starts or stops TIM7,
 * and does nothing when INA3221 is disabled.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
static void toggle_ina(void)
{
#if PEO_ENABLE_INA3221
    g_ina_on = !g_ina_on;
    if (g_ina_on)
        INA3221_PollInit();
    else
        INA3221_Stop();
#endif
}

/**
 * @brief Toggle the MPU6050 attitude-data engine.
 *
 * Called by GAMEPAD_Process on a new A press in valid manual mode. Sends start/stop
 * requests to DMP; does nothing when MPU6050 is disabled.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
static void toggle_dmp(void)
{
#if PEO_ENABLE_MPU6050
    g_dmp_on = !g_dmp_on;
    if (g_dmp_on)
        MPU6050_Start();
    else
        MPU6050_Stop();
#endif
}

/**
 * @brief Use the latest controller input to choose ownership and request wheel movement.
 *
 * Called repeatedly by main. Stops on ownership changes or B; in manual mode also stops
 * for stale/disconnected input, then maps sticks/triggers, clamps four wheel requests,
 * and handles new sensor-toggle presses. Autonomous mode skips mapping and sensor
 * toggles.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void GAMEPAD_Process(void)
{
    gamepad_payload_t pad;  /* Local controller snapshot used for validation or one main-loop mapping pass. */
    /* Saved interrupt-mask state; restore it so callers that already masked interrupts stay protected. */
    uint32_t primask = __disable_irq();
    pad = g_pad;
    if (!primask)
        __enable_irq();

    uint32_t now = TIMER_GetTick();  /* Current SysTick time in milliseconds for input-age checks. */
    uint8_t  lock = pad.lock;  /* Driving owner from the snapshot: manual=0, autonomous=1. */
    int32_t  throttle, steer, strafe, m1, m2, m3, m4; /* Forward, steering, sideways, and final wheel requests, all in 0.01 cm/s. */

    /* Stop immediately when ownership changes, preventing a sudden speed change
       caused by handing control from one source to the other. */
    if (lock != g_prev_lock)
    {
        CONTROL_StopAll();
        g_prev_lock = lock;
    }

    if (pad.buttons & GAMEPAD_BTN_B)
    {
        CONTROL_StopAll();            /* Emergency-stop button: takes effect for either ownership state. */
        return;
    }

    /* Autonomous ownership: host commands control the rover; skip joystick mapping. */
    if (lock == GAMEPAD_LOCK_AUTO)
        return;

    /* Manual ownership: stop if the C3 link or controller connection is lost. */
    if ((uint32_t)(now - g_last_rx_ms) >= GAMEPAD_STALE_MS)
    {
        CONTROL_StopAll();            /* No recent valid frame from C3. */
        return;
    }
    if (!pad.connected || !pad.input_fresh)
    {
        CONTROL_StopAll();            /* Controller disconnected or its input is stale. */
        return;
    }
    /* Map joystick and trigger readings into wheel requests. */
    throttle = map_axis(pad.joy_ly);
    steer    = map_axis(pad.joy_rx);
    strafe   = GAMEPAD_STRAFE_SIGN *
               (map_trigger(pad.trigger_rt) - map_trigger(pad.trigger_lt));  /* Positive means movement to the right. */

    /* Mecanum mixing: M1=front left, M2=front right, M3=rear left, M4=rear right.
       Combine differential steering with sideways movement: M1/M4 receive
       +strafe and the opposite diagonal M2/M3 receives -strafe. */
    m1 = throttle + strafe;               /* Front left wheel. */
    m2 = throttle + steer - strafe;       /* Front right wheel. */
    m3 = throttle - strafe;               /* Rear left wheel. */
    m4 = throttle + steer + strafe;       /* Rear right wheel. */

    /* LB boosts the two right wheels; RB boosts the two left wheels. */
    if (pad.buttons & GAMEPAD_BTN_LB) { m2 += GAMEPAD_BOOST_SPEED; m4 += GAMEPAD_BOOST_SPEED; }
    if (pad.buttons & GAMEPAD_BTN_RB) { m1 += GAMEPAD_BOOST_SPEED; m3 += GAMEPAD_BOOST_SPEED; }

    /* Clamp each wheel request to the allowed speed range. */
    if (m1 >  GAMEPAD_SPEED_MAX) m1 =  GAMEPAD_SPEED_MAX;
    if (m1 < -GAMEPAD_SPEED_MAX) m1 = -GAMEPAD_SPEED_MAX;
    if (m2 >  GAMEPAD_SPEED_MAX) m2 =  GAMEPAD_SPEED_MAX;
    if (m2 < -GAMEPAD_SPEED_MAX) m2 = -GAMEPAD_SPEED_MAX;
    if (m3 >  GAMEPAD_SPEED_MAX) m3 =  GAMEPAD_SPEED_MAX;
    if (m3 < -GAMEPAD_SPEED_MAX) m3 = -GAMEPAD_SPEED_MAX;
    if (m4 >  GAMEPAD_SPEED_MAX) m4 =  GAMEPAD_SPEED_MAX;
    if (m4 < -GAMEPAD_SPEED_MAX) m4 = -GAMEPAD_SPEED_MAX;

    {
#if PEO_ENABLE_CONTROL
        const int32_t targets[4] = { m1, m2, m3, m4 };  /* Four requested wheel speeds in 0.01 cm/s, copied together into CONTROL. */
        CONTROL_SetTargetSpeedAll(targets);
#elif PEO_ENABLE_MOTOR
        /* Open-loop fallback when there is no TIM6 speed controller: convert
           speed to direct PWM per-mille. GAMEPAD_SPEED_MAX (4424 in 0.01 cm/s)
           maps to MOTOR_SPEED_MAX (1000 per-mille). */
        MOTOR_SetSpeed(MOTOR_M1, (int16_t)(m1 * MOTOR_SPEED_MAX / GAMEPAD_SPEED_MAX));
        MOTOR_SetSpeed(MOTOR_M2, (int16_t)(m2 * MOTOR_SPEED_MAX / GAMEPAD_SPEED_MAX));
        MOTOR_SetSpeed(MOTOR_M3, (int16_t)(m3 * MOTOR_SPEED_MAX / GAMEPAD_SPEED_MAX));
        MOTOR_SetSpeed(MOTOR_M4, (int16_t)(m4 * MOTOR_SPEED_MAX / GAMEPAD_SPEED_MAX));
#endif
    }

    /* New button presses: Y toggles ranging, X toggles current polling,
       and A toggles attitude sampling. Holding a button does not retrigger it. */
    {
        /* Bits for buttons pressed now but not on the previous processed pass. */
        uint16_t edge = (g_prev_buttons ^ pad.buttons) & pad.buttons;
        g_prev_buttons = pad.buttons;
        if (edge & GAMEPAD_BTN_Y) toggle_vl53();
        if (edge & GAMEPAD_BTN_X) toggle_ina();
        if (edge & GAMEPAD_BTN_A) toggle_dmp();
    }
}
