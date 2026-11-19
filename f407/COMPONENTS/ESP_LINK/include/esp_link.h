/**
 * @file  esp_link.h
 * @brief Define the ESP32-S3 frame format, host commands, and queue interface.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * F407 <-> ESP32-S3 uses full-duplex USART2 (PD5=TX, PD6=RX, AF7).
 * Frame: SOF(2 bytes) + LEN(1 byte) + PAYLOAD(LEN bytes) + CRC16(2 bytes).
 * F407 -> ESP32 payload: SensorData_t, reported continuously.
 * ESP32 -> F407 payload: one command ID followed by its parameters.
 * CRC16-CCITT uses polynomial 0x1021 and initial value 0xFFFF; it covers
 * all bytes from SOF through payload, excluding the CRC itself.
 *
 * Sender rules:
 * 1. Exactly one command per frame: one-byte ID plus command-specific parameters.
 * 2. Earlier link notes described Xbox BLE HID being decoded by an ESP32 and
 *    converted to MOTOR/MOTOR_CLOSED/STOP_ALL commands rather than raw HID.
 *    In the current four-endpoint system, the separate ESP32-C3 gamepad bridge
 *    sends controller snapshots on USART3; GAMEPAD maps them on the F407.
 * 3. Append the CRC after the payload, low byte first (little-endian).
 * 4. Leave at least one byte-time of silence after each frame so the F407
 *    USART2 IDLE interrupt can recognize the frame boundary.
 */
#ifndef __ESP_LINK_H__
#define __ESP_LINK_H__

#include "stm32f4xx.h"
#include "COMPONENTS/PEO_PINMAP.H"

/* Frame-format constants. Lengths below are measured in bytes. */
#define ESP_LINK_SOF0           0xAA    /* First SOF byte: the high byte of the marker 0xAA55. */
#define ESP_LINK_SOF1           0x55    /* Second SOF byte: the low byte of the marker 0xAA55. */
/* Incoming command payload limit in bytes; current commands use at most 9 bytes, leaving headroom. */
#define ESP_LINK_MAX_PAYLOAD    64
#define ESP_LINK_FRAME_OVH      5       /* SOF(2) + LEN(1) + CRC(2) */
/* Link-loss timeout in ms: 500 ms without valid frames calls CONTROL_StopAll(); S3 heartbeat period is
 * 200 ms.
 */
#define ESP_LINK_DEADMAN_MS     500

/* Command IDs: ESP32 -> F407, one command per payload.
   Layout: cmd_id (1 byte) + command-specific parameter bytes.
   LED_TOGGLE: no parameters; reverse the LED state.
   LED_ON / LED_OFF: no parameters; set an absolute LED state.
   MOTOR: motor (1 byte) + signed speed (2 bytes); direct open-loop PWM.
   PID: kp (4-byte float) + ki (4-byte float); gains shared by all wheels.
   MOTOR_CLOSED: motor (1 byte) + speed_d100 (4-byte signed integer);
                 closed-loop target in 0.01 cm/s.
   STOP_ALL: no parameters; emergency stop.
   ODOMETER_RESET: no parameters; reset all four wheel distances. */
#define ESP_CMD_LED_TOGGLE      0x01  /* Wire command ID: reverse the onboard LED state; no parameters. */
#define ESP_CMD_MOTOR           0x03  /* Wire command ID: motor index plus signed PWM per-mille for open-loop drive. */
#define ESP_CMD_PID             0x04  /* Wire command ID: two floats set the shared incremental PI gains. */
#define ESP_CMD_MOTOR_CLOSED    0x05  /* Wire command ID: motor index plus signed target speed in 0.01 cm/s. */
#define ESP_CMD_STOP_ALL        0x06  /* Wire command ID: clear all wheel requests and coast; no parameters. */
#define ESP_CMD_ODOMETER_RESET  0x07  /* Wire command ID: zero all accumulated wheel counts; no parameters. */
#define ESP_CMD_SYS_RESET       0x08    /* Reset the complete F407 with NVIC_SystemReset(); design finalized 2026-11-16. */
#define ESP_CMD_LED_ON          0x09    /* Turn the onboard LED on (absolute state). */
#define ESP_CMD_LED_OFF         0x0A    /* Turn the onboard LED off (absolute state). */

/* Command-result codes echoed through sensorhub.last_cmd_status. */
#define ESP_CMD_OK              0x00    /* Command completed successfully. */
#define ESP_CMD_ERR_INVALID     0x01    /* Unknown command ID, disallowed command, or invalid parameter value. */
#define ESP_CMD_ERR_BADLEN      0x02    /* Incorrect parameter length. */
#define ESP_CMD_ERR_BADMOTOR    0x03    /* Wheel index is outside the supported range. */

/* Command structure: one element in the pending-command queue.
   ESP32 packages host commands as command ID plus parameters. Older protocol
   notes also allowed gamepad-mapped commands in this format; current gamepad
   snapshots use USART3. The USART2 IDLE interrupt validates and enqueues;
   main removes and executes commands so lengthy actions stay out of the ISR. */
#define ESP_CMD_MAX_PARAM       8   /* Maximum parameter bytes; PID needs 8 bytes for kp plus ki. */

typedef struct
{
    uint8_t id;              /* Command identifier from ESP_CMD_*. */
    uint8_t param_len;       /* Number of meaningful bytes in param. */
    uint8_t param[ESP_CMD_MAX_PARAM]; /* Copied parameter bytes; only the first param_len entries are meaningful. */
} ESP_Cmd_t;

/* ============================================================================
   PUBLIC API
   ============================================================================ */

/**
 * @brief Prepare the USART2 host link, CRC table, and transmit/receive DMA.
 *
 * Called by main after RCC/NVIC setup. Uses PD5/PD6 at PEO_PM_ESP_BAUDRATE;
 * ESP_LINK_StartReport starts periodic telemetry separately.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note RCC and NVIC components own clock and interrupt-controller setup.
 */
void ESP_LINK_Init(void);

/**
 * @brief Start the timer that sends regular telemetry to ESP32-S3.
 *
 * Called by main after ESP_LINK_Init. TIM14 triggers at SENSOR_RATE_REPORT_HZ, normally
 * 50 reports per second.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note Call after ESP_LINK_Init(); DMA sends without waiting for all serial bytes,
 *       allowing the control loop to continue.
 */
void ESP_LINK_StartReport(void);

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
uint16_t ESP_LINK_CRC16(const uint8_t *data, uint16_t len);

/* Command-queue API: enqueue in the interrupt, dequeue and execute in main. */

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
uint8_t ESP_LINK_PushCommand(uint8_t id, const uint8_t *param, uint8_t param_len);

/**
 * @brief Take the oldest waiting command out of the queue.
 *
 * Consumer API called by main; on success main passes the copied command to
 * ESP_LINK_ExecCommand.
 * @param out Non-NULL writable command structure receiving the queue item.
 * @retval 1 A command was copied to out.
 * @retval 0 The queue was empty; out was not changed.
 */
uint8_t ESP_LINK_PopCommand(ESP_Cmd_t *out);

/**
 * @brief Check whether an urgent STOP request is waiting for main.
 *
 * Main checks this before removing ordinary commands, so a full command queue cannot
 * prevent a STOP request.
 * @note No arguments are required.
 * @retval value Nonzero if STOP is pending; zero otherwise.
 */
uint8_t ESP_LINK_StopPending(void);

/**
 * @brief Acknowledge consumption of the independent STOP request.
 *
 * Main clears the flag immediately before calling CONTROL_StopAll and publishing the
 * acknowledgement.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void ESP_LINK_ClearStop(void);

/**
 * @brief Apply a host command and record whether it was accepted.
 *
 * Called by main after dequeuing. Manual ownership blocks host motor and PI-tuning
 * requests; other commands include LED control, odometry reset, STOP, and system reset.
 * @param cmd Non-NULL queued command with its ID and parameter bytes.
 * @retval None This function does not return a value.
 */
void ESP_LINK_ExecCommand(const ESP_Cmd_t *cmd);

#endif /* __ESP_LINK_H__ */
