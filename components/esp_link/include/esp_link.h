#ifndef __ESP_LINK_H__
#define __ESP_LINK_H__

/**
 * @file  esp_link.h
 * @brief Serial link (esp_link frames) between the ESP32-S3 and the F407.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

/*
 * Frame format (little-endian, identical to the F407 COMPONENTS/ESP_LINK):
 *   SOF(2) + LEN(1) + PAYLOAD(LEN) + CRC16(2)
 *
 *   Direction 1 (F407 -> ESP32, ~50 Hz): PAYLOAD = SensorData_t, 116 bytes.
 *     -> The RX task parses the byte stream; after CRC passes, the newest
 *        snapshot is pushed into esp_link_sensor_queue.
 *   Direction 2 (ESP32 -> F407): PAYLOAD = one command frame (cmd_id + params,
 *        exactly one command per frame).
 *     -> Producers (cmd_parser) push into esp_link_cmd_queue through the safe
 *        wrappers; the TX task packs and sends each frame, pausing >= 1 byte
 *        time after each frame to trigger the F407 USART2 IDLE framing.
 *
 * CRC16-CCITT: polynomial 0x1021, init 0xFFFF, low byte first.
 *
 * Queue-creation rule: every queue is created by its component's
 * xxx_queues_init() in app_main, BEFORE any task starts (cross-core pinning
 * means priority cannot guarantee order — a core-1 consumer may outrun a
 * core-0 creator). Tasks never create queues internally.
 */

#include <stdint.h>
#include <stdbool.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

/* ============================================================================
   Frame constants (aligned with F407 esp_link.h)
   ============================================================================ */
#define ESP_LINK_SOF0           0xAA
#define ESP_LINK_SOF1           0x55
#define ESP_LINK_FRAME_OVH      5       /* SOF(2) + LEN(1) + CRC(2) */
#define ESP_LINK_MAX_PAYLOAD    64      /* F407 command RX limit; NOT telemetry */
#define ESP_LINK_RX_PAYLOAD_SIZE 116U    /* F407 SensorData_t wire size, in bytes */
#define ESP_LINK_STALE_MS        500U    /* ~25 frames at 50 Hz; judged by local receive age */

/* ============================================================================
   Command IDs (aligned with F407 esp_link.h)
   ============================================================================ */
#define ESP_CMD_LED_TOGGLE      0x01
#define ESP_CMD_MOTOR           0x03    /* motor(1) + speed(2, int16 LE) */
#define ESP_CMD_PID             0x04    /* kp(4, f32 LE) + ki(4, f32 LE) */
#define ESP_CMD_MOTOR_CLOSED    0x05    /* motor(1) + speed_d100(4, int32 LE) */
#define ESP_CMD_STOP_ALL        0x06
#define ESP_CMD_ODOMETER_RESET  0x07
#define ESP_CMD_SYS_RESET       0x08    /* Whole-car soft reset: F407 NVIC_SystemReset()
                                           (decided 2026-11-16, PROTOCOL section 1.3; the
                                           upper-computer SDK re-sends PID/CAM_RES on the
                                           fresh recovery edge). */
#define ESP_CMD_LED_ON          0x09    /* On-board LED on (absolute state). */
#define ESP_CMD_LED_OFF         0x0A    /* On-board LED off (absolute state). */

#define ESP_CMD_PARAM_MAX       8

/* ============================================================================
   Structures
   ============================================================================ */

/** Command frame (element of the TX queue). */
typedef struct
{
    uint8_t cmd_id;
    uint8_t param_len;
    uint8_t param[ESP_CMD_PARAM_MAX];
    uint32_t generation;          /* Local safety epoch; never serialized on the wire. */
} esp_link_cmd_t;

/**
 * SensorData_t — byte-for-byte aligned with the F407 SensorData_t and with the
 * PROTOCOL.md section 1.2 offset table. 116 bytes (packed, including the two
 * explicit 3-byte padding fields).
 */
typedef struct __attribute__((packed))
{
    uint16_t dist_front_mm;         /* Front distance, mm. */
    uint16_t dist_left_mm;          /* Left distance, mm. */
    uint16_t dist_right_mm;         /* Right distance, mm. */
    uint8_t  dist_front_status;     /* 0 = valid reading, non-zero = invalid. */
    uint8_t  dist_left_status;
    uint8_t  dist_right_status;
    uint8_t  pad1[3];               /* Alignment padding. */
    uint32_t dist_timestamp_ms;
    float    quat_w, quat_x, quat_y, quat_z;   /* MPU6050 quaternion. */
    float    accel_x, accel_y, accel_z;        /* Acceleration, g. */
    uint32_t mpu_timestamp_ms;
    int32_t  speed_d100[4];         /* Four wheel speeds, 0.01 cm/s, positive = forward. */
    int32_t  odometer[4];           /* Four wheel odometer counts. */
    uint32_t ctrl_timestamp_ms;
    int16_t  current_mA[4];         /* Four wheel currents, mA (signed). */
    uint16_t bus_voltage_mV[4];     /* Four bus voltages, mV. */
    uint32_t ina_timestamp_ms;
    uint32_t sys_tick_ms;
    uint8_t  last_cmd_id;           /* Last command ID (echo). */
    uint8_t  last_cmd_status;       /* 0 = OK, non-zero = error code. */
    uint8_t  control_mode;          /* Control mode: 0 = closed loop, 1 = open loop. */
    uint8_t  fw_version;            /* F407 firmware version (increasing integer, 28 = v2.28). */
    uint8_t  lock;                  /* Control lock: 0 = manual (controller), 1 = autonomous (upper computer). */
    uint8_t  pad2[3];               /* Tail alignment padding (brings the struct to 116 bytes). */
} esp_link_sensor_t;

/** Local snapshot envelope (queue-only; NOT part of the 116-byte wire protocol). */
typedef struct
{
    esp_link_sensor_t sensor;
    uint32_t received_ms;         /* FreeRTOS monotonic ms; consumers must use the same time base. */
    uint32_t seq;                 /* RX single-producer valid-frame counter; increments and wraps uint32_t. */
} esp_link_snapshot_t;

/* ============================================================================
   Task entries (created by app_main; queues are already created, the RX task
   owns UART initialization)
   ============================================================================ */
void esp_link_rx_task(void *arg);   /* UART init + sensor frame receive/parse. */
void esp_link_tx_task(void *arg);   /* Command queue consume + paced frame send. */

/** @brief  Create the queues (called in app_main before task creation; idempotent). */
void esp_link_queues_init(void);

/* ============================================================================
   Queue handles (created once in app_main; command producers use the send
   wrappers, snapshot consumers use peek)
   ============================================================================ */
extern QueueHandle_t esp_link_cmd_queue;      /* In: command frames (produced by cmd_parser via the wrappers). */
extern QueueHandle_t esp_link_sensor_queue;   /* Out: esp_link_snapshot_t (1-deep overwrite; all consumers peek). */

/** @brief  Latch STOP and bump the safety generation (independent of the command queue).
 *  @retval none
 *  Safety barrier: STOP is latched independently of the ordinary command queue.
 *  TX commits a normal frame at its allowed/generation check under the safety lock.
 *  Once STOP is latched, no new normal frame can commit until an explicit resume.
 *  At most one previously committed old frame, possibly not yet physically sent,
 *  can precede STOP; the single TX task preserves their UART order. UART calls are
 *  outside the safety lock. Resume requires the observed generation and a transmitted STOP.
 *  Motion callers must require deliberate fresh input, never resume old work.
 *  Stored PID restore may resume automatically: PID changes gains only, not motion. */
void esp_link_request_stop(void);

/** @brief  Name of the most recent command successfully sent to the F407.
 *  @retval command name, or NULL if nothing has been sent yet (drives the net stats
 *          dashboard until the next command updates it). */
const char *esp_link_last_command_name(void);

/** @brief  Startup health fault: latch STOP and permanently disable resume.
 *  @retval none
 *  Ordinary input/STOP cannot clear this; only a reboot recovers. */
void esp_link_fatal_fault(void);

/** @brief  Current safety generation value.
 *  @retval the generation counter. */
uint32_t esp_link_command_generation(void);

/** @brief  Try to resume commands (clear the inhibition) if the epoch is current.
 *  @param observed_generation  Generation the caller captured earlier.
 *  @retval true if resume was allowed (generation matched, no pending STOP, no fatal). */
bool esp_link_resume_commands(uint32_t observed_generation);

/** @brief  Enqueue a command only if the producer's captured epoch is still current.
 *  @param cmd_id      Command ID (ESP_CMD_*).
 *  @param param       Parameter bytes (may be NULL only when param_len == 0).
 *  @param param_len   Number of parameter bytes.
 *  @param generation  Generation the producer captured.
 *  @retval true if the command was queued; false if rejected (STOP latched, stale, bad args). */
bool esp_link_send_command_for_generation(uint8_t cmd_id, const uint8_t *param,
                                          uint8_t param_len, uint32_t generation);

/** @brief  PID-specific forward + NVS save (param must be the 8-byte kp/ki little-endian blob).
 *  @param param      8-byte kp/ki blob (kp float LE + ki float LE).
 *  @param generation Generation the producer captured.
 *  @retval true if the PID was forwarded/queued. Persistence failure is logged separately.
 *  Serialized against the automatic restore; writes+commits only after a successful enqueue.
 *  The return value means "forwarded and queued", not "F407 confirmed execution". */
bool esp_link_send_pid_and_save(const uint8_t *param, uint32_t generation);

/** @brief  Asynchronously restore the NVS-stored PID to the F407.
 *  @retval none
 *  Called on the first sensor frame and on TCP command-client connect. TX owns the
 *  resume/enqueue and retry; if NVS has no valid PID, the F407 defaults are left alone. */
void esp_link_request_pid_push(void);

#endif /* __ESP_LINK_H__ */
