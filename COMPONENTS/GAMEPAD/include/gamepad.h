/**
 * @file  gamepad.h
 * @brief Define controller snapshots, input scaling, and driving ownership.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * Design A uses a separate physical link from WiFi/autonomous commands:
 * Xbox controller --BLE--> ESP32-C3 --USART3 (PD8/PD9, 115200 baud)--> F407.
 * C3 sends raw controller snapshots at 100 Hz. The F407 parses them, chooses
 * the active driver, and maps stick values into wheel targets. Keeping raw
 * inputs here lets dead zones and future curves/ramps be tuned on the F407
 * without reflashing C3; this does not imply all those shaping features exist.
 * Ownership: lock=0 means manual/controller; lock=1 means autonomous/host.
 * A long press of Share toggles this lock on C3; each frame carries the state
 * so the F407 can decide which source may move the rover.
 */
#ifndef __GAMEPAD_H__
#define __GAMEPAD_H__

#include "stm32f4xx.h"
#include "COMPONENTS/PEO_PINMAP.H"

/* Frame layout, byte-for-byte compatible with C3 c3_uart:
   SOF (0xAA, 0x55) + LEN (one byte, value 27) + type (0x10) + body (26 bytes)
   + CRC16-CCITT (two bytes, low byte first). Total length: 32 bytes.
   CRC covers the first 30 bytes, SOF through payload.
   Standard check vector: CRC of the nine ASCII bytes "123456789" is 0x29B1. */
#define GAMEPAD_SOF0          (0xAAU)  /* First byte of the controller frame start marker. */
#define GAMEPAD_SOF1          (0x55U)  /* Second byte of the controller frame start marker. */
#define GAMEPAD_TYPE_STATE    (0x10U)  /* Payload type identifying a controller-state snapshot. */
#define GAMEPAD_BODY_LEN      (26U)     /* Size of gamepad_payload_t in bytes. */
#define GAMEPAD_PAYLOAD_LEN   (27U)     /* type(1) + body(26) */
#define GAMEPAD_FRAME_LEN     (32U)     /* SOF2 + LEN1 + payload27 + CRC2 */

/* Configuration: serial rate, input scaling, and link timeout. */
#define GAMEPAD_BAUDRATE      (115200U) /* Baud rate in bits/s; must match C3 PEO_PM_C3_BAUDRATE. */
#define GAMEPAD_STALE_MS      (500U)    /* C3 link timeout in ms; a longer gap between valid frames stops manual driving. */
#define GAMEPAD_DEADZONE      (500)     /* Stick dead-zone half-width in raw counts around center 32768. */
#define GAMEPAD_SPEED_MAX     (4424)    /* Full-scale speed in 0.01 cm/s: 4424 means about 44.24 cm/s. */
#define GAMEPAD_TRIGGER_MAX   (1023)   /* Maximum raw trigger reading (dimensionless counts). */
#define GAMEPAD_BOOST_SPEED   (2212)   /* LB/RB speed added to one side, in 0.01 cm/s; half of full speed. */
#define GAMEPAD_STRAFE_SIGN   (1)      /* Sideways direction multiplier; use -1 if LT/RT directions are reversed on the vehicle. */

/* Ownership lock states. */
#define GAMEPAD_LOCK_MANUAL   (0U)      /* Lock off: manual driving through the controller. */
#define GAMEPAD_LOCK_AUTO     (1U)      /* Lock on: autonomous driving through the host. */

/* Button bitmap: bit positions match XB_BTN_* in C3 xbox_service.h. */
#define GAMEPAD_BTN_A         ((uint16_t)(1U << 0))  /* A-button bit mask; a new press toggles MPU6050 DMP sampling. */
#define GAMEPAD_BTN_B         ((uint16_t)(1U << 1))    /* B emergency-stop button; active regardless of which source owns driving. */
#define GAMEPAD_BTN_X         ((uint16_t)(1U << 2))   /* X toggles INA3221 current polling. */
#define GAMEPAD_BTN_Y         ((uint16_t)(1U << 3))   /* Y toggles VL53 ranging. */
#define GAMEPAD_BTN_LB        ((uint16_t)(1U << 4))   /* LB boosts both right-side wheels. */
#define GAMEPAD_BTN_RB        ((uint16_t)(1U << 5))   /* RB boosts both left-side wheels. */

/* Packed 26-byte controller payload; must match C3 c3_gamepad_payload_t
   byte for byte. Packing removes compiler-inserted gaps between fields.
   AC5/armcc uses __packed; AC6/GCC uses __attribute__((packed)). */
typedef __packed struct
{
    uint32_t seq;            /* Notification sequence number from C3; a count, not a time. */
    uint32_t disc_gen;       /* Disconnect generation counter from C3, distinguishing connection sessions. */
    uint8_t  connected;      /* Whether the controller is connected (nonzero means connected). */
    uint8_t  battery;        /* Controller battery report from C3; retained in its bridge-protocol encoding. */
    uint8_t  input_fresh;    /* Input freshness flag: C3 received a HID notification within the last 5 seconds. */
    uint16_t joy_lx;         /* Left stick horizontal raw counts, 0..65535; center 32768. */
    uint16_t joy_ly;         /* Left stick vertical raw counts, 0..65535; used for forward/reverse throttle. */
    uint16_t joy_rx;         /* Right stick horizontal raw counts, 0..65535; used for one-sided differential steering. */
    uint16_t joy_ry;         /* Right stick vertical raw counts, 0..65535; center 32768. */
    uint16_t trigger_lt;     /* Left trigger raw counts, 0..1023; released=0. */
    uint16_t trigger_rt;     /* Right trigger raw counts, 0..1023; released=0. */
    uint16_t buttons;        /* Pressed-button bit mask using GAMEPAD_BTN_* positions. */
    uint8_t  lock;           /* Driving ownership: 0=manual controller, 1=autonomous host. */
} gamepad_payload_t;

/* ============================================================================
   PUBLIC API
   ============================================================================ */

/**
 * @brief Prepare the separate ESP32-C3 controller link on USART3.
 *
 * Called by main after RCC/NVIC initialization. Receives controller frames with circular
 * DMA and IDLE interrupts; movement mapping remains in main.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void GAMEPAD_Init(void);

/**
 * @brief Read the driving-owner field from the latest valid controller frame.
 *
 * Used by host command arbitration and telemetry. This getter returns the saved field
 * and does not itself check frame freshness.
 * @note No arguments are required.
 * @retval value Protocol lock field: 0=manual/controller, 1=autonomous/host.
 */
uint8_t GAMEPAD_GetLock(void);

/**
 * @brief Read the controller connection flag in the latest saved frame.
 *
 * Available to application status displays; it does not independently time out old
 * frames.
 * @note No arguments are required.
 * @retval value Saved connection flag; nonzero means C3 reported a connected controller.
 */
uint8_t GAMEPAD_IsConnected(void);

/**
 * @brief Use the latest controller input to choose ownership and request wheel movement.
 *
 * Called repeatedly by main. Stops on ownership changes or B; in manual mode also stops
 * for stale/disconnected input, then maps sticks/triggers, clamps four wheel requests,
 * and handles new sensor-toggle presses. Autonomous mode skips mapping and sensor
 * toggles.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note Controller frames arrive at about 100 Hz. The current main loop calls this after
 *       each pass, with a 5 ms delay between passes. Manual mode maps input to four
 *       wheel targets; autonomous mode leaves driving to the host. The B stop button
 *       takes priority over either ownership state.
 */
void GAMEPAD_Process(void);

#endif /* __GAMEPAD_H__ */
