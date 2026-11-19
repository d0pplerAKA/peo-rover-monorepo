/**
 * @file  sensor_hub.h
 * @brief Define telemetry fields, sample rates, and module enable switches.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * Each sensor pushes its latest readings from its own interrupt at its own
 * rate. Call SENSOR_GetHub() to copy a consistent view of the current state.
 * Typical push paths: MPU6050 EXTI3 at 100 Hz -> SENSOR_PushMpu();
 * VL53L1X EXTI0/1/2 (profile dependent, historically about 13 Hz) ->
 * SENSOR_PushDist(); CONTROL TIM6 at 1 kHz -> SENSOR_PushWheel().
 * INA3221 TIM7 also pushes current/voltage readings at its configured rate.
 * Each group includes its last update time, measured by TIMER_GetTick() in
 * SysTick milliseconds, so consumers can judge how old each reading is.
 * SENSOR_GetHub() briefly masks interrupts while copying the structure.
 * This prevents a writer from changing fields during the copy; it does not
 * make sensors with different update rates sample at the same instant.
 * Read sensor measurements through this single entry point, then use fields
 * such as current_mA[0], dist_front_mm, and quat_w. MPU6050, VL53, and INA3221
 * do not provide separate measurement getters.
 */
#ifndef __SENSOR_HUB_H__
#define __SENSOR_HUB_H__

#include "stm32f4xx.h"

/* Distance-sensor indices: same 0/1/2 order as TOF400C_FRONT/LEFT/RIGHT. */
#define SENSOR_DIST_FRONT   (0U)  /* Index of the forward-facing distance sensor. */
#define SENSOR_DIST_LEFT    (1U)  /* Index of the left-facing distance sensor. */
#define SENSOR_DIST_RIGHT   (2U)  /* Index of the right-facing distance sensor. */

/* Number of driven wheels. */
#define SENSOR_WHEEL_COUNT  (4U)  /* Number of entries in each per-wheel telemetry array. */

/* Central sensor-rate overview (rates in Hz):
   Encoder/control: TIM6 interrupt, PEO_CTRL_LOOP_HZ in control.h, 1000 Hz.
   MPU6050: EXTI3 interrupt, MPU6050_SAMPLE_HZ in mpu6050.h, 100 Hz.
   VL53L1X range: EXTI0/1/2, PEO_PM_VL53_PROFILE in tof400c.h, profile dependent.
   INA3221 current/voltage: TIM7 polling, SENSOR_RATE_INA_HZ here, 10 Hz. */
#define SENSOR_RATE_INA_HZ     (10U)   /* INA3221 current/voltage polling frequency on TIM7, in Hz. */
#define SENSOR_RATE_REPORT_HZ  (50U)   /* Sensor-hub reporting frequency in Hz (50 Hz means one report every 20 ms). */

/* Module build switches: 0=disabled, 1=enabled.
   These select the initialization calls compiled into main and the MODULES
   section of the boot banner. They live here because print_boot_banner()
   also needs them.
   Drive-system dependencies:
   MOTOR supplies PWM outputs; both open-loop and closed-loop driving need it.
   CONTROL supplies the TIM6 1 kHz loop: read encoders -> speed PI -> PWM.
   ENCODER supplies speed feedback, consumed by CONTROL.
   Open-loop driving needs MOTOR alone; closed-loop driving needs MOTOR plus
   CONTROL and ENCODER. PEO_ENABLE_ENCODER automatically follows
   PEO_ENABLE_CONTROL: setting CONTROL=1 enables encoders; CONTROL=0 disables
   them. MOTOR remains an independent switch. */
#define PEO_ENABLE_MOTOR      1    /* Enable motor PWM, needed for either open-loop or closed-loop driving. */
#define PEO_ENABLE_CONTROL    0    /* Enable the 1 kHz PI speed controller and encoder speed measurements. */
#define PEO_ENABLE_ENCODER    PEO_ENABLE_CONTROL  /* Enable encoders automatically whenever CONTROL is enabled. */
#define PEO_ENABLE_TOF400C    1  /* Build switch: initialize ranging hardware and allow the Y-button sampling toggle. */
#define PEO_ENABLE_INA3221    1  /* Build switch: initialize current monitors and allow the X-button polling toggle. */
#define PEO_ENABLE_MPU6050    1  /* Build switch: initialize attitude hardware and allow the A-button DMP toggle. */

/* F407 firmware version sent in SensorData so the host can check peer
   compatibility (earlier notes called this a three-endpoint version check). */
#define PEO_FW_VERSION  (28U)  /* Firmware protocol/version identifier sent to the host; 28 denotes v2.28. */

/* Sensor packet: a snapshot of the latest known state.
 * Arrays use M1/M2/M3/M4 order (front left/front right/rear left/rear right).
 * Zero-initialized fields do not prove that a sensor has already sampled; use
 * its timestamp, status, and module configuration when interpreting a reading.
 * This C structure is copied directly into the serial payload, including its
 * compiler layout/padding. Peer firmware must agree on field sizes and offsets.
 * Current target layout is 112 bytes; do not reorder fields as a cosmetic edit.
 */
typedef struct
{
    /* Three VL53L1X distance measurements. */
    uint16_t dist_front_mm;        /* Front distance in millimeters. */
    uint16_t dist_left_mm;         /* Left distance in millimeters. */
    uint16_t dist_right_mm;        /* Right distance in millimeters. */
    uint8_t  dist_front_status;    /* Front ranging status; zero means a valid measurement. */
    uint8_t  dist_left_status;     /* Left ranging status; zero means a valid measurement. */
    uint8_t  dist_right_status;    /* Right ranging status; zero means a valid measurement. */
    uint32_t dist_timestamp_ms;    /* Time of the most recent distance update, in SysTick milliseconds. */

    /* ---- MPU6050 ---- */
    float quat_w;                  /* Orientation quaternion (w,x,y,z), dimensionless. */
    float quat_x; /* Quaternion x component, dimensionless. */
    float quat_y; /* Quaternion y component, dimensionless. */
    float quat_z; /* Quaternion z component, dimensionless. */
    float accel_x;                 /* Acceleration along three axes, in g (multiples of gravity). */
    float accel_y; /* Y-axis acceleration in g. */
    float accel_z; /* Z-axis acceleration in g. */
    uint32_t mpu_timestamp_ms;     /* Time of the most recent MPU update, in SysTick milliseconds. */

    /* Encoder measurements and control-loop state. */
    int32_t  speed_d100[4];        /* Four measured wheel speeds in 0.01 cm/s; positive means forward. */
    int32_t  odometer[4];          /* Four accumulated signed encoder counts; 1980 counts per wheel revolution. */
    uint32_t ctrl_timestamp_ms;    /* Time of the most recent control update, in SysTick milliseconds. */

    /* INA3221 motor currents and bus voltages, four channels. */
    int16_t  current_mA[4];        /* Four signed motor currents in milliamps. */
    uint16_t bus_voltage_mV[4];    /* Four bus voltages in millivolts. */
    uint32_t ina_timestamp_ms;     /* Time of the most recent successful INA update, in SysTick milliseconds. */

    /* Whole-snapshot timestamp and command acknowledgement (ACK). */
    uint32_t sys_tick_ms;          /* SysTick milliseconds when this complete snapshot was copied. */
    uint8_t  last_cmd_id;          /* Identifier of the most recently acknowledged command, echoed to the host. */
    uint8_t  last_cmd_status;      /* Command result: zero=OK; nonzero=ESP_CMD_ERR_* code. */
    uint8_t  control_mode;         /* Drive mode: 0=closed-loop speed control; 1=open-loop PWM. */
    uint8_t  fw_version;           /* F407 firmware version as an increasing integer; for example, 28 denotes v2.28. */
    uint8_t  lock;                 /* Driving ownership: 0=manual/controller; 1=autonomous/host. */
} SensorData_t;

/* Push interfaces: sensor interrupt handlers call these to publish readings;
   each update receives a timestamp automatically. */

/**
 * @brief Store the latest attitude and acceleration as one sensor group.
 *
 * Called by the MPU FIFO callback in EXTI3; adds the current millisecond timestamp.
 * @param qw Quaternion scalar component w, dimensionless.
 * @param qx Quaternion x component, dimensionless.
 * @param qy Quaternion y component, dimensionless.
 * @param qz Quaternion z component, dimensionless.
 * @param ax X-axis acceleration in g (1 g is standard gravity).
 * @param ay Y-axis acceleration in g.
 * @param az Z-axis acceleration in g.
 * @retval None This function does not return a value.
 */
void SENSOR_PushMpu(float qw, float qx, float qy, float qz,
                    float ax, float ay, float az);

/**
 * @brief Store one range sensor result and refresh the distance-group timestamp.
 *
 * Called by EXTI0/1/2 after a successful sensor read. The three distances share a
 * timestamp recording the most recent update to any one of them.
 * @param which SENSOR_DIST_FRONT, SENSOR_DIST_LEFT, or SENSOR_DIST_RIGHT.
 * @param mm Measured distance in millimeters; interpret it together with status.
 * @param status VL53L1X range-status code; zero means a valid measurement.
 * @retval None This function does not return a value.
 */
void SENSOR_PushDist(uint8_t which, uint16_t mm, uint8_t status);

/**
 * @brief Store one measured wheel speed and signed accumulated count.
 *
 * Called for each wheel by the TIM6 control interrupt; updates the control-group
 * timestamp.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right.
 * @param speed_d100 Measured signed speed in 0.01 cm/s, positive for forward.
 * @param odometer Accumulated signed encoder counts; 1980 counts per wheel revolution.
 * @retval None This function does not return a value.
 */
void SENSOR_PushWheel(uint8_t motor, int32_t speed_d100, int32_t odometer);

/**
 * @brief Copy all four motor-current and bus-voltage readings into telemetry.
 *
 * Called by TIM7 only after both INA3221 reads succeed; records the update time
 * automatically.
 * @param current_mA Readable array of four signed currents in milliamps, in M1..M4
 *                   order.
 * @param bus_voltage_mV Readable array of four unsigned bus voltages in millivolts, in
 *                       M1..M4 order.
 * @retval None This function does not return a value.
 */
void SENSOR_PushIna(int16_t *current_mA, uint16_t *bus_voltage_mV);

/**
 * @brief Record which command was handled and its result for the host.
 *
 * Called by command handling and queue-error paths. A critical section keeps the command
 * ID and result together; only the latest acknowledgement is retained.
 * @param cmd_id Command identifier to echo, from ESP_CMD_*.
 * @param status ESP_CMD_OK or an ESP_CMD_ERR_* result code.
 * @retval None This function does not return a value.
 */
void SENSOR_SetCmdAck(uint8_t cmd_id, uint8_t status);

/* Copy a consistent snapshot. */

/**
 * @brief Copy the complete current telemetry state into a caller-owned object.
 *
 * Called by ESP_LINK reporting and available to main. Briefly masks interrupts, adds
 * snapshot time/mode/version/ownership, copies the packet, and restores the prior
 * interrupt mask.
 * @param out Writable SensorData_t destination; NULL is accepted and performs no copy.
 * @retval None This function does not return a value.
 * @note Masking prevents writes during the short copy (intended to take only
 *       microseconds); this may be called from the main loop. Sensor timestamps still
 *       describe different acquisition times.
 */
void SENSOR_GetHub(SensorData_t *out);

/* Startup diagnostics (boot banner), moved here from main.c to keep main focused on startup and scheduling. */

/**
 * @brief Print a startup summary so teammates can inspect the rover configuration.
 *
 * Called once by main after component setup. Shows actual clocks, chip/build identity,
 * drivetrain scaling, PI defaults, sensor setup/test results, and enabled modules on
 * USART1.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void print_boot_banner(void);

#endif /* __SENSOR_HUB_H__ */
