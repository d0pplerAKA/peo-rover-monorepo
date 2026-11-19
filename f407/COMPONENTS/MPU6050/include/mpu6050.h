/**
 * @file  mpu6050.h
 * @brief Define MPU6050 setup, calibration status, and sampling controls.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdarg.h>
#include <stdio.h>

#include "stm32f4xx.h"
#include "COMPONENTS/PEO_PINMAP.H"

#include "COMPONENTS/motion_driver_dmp/driver/eMPL/inv_mpu.h"
#include "COMPONENTS/motion_driver_dmp/driver/eMPL/inv_mpu_dmp_motion_driver.h"
#include "COMPONENTS/motion_driver_dmp/mllite/ml_math_func.h"

#include "COMPONENTS/I2C/include/i2c.h"
#include "COMPONENTS/DELAY/include/delay.h"
#include "COMPONENTS/BUILTIN_LED/builtin_led.h"

/* MPU6050 DMP configuration. MPU6050_SAMPLE_HZ is a rate in Hz,
   not the raw SMPLRT_DIV divider. eMPL mpu_set_sample_rate(rate) and
   dmp_set_fifo_rate(rate) both accept Hz and calculate the divider internally
   (SMPLRT_DIV=1000/rate-1). The DMP output rate is limited to 200 Hz. */
#define MPU6050_SAMPLE_HZ       (100U)          /* Sensor sampling rate and DMP output rate, in Hz. */

#define MPU6050_Q30             (1073741824.0f) /* q30 -> float(1 << 30)    */
#define MPU6050_ACCEL_SENS      (16384.0f)      /* Accelerometer raw counts per g for the +/-2 g range. */

#define MPU6050_WHO_AM_I_REG    (0x75)  /* Identity register address read before loading the DMP firmware. */
#define MPU6050_WHO_AM_I_EXP    (0x68)          /* AD0 = GND */

/* eMPL platform adapter functions, implemented in mpu6050.c. */
/**
 * @brief Register the motion-driver callback and connect the MPU data-ready interrupt.
 *
 * Called by the eMPL mpu_init routine. Saves the callback, configures PE3 as an input,
 * and enables rising-edge EXTI3 detection; NVIC setup has already run.
 * @param int_param Callback descriptor; only its cb member is used.
 * @retval 0 Callback descriptor accepted and GPIO/EXTI configured.
 * @retval -1 int_param was NULL.
 */
int reg_int_cb(struct int_param_s *int_param);
/**
 * @brief Send a formatted motion-driver diagnostic message to the USART1 console.
 *
 * Backend called by eMPL logging when logging is enabled. Formats into a bounded
 * 128-byte buffer and optionally prefixes a tag.
 * @param priority Logging priority supplied by eMPL; ignored by this backend.
 * @param tag Optional text label; NULL or empty means no prefix.
 * @param fmt Non-NULL printf-style format string.
 * @param ... Values corresponding to the format placeholders.
 * @retval 0 Logging call completed; this implementation does not report
 *           formatting/transmit errors.
 */
int _MLPrintLog(int priority, const char *tag, const char *fmt, ...);

/* Application API for the MPU6050 DMP attitude sensor. */
/**
 * @brief Initialize the attitude sensor and cache its startup result.
 *
 * Called by main. Prepares I2C1, verifies WHO_AM_I, and configures DMP; GAMEPAD starts
 * actual DMP output later.
 * @note No arguments are required.
 * @retval 0 Sensor initialization succeeded.
 * @retval 1 WHO_AM_I read or identity check failed.
 * @retval 2 DMP setup failed.
 */
uint8_t MPU6050_Init(void); /* 0=success; 1=identity check failed; 2=DMP setup failed (sampling not started). */
/**
 * @brief Load and configure the sensor processing engine without starting its output.
 *
 * Called by MPU6050_Init. Sets sensor/FIFO rates, mounting orientation, and
 * quaternion/acceleration features; self-test outcome is cached separately and does not
 * change this setup return code.
 * @note No arguments are required.
 * @retval 0 Setup succeeded; self-test result must be queried separately.
 * @retval 1 mpu_init failed.
 * @retval 2 Enabling gyro/accelerometer failed.
 * @retval 3 FIFO configuration failed.
 * @retval 4 Setting the sample rate failed.
 * @retval 5 Loading DMP firmware failed.
 * @retval 6 Applying mounting orientation failed.
 * @retval 7 Enabling DMP features failed.
 * @retval 8 Setting DMP FIFO rate failed.
 */
uint8_t MPU6050_DMP_Init(void);                                 /* Zero means DMP configuration succeeded. */
/**
 * @brief Enable DMP output so attitude samples can arrive by interrupt.
 *
 * Called by the gamepad A toggle after successful setup; data-ready pulses then occur at
 * MPU6050_SAMPLE_HZ.
 * @note No arguments are required.
 * @retval 0 DMP start request succeeded.
 * @retval 1 mpu_set_dmp_state(1) failed.
 */
uint8_t MPU6050_Start(void); /* Zero means sampling started successfully and data-ready interrupts can begin. */
/**
 * @brief Disable DMP output when attitude sampling is no longer requested.
 *
 * Called by the gamepad A toggle. Previously published hub data remains available with
 * its old timestamp.
 * @note No arguments are required.
 * @retval 0 DMP stop request succeeded.
 * @retval 1 mpu_set_dmp_state(0) failed.
 */
uint8_t MPU6050_Stop(void);                                     /* 0=stopped successfully; 1=stop failed. */
/**
 * @brief Check gyro/accelerometer operation and apply measured biases to DMP.
 *
 * Called during DMP initialization. Only the gyro and accelerometer pass bits matter
 * because this board has no compass.
 * @note No arguments are required.
 * @retval 0 Tests passed and both bias writes succeeded.
 * @retval 1 A self-test or bias-write step failed.
 */
uint8_t run_self_test(void);                                   /* Run self-test and calibrate sensor biases. */
/**
 * @brief Read the stored test/calibration result for startup diagnostics.
 *
 * Called by print_boot_banner; no new test is performed.
 * @note No arguments are required.
 * @retval 0 Gyro and accelerometer passed and biases were written.
 * @retval 1 Test or bias write failed.
 * @retval 2 Test has not run.
 */
uint8_t MPU6050_GetSelfTest(void);                             /* 0=passed; 1=failed; 2=not attempted. */
/**
 * @brief Read the stored attitude-sensor startup result.
 *
 * Called by print_boot_banner; no I2C transaction is performed.
 * @note No arguments are required.
 * @retval 0 Initialization succeeded.
 * @retval 1 Identity check failed.
 * @retval 2 DMP setup failed.
 * @retval 3 Initialization has not run.
 */
uint8_t MPU6050_GetInitStatus(void);                           /* 0=success; 1=WHO_AM_I failed; 2=DMP setup failed; 3=not attempted. */

#endif /* __MPU6050_H__ */
