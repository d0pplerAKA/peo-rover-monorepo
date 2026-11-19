/**
 * @file  tof400c.h
 * @brief Define range-sensor profiles and controls without modifying the vendor API.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
#ifndef __TOF400C_H__
#define __TOF400C_H__


#ifdef __cplusplus
extern "C" {
#endif

#include "COMPONENTS/I2C/include/i2c.h"
#include "COMPONENTS/VL53L1X/core/VL53L1X_api.h"


#define TOF400C_FRONT           0  /* Array index for the forward-facing range sensor. */
#define TOF400C_LEFT            1  /* Array index for the left-facing range sensor. */
#define TOF400C_RIGHT           2  /* Array index for the right-facing range sensor. */

#define TOF400C_ON              1  /* Software request to keep this range sensor measuring. */
#define TOF400C_OFF             0  /* Software request to stop this range sensor after a processed result. */

/* VL53L1X rate profiles: distance mode (DM), timing budget (TB), and
   intermeasurement period (IMP). For one continuously ranging device, nominal
   rate = 1000/max(TB,IMP) Hz, with TB/IMP in ms and IMP >= TB.
   DM=1: Short, up to about 1.3 m with stronger ambient-light tolerance.
   DM=2: Long, up to about 4 m (the default distance mode).
   Allowed TB values listed in the original design notes:
   Short: 15/20/33/50/100/200/500 ms; Long: 33/50/100/200/500 ms.
   Maximum ranges assume darkness and a white/gray target. Bright ambient
   light greatly reduces range; the original notes cite about 73 cm in Long
   mode at 200 kcps/SPAD ambient light with TB=100 ms.

   Profile  DM      TB ms  IMP ms  Nominal rate  Approximate dark range
   FAST     Short     20      20      50 Hz      1.3 m (Short-mode limit)
   BAL      Long      33      33      30 Hz      2.0 m (minimum Long TB)
   STD      Long      50      75      13 Hz      2.7 m
   PREC     Long     100     100      10 Hz      3.6 m (cited measurement)
   MAX      Long     200     200       5 Hz      4.0 m (Long-mode limit)

   Original references: ST datasheet / UM2356; about 140 ms TB is needed
   for 4 m in Long mode. At TB=100 ms the cited ranges are 360 cm in darkness
   and 73 cm under 200 kcps bright light. Ranges for 20/33/50 ms are interpolated
   design estimates, not guaranteed performance. */

#define PEO_PM_VL53_PROFILE_FAST   0   /* Nominal 50 Hz profile for fast obstacle detection. */
#define PEO_PM_VL53_PROFILE_BAL    1   /* Nominal 30 Hz balanced profile. */
#define PEO_PM_VL53_PROFILE_STD    2   /* Nominal 13 Hz standard profile. */
#define PEO_PM_VL53_PROFILE_PREC   3   /* Nominal 10 Hz precision profile. */
#define PEO_PM_VL53_PROFILE_MAX    4   /* Nominal 5 Hz long-range profile. */

/* Select the active profile here: FAST, BAL, STD, PREC, or MAX. */
#define PEO_PM_VL53_PROFILE   PEO_PM_VL53_PROFILE_STD  /* Compile-time range profile; selects mode and both measurement timing values. */

#if (PEO_PM_VL53_PROFILE == PEO_PM_VL53_PROFILE_FAST)
    #define PEO_PM_VL53_DISTANCE_MODE        1   /* Short */
    #define PEO_PM_VL53_TIMING_BUDGET_MS     20  /* Time allocated to one range measurement, in milliseconds. */
    #define PEO_PM_VL53_INTER_MEAS_MS        20  /* Interval between measurement starts, in milliseconds; must be at least the budget. */
#elif (PEO_PM_VL53_PROFILE == PEO_PM_VL53_PROFILE_BAL)
    #define PEO_PM_VL53_DISTANCE_MODE        2   /* Long */
    #define PEO_PM_VL53_TIMING_BUDGET_MS     33  /* Time allocated to one range measurement, in milliseconds. */
    #define PEO_PM_VL53_INTER_MEAS_MS        33  /* Interval between measurement starts, in milliseconds; must be at least the budget. */
#elif (PEO_PM_VL53_PROFILE == PEO_PM_VL53_PROFILE_STD)
    #define PEO_PM_VL53_DISTANCE_MODE        2   /* Long */
    #define PEO_PM_VL53_TIMING_BUDGET_MS     50  /* Time allocated to one range measurement, in milliseconds. */
    #define PEO_PM_VL53_INTER_MEAS_MS        75  /* Interval between measurement starts, in milliseconds; must be at least the budget. */
#elif (PEO_PM_VL53_PROFILE == PEO_PM_VL53_PROFILE_PREC)
    #define PEO_PM_VL53_DISTANCE_MODE        2   /* Long */
    #define PEO_PM_VL53_TIMING_BUDGET_MS     100  /* Time allocated to one range measurement, in milliseconds. */
    #define PEO_PM_VL53_INTER_MEAS_MS        100  /* Interval between measurement starts, in milliseconds; must be at least the budget. */
#elif (PEO_PM_VL53_PROFILE == PEO_PM_VL53_PROFILE_MAX)
    #define PEO_PM_VL53_DISTANCE_MODE        2   /* Long */
    #define PEO_PM_VL53_TIMING_BUDGET_MS     200  /* Time allocated to one range measurement, in milliseconds. */
    #define PEO_PM_VL53_INTER_MEAS_MS        200  /* Interval between measurement starts, in milliseconds; must be at least the budget. */
#else
    #error "PEO_PM_VL53_PROFILE invalid: use FAST/BAL/STD/PREC/MAX"
#endif

extern VL53L1X_Result_t TOF400C_result[3];

/**
 * @brief Bring up three range sensors one at a time and give them separate bus
 *        addresses.
 *
 * Called by main. Prepares I2C3, XSHUT reset pins, and data-ready EXTI inputs. Sensors
 * start with a shared default address, so right, left, then front are released and
 * configured separately; failed devices are held in reset.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void TOF400C_Init(void);
/**
 * @brief Configure the single range sensor currently awake at the default address.
 *
 * Internal startup helper called by TOF400C_Init. Applies active-low interrupt polarity,
 * 4x4 ROI, threshold settings, selected timing profile, and the supplied address;
 * ranging is started separately.
 * @param assigned_addr Project address token from PEO_PM_VL53_*_ADDR, in the existing
 *                      shifted-address convention; this function passes assigned_addr<<1
 *                      to SetI2CAddress and uses assigned_addr for later access.
 * @retval 0 All setup steps succeeded.
 * @retval 1 Boot readiness timed out or failed.
 * @retval 2 Sensor initialization failed.
 * @retval 3 Interrupt polarity setup failed.
 * @retval 4 ROI setup failed.
 * @retval 5 Threshold setup failed.
 * @retval 6 Distance-mode setup failed.
 * @retval 7 Timing-budget setup failed.
 * @retval 8 Intermeasurement-period setup failed.
 * @retval 9 Address assignment failed.
 * @retval 10 Clearing the sensor interrupt failed.
 */
uint8_t TOF400C_BOOT_Init(uint8_t assigned_addr);   /* 0=success; nonzero identifies a failed initialization step. */
/**
 * @brief Request continuous ranging on one distance sensor.
 *
 * Called by the gamepad Y toggle. Sets the requested-running flag and starts the device
 * only when the flag was previously off. Readings arrive later through EXTI.
 * @param index TOF400C_FRONT (0), LEFT (1), or RIGHT (2); invalid indices are ignored.
 * @retval None This function does not return a value.
 */
void TOF400C_StartRanging_Interrupt(uint8_t index);
/**
 * @brief Read one range sensor startup result without accessing the bus.
 *
 * Called by print_boot_banner. TOF400C_Init stores success/failure separately for front,
 * left, and right.
 * @param index Sensor index 0=front, 1=left, 2=right.
 * @retval 0 Sensor initialization succeeded.
 * @retval 1 Sensor initialization failed.
 * @retval 3 Initialization has not run, or index is invalid.
 */
uint8_t TOF400C_GetInitStatus(uint8_t index);   /* 0=success; 1=failure; 3=not attempted or invalid index. */
/**
 * @brief Request that one range sensor stop after its next processed result.
 *
 * Called by the gamepad Y toggle. This only clears a software flag; the EXTI handler
 * issues the actual StopRanging call after reading a result.
 * @param index Valid sensor index 0..2 (front, left, right); caller must ensure it is in
 *              range.
 * @retval None This function does not return a value.
 */
void TOF400C_StopRanging_Interrupt(uint8_t index);


#ifdef __cplusplus
}
#endif


#endif
