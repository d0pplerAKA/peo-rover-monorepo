/**
 * @file  ina3221.h
 * @brief Expose startup status and periodic polling controls for motor-current sensing.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
#ifndef __INA3221_H__
#define __INA3221_H__

#include "stm32f4xx.h"

#include "COMPONENTS/PEO_PINMAP.H"

/* Shunt resistance in milliohms: onboard R100 = 0.1 ohm = 100 milliohms. */
#define PEO_PM_INA3221_RSHUNT_MOHM   (100U)  /* Shunt resistance in milliohms; converts measured voltage drop to current. */

/**
 * @brief Configure both current-monitor chips without starting periodic host reads.
 *
 * Called by main at startup. Initializes I2C2 and continuous conversions; the gamepad X
 * toggle later starts TIM7 polling through INA3221_PollInit.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void  INA3221_Init(void);
/**
 * @brief Start TIM7 so motor-current measurements are refreshed periodically.
 *
 * Called by the gamepad X toggle after INA3221_Init. Only timer-side configuration is
 * done here; RCC/NVIC are owned centrally.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void  INA3221_PollInit(void);                       /* Start TIM7 polling at SENSOR_RATE_INA_HZ. */
/**
 * @brief Stop periodic INA3221 reads by disabling TIM7.
 *
 * Called when the gamepad X toggle turns monitoring off. The chips remain configured and
 * the sensor hub retains its previous readings.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void  INA3221_Stop(void);                           /* Stop TIM7 polling. */
/**
 * @brief Read the saved startup presence check for one current-monitor chip.
 *
 * Used by print_boot_banner; this does not perform a new I2C transaction.
 * @param idx Device index 0=A or 1=B.
 * @retval 0 Expected manufacturer ID was found.
 * @retval 1 Identity check failed.
 * @retval 3 Initialization not attempted, or index is invalid.
 */
uint8_t INA3221_GetInitStatus(uint8_t idx);         /* 0=found; 1=not found; 3=initialization not attempted. */

#endif
