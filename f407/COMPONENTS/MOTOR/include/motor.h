/**
 * @file  motor.h
 * @brief Expose signed PWM drive, braking, and coasting for each rover wheel.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * 8-channel PWM for 4x DRV8871 motor drivers (IN1/IN2 per motor).
 * Only the OUTPUT side is implemented here. The quadrature encoders
 * (TIM1/3/4/8, PEO_PINMAP.H section 4) are a separate concern and are
 * intentionally left untouched.
 *
 * PWM mapping (PEO_PINMAP.H section 5; all channels use 20 kHz):
 *   M1 -> TIM5  CH3/CH4 = PA2/PA3        (APB1; period counts 4200, ARR 4199)
 *   M2 -> TIM9  CH1/CH2 = PE5/PE6        (APB2; period counts 8400, ARR 8399)
 *   M3 -> TIM12 CH1/CH2 = PB14/PB15      (APB1; period counts 4200, ARR 4199)
 *   M4 -> TIM10 CH1(PB8) + TIM11 CH1(PB9) (APB2; period counts 8400, ARR 8399)
 *
 * DRV8871 direction logic (simplified module, IN1/IN2 only):
 *   IN1 = PWM, IN2 = 0    ->  forward
 *   IN1 = 0,   IN2 = PWM  ->  reverse
 *   IN1 = 0,   IN2 = 0    ->  coast
 *   IN1 = 1,   IN2 = 1    ->  brake (low side)
 *
 * Architecture rule (see PEO_PINMAP.H / rcc.h):
 *   - This component ONLY configures its own peripherals (GPIO + TIM).
 *   - RCC_Clock_Init() centrally enables TIM5/12 on APB1 and
 *     TIM9/10/11 on APB2; clocks are not enabled again here.
 *   - Edge-aligned PWM uses no timer interrupt, so no NVIC channel is
 *     configured here (NVIC_InitAll() in COMPONENTS/NVIC).
 */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx.h"
#include "COMPONENTS/PEO_PINMAP.H"

/* ============================================================================
   MOTOR INDICES (0-based)
   ============================================================================ */
#define MOTOR_M1        (0U)  /* Zero-based index for motor M1 (front left). */
#define MOTOR_M2        (1U)  /* Zero-based index for motor M2 (front right). */
#define MOTOR_M3        (2U)  /* Zero-based index for motor M3 (rear left). */
#define MOTOR_M4        (3U)  /* Zero-based index for motor M4 (rear right). */

/* SPEED SCALE: signed PWM duty in per-mille (parts per thousand).
   +1000 means full forward duty; -1000 means full reverse duty. This is
   a drive request, not a measured speed. Each channel scales its CCR
   compare value by its own period count (4200 on APB1, 8400 on APB2).
   At 100% duty CCR equals the period count, which is hardware ARR+1.
   Callers therefore do not need to know which bus supplies a motor timer. */
#define MOTOR_SPEED_MAX (1000)  /* Full PWM duty in per-mille: 1000 equals 100 percent, not a speed in cm/s. */

/* ============================================================================
   PUBLIC API
   ============================================================================ */

/**
 * @brief Prepare eight PWM outputs for four DRV8871 drivers and coast all wheels.
 *
 * Called by main before CONTROL starts. Configures GPIO routing and five timers at 20
 * kHz; clock enabling belongs to RCC.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note All channels start at 0% duty (all motors coast). Call once in main(), after
 *       RCC_Clock_Init()/NVIC_InitAll().
 */
void MOTOR_Init(void);

/**
 * @brief Apply a signed motor-drive duty request to one wheel.
 *
 * Called by TIM6 PI control, host direct-PWM commands, or the gamepad open-loop
 * fallback. The sign chooses direction; magnitude is clamped. This function does not
 * measure or regulate actual wheel speed.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right.
 * @param speed Signed duty in per-mille, normally -1000..1000; positive=forward,
 *              negative=reverse, zero=coast.
 * @retval None This function does not return a value.
 */
void MOTOR_SetSpeed(uint8_t motor, int16_t speed);

/**
 * @brief Request electrical braking by holding both driver inputs high.
 *
 * Available to application code; a later speed-controller write may replace this setting
 * if CONTROL remains active.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right. Invalid indices are ignored.
 * @retval None This function does not return a value.
 */
void MOTOR_Brake(uint8_t motor);

/**
 * @brief Release one motor by driving both bridge inputs low.
 *
 * Used at initialization and by CONTROL_StopAll. The wheel can slow naturally; this is
 * different from actively braking it.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right. Invalid indices are ignored.
 * @retval None This function does not return a value.
 */
void MOTOR_Coast(uint8_t motor);

#endif /* __MOTOR_H__ */
