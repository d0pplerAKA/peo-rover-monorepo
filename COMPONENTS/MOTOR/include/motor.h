/**
 ******************************************************************************
 * @file    motor.h
 * @brief   PEO Rover - MOTOR component public API
 *
 * 8-channel PWM for 4x DRV8871 motor drivers (IN1/IN2 per motor).
 * Only the OUTPUT side is implemented here. The quadrature encoders
 * (TIM1/3/4/8, PEO_PINMAP.H section 4) are a separate concern and are
 * intentionally left untouched.
 *
 * PWM routing (PEO_PINMAP.H section 5):
 *   M1 -> TIM2  CH3/CH4 = PB10/PB11      (APB1, ARR 4200 @ 20 kHz)
 *   M2 -> TIM5  CH3/CH4 = PA2/PA3        (APB1, ARR 4200 @ 20 kHz)
 *   M3 -> TIM9  CH1/CH2 = PE5/PE6        (APB2, ARR 8400 @ 20 kHz)
 *   M4 -> TIM10 CH1(PB8) + TIM11 CH1(PB9) (APB2, ARR 8400 @ 20 kHz)
 *
 * DRV8871 direction logic (simplified module, IN1/IN2 only):
 *   IN1 = PWM, IN2 = 0    ->  forward
 *   IN1 = 0,   IN2 = PWM  ->  reverse
 *   IN1 = 0,   IN2 = 0    ->  coast
 *   IN1 = 1,   IN2 = 1    ->  brake (low side)
 *
 * Architecture rule (see PEO_PINMAP.H / rcc.h):
 *   - This component ONLY configures its own peripherals (GPIO + TIM).
 *   - Timer clocks are enabled by RCC_Clock_Init() (TIM2/5 on APB1,
 *     TIM9/10/11 on APB2) - never re-enabled here.
 *   - Edge-aligned PWM uses no timer interrupt, so no NVIC channel is
 *     configured here (NVIC_InitAll() in COMPONENTS/NVIC).
 ******************************************************************************
 */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx.h"
#include "PEO_PINMAP.H"

/* ============================================================================
   MOTOR INDICES (0-based)
   ============================================================================ */
#define MOTOR_M1        (0U)
#define MOTOR_M2        (1U)
#define MOTOR_M3        (2U)
#define MOTOR_M4        (3U)

/* ============================================================================
   SPEED SCALE
   ----------------------------------------------------------------------------
   Speeds are given in per-mille of the PWM period (+1000 = full forward,
   -1000 = full reverse). Internally each channel is scaled to its own
   timer's ARR (4200 on APB1, 8400 on APB2) so the caller never has to know
   which bus a motor's timer lives on.
   ============================================================================ */
#define MOTOR_SPEED_MAX (1000)

/* ============================================================================
   PUBLIC API
   ============================================================================ */

/**
 * @brief  Initialize all 8 PWM channels (GPIO AF + 5 timers).
 * @note   All channels start at 0% duty (all motors coast).
 *         Call once in main(), after RCC_Clock_Init()/NVIC_InitAll().
 */
void MOTOR_Init(void);

/**
 * @brief  Drive a motor at a signed speed.
 * @param  motor  MOTOR_M1..MOTOR_M4.
 * @param  speed  -MOTOR_SPEED_MAX .. +MOTOR_SPEED_MAX (per-mille).
 *                Positive = forward (IN1 PWM, IN2 0);
 *                negative = reverse (IN1 0, IN2 PWM); 0 = coast.
 *                Values beyond +/-MOTOR_SPEED_MAX are clamped.
 */
void MOTOR_SetSpeed(uint8_t motor, int16_t speed);

/**
 * @brief  Low-side brake: IN1 = IN2 = 100% (motor terminals shorted to GND).
 */
void MOTOR_Brake(uint8_t motor);

/**
 * @brief  Coast: IN1 = IN2 = 0 (motor terminals floating, free wheel).
 */
void MOTOR_Coast(uint8_t motor);

#endif /* __MOTOR_H__ */
