/**
 ******************************************************************************
 * @file    control.h
 * @brief   PEO Rover - CONTROL component public API (1 kHz control loop)
 *
 * The control loop runs in a TIM6 update interrupt at 1 kHz. TIM6 is an APB1
 * BASIC timer (16-bit, time-base only - no output channels), which makes it
 * the natural scheduler for a periodic control task.
 *
 * Why TIM6 and not the SysTick (COMPONENTS/TIMER)?
 *   - SysTick (1 ms) is the generic timebase for get_tick()/mdelay() used by
 *     the motion driver - it is not a control-loop scheduler.
 *   - TIM6 is a dedicated peripheral with its own NVIC priority, so the
 *     control loop gets a clean, fixed 1 kHz cadence independent of SysTick.
 *
 * 1 kHz timebase (TIM6 clock = APB1 timer clock = 84 MHz):
 *   TIM6 is 16-bit, so a 1 kHz ARR does not fit directly (84000 > 65535).
 *   Prescale to 1 MHz first (PSC = 83), then ARR = 999 -> 1 kHz:
 *     84 MHz / (83+1) = 1 MHz;  1 MHz / (999+1) = 1 kHz.
 *
 * Loop body (TIM6_DAC_IRQHandler, this component):
 *   - read the 4 encoder deltas,
 *   - accumulate odometry (32-bit count),
 *   - compute each wheel speed in 0.01 cm/s,
 *   - run a speed PI controller per wheel -> MOTOR_SetSpeed().
 *
 * Speed scaling (1 kHz sample, delta = counts in 1 ms):
 *   cm/s = delta * 10.31, stored as 0.01 cm/s = delta * 1031.
 *
 * Speed PI controller (velocity / incremental form), one per wheel:
 *   e[n] = target - actual                       (0.01 cm/s)
 *   u[n] = u[n-1] + Kp*(e[n]-e[n-1]) + Ki*e[n]   (PWM per-mille)
 *   u[n] clamped to [-MOTOR_SPEED_MAX, +MOTOR_SPEED_MAX].
 *
 * Architecture rule: this component only configures TIM6 (peripheral-side
 * TIM_ITConfig). The TIM6 clock is enabled in RCC_Clock_Init() and the
 * TIM6_DAC IRQ channel in NVIC_InitAll() (preemption priority 3 - lowest,
 * so sensor data-ready INTs still preempt the loop).
 ******************************************************************************
 */
#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "stm32f4xx.h"
#include "COMPONENTS/PEO_PINMAP.H"

/* ============================================================================
   CONTROL LOOP TIMEBASE (TIM6, APB1, 16-bit basic timer)
   ============================================================================ */
#define PEO_CTRL_LOOP_HZ            (1000U)
#define PEO_CTRL_TIM                TIM6

/* TIM6 is 16-bit: 84 MHz can't fit a 1 kHz ARR directly (84000 > 65535).
   Prescale to 1 MHz (PSC=83), then ARR=999 -> 1 kHz. */
#define PEO_CTRL_TIM_PSC            (PEO_PM_TIM_APB1_HZ / 1000000UL - 1)  /* = 83  -> 1 MHz */
#define PEO_CTRL_TIM_ARR            (1000000UL / PEO_CTRL_LOOP_HZ - 1)    /* = 999 -> 1 kHz */

/* ============================================================================
   SPEED SCALING (1 kHz sample)
   ----------------------------------------------------------------------------
   cm/s = delta * 10.31. Stored in 0.01 cm/s units:
     speed_d100 = delta * 1031   (full speed ~ 44.24 cm/s -> 4424)
   ============================================================================ */
#define PEO_CTRL_CM_S_D100_PER_DELTA   (1031)

/* ============================================================================
   SPEED PI CONTROLLER GAINS
   ----------------------------------------------------------------------------
   Velocity-form (incremental) PI, run once per 1 kHz tick per motor:
       e[n] = target - actual                        (0.01 cm/s)
       u[n] = u[n-1] + Kp*(e[n]-e[n-1]) + Ki*e[n]
       u[n] clamped to [-MOTOR_SPEED_MAX, +MOTOR_SPEED_MAX]  (per-mille)

   Why the velocity form? It accumulates the OUTPUT u directly, so clamping
   u also clamps the integral term - built-in anti-windup, no separate
   integrator clamp needed (important when a wheel is stalled / blocked).

   Gain meaning (0.01 cm/s and per-mille PWM units):
       Kp = immediate PWM change per unit speed error. Raise until the wheel
            tracks the target briskly without hunting/oscillation.
       Ki = PWM accumulated per 1 kHz tick per unit speed error. Raise until
            the steady-state error vanishes without overshoot; too high
            oscillates.
   Full speed ~ 4424 (0.01 cm/s) maps to PWM 1000, so Kp ~ 0.1-0.5 and
   Ki ~ 0.001-0.01 are a sane starting band.

   NOTE: these are STARTING POINTS only - they MUST be tuned on the real
   vehicle (call CONTROL_SetGains() at runtime to sweep them).
   ============================================================================ */
#define PEO_CTRL_KP_DEFAULT         (0.25f)   /* proportional gain (starting point) */
#define PEO_CTRL_KI_DEFAULT         (0.005f)  /* integral gain   (starting point) */

/* ============================================================================
   PUBLIC API
   ============================================================================ */

/**
 * @brief  Configure TIM6 for a 1 kHz update interrupt and start the loop.
 * @note   Call LAST in main(), after ENCODER_Init()/MOTOR_Init() etc. so the
 *         ISR reads a fully-initialized system. The TIM6 clock (RCC) and the
 *         TIM6_DAC IRQ channel (NVIC, priority 3) are already set up by
 *         RCC_Clock_Init()/NVIC_InitAll(). Initialises the PI gains to
 *         PEO_CTRL_KP_DEFAULT / PEO_CTRL_KI_DEFAULT and clears all targets.
 */
void CONTROL_Init(void);

/**
 * @brief  Return the latest wheel speed in 0.01 cm/s (negative = reverse).
 * @param  motor  0..3 (M1..M4).
 */
int32_t CONTROL_GetSpeedD100(uint8_t motor);

/**
 * @brief  Return the accumulated odometry count for one wheel.
 * @param  motor  0..3 (M1..M4).
 */
int32_t CONTROL_GetOdometer(uint8_t motor);
void    CONTROL_ResetOdometer(void);

/**
 * @brief  Return the number of 1 kHz loop ticks since CONTROL_Init().
 *         (Diagnostic: confirms the loop is actually running at 1 kHz.)
 */
uint32_t CONTROL_GetLoopTick(void);

/**
 * @brief  Set the closed-loop target speed for one wheel.
 * @param  motor      0..3 (M1..M4).
 * @param  speed_d100 Target speed in 0.01 cm/s (positive = forward, negative
 *                    = reverse). The speed PI loop drives the motor toward
 *                    this value every 1 kHz tick.
 */
void CONTROL_SetTargetSpeed(uint8_t motor, int32_t speed_d100);

/**
 * @brief  Return the current target speed for one wheel (0.01 cm/s).
 * @param  motor  0..3 (M1..M4).
 */
int32_t CONTROL_GetTargetSpeed(uint8_t motor);

/**
 * @brief  Return the PI output currently written to the motor (-1000..+1000).
 * @param  motor  0..3 (M1..M4).
 * @note   Diagnostic only - watch this while tuning to see how hard the loop
 *         is driving each wheel.
 */
int16_t CONTROL_GetPwm(uint8_t motor);

/**
 * @brief  Override the PI gains for ALL motors at runtime (for tuning).
 * @note   All four motors share a single set of PI gains.
 * @param  kp     Proportional gain (replaces PEO_CTRL_KP_DEFAULT).
 * @param  ki     Integral gain   (replaces PEO_CTRL_KI_DEFAULT).
 */
void CONTROL_SetGains(float kp, float ki);

/**
 * @brief  Stop the closed loop and coast ALL motors.
 * @note   Clears every target to 0, resets the PI integrators, then coasts
 *         (motor terminals floating). Use for emergency stop.
 */
void CONTROL_StopAll(void);

#endif /* __CONTROL_H__ */
