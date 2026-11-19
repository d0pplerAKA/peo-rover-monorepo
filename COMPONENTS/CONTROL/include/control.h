/**
 * @file  control.h
 * @brief Define wheel-speed control settings and the application interface.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
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
 * TIM6_DAC IRQ channel in NVIC_InitAll() (preemption priority 1, above
 * the sensor data-ready handlers that perform blocking I2C reads).
 */
#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "stm32f4xx.h"
#include "COMPONENTS/PEO_PINMAP.H"

/* ============================================================================
   CONTROL LOOP TIMEBASE (TIM6, APB1, 16-bit basic timer)
   ============================================================================ */
#define PEO_CTRL_LOOP_HZ            (1000U)  /* Control samples per second (Hz); 1000 Hz gives a nominal 1 ms step. */
#define PEO_CTRL_TIM                TIM6  /* Dedicated basic timer scheduling the wheel-speed loop. */

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
#define PEO_CTRL_CM_S_D100_PER_DELTA   (1031)  /* Speed scale: one encoder count per 1 ms corresponds to 1031 units of 0.01 cm/s. */

/* SPEED MEASUREMENT WINDOW: smooth low-speed encoder quantization.
   Speed = (sum of encoder deltas over the latest N ticks) * 1031 / N;
   each tick lasts 1 ms, and the stored speed unit is 0.01 cm/s.
   Resolution = 1031/N: N=5 gives 2.06 cm/s; N=10 gives 1.03 cm/s;
   N=20 gives 0.52 cm/s. A larger window gives finer resolution but adds
   about N/2 ms of delay, limiting how high the PI gains can be set.
   Default N=10: about 1.03 cm/s (2.3% of full speed), with about 5 ms delay.
   Adjust the window on the actual vehicle as needed. */
#define PEO_CTRL_SPEED_WINDOW_TICKS    (10U)  /* Number of 1 ms encoder samples averaged together; 10 ticks span 10 ms. */

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
 * @brief Start the dedicated 1 kHz wheel-speed controller on TIM6.
 *
 * Called by main after motor, encoder, and sensor setup. Loads shared PI gains and
 * clears targets and filter history. RCC and NVIC must already be configured.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note Call LAST in main(), after ENCODER_Init()/MOTOR_Init() etc. so the ISR reads a
 *       fully-initialized system. The TIM6 clock (RCC) and the TIM6_DAC IRQ channel
 *       (NVIC, priority 1) are already set up by RCC_Clock_Init()/NVIC_InitAll().
 *       Initialises the PI gains to PEO_CTRL_KP_DEFAULT / PEO_CTRL_KI_DEFAULT and clears
 *       all targets.
 */
void CONTROL_Init(void);

/**
 * @brief Read the latest filtered speed for one wheel.
 *
 * Available to application diagnostics; the TIM6 handler supplies the measurement.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right.
 * @retval value Signed speed in 0.01 cm/s; zero also returned for an invalid index.
 */
int32_t CONTROL_GetSpeedD100(uint8_t motor);

/**
 * @brief Read how many signed encoder counts a wheel has accumulated.
 *
 * Available to diagnostics; TIM6 adds counts and CONTROL_ResetOdometer clears them.
 * Divide by 1980 to obtain wheel revolutions.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right.
 * @retval value Accumulated encoder counts; zero for an invalid index.
 */
int32_t CONTROL_GetOdometer(uint8_t motor);
/**
 * @brief Set all four wheel distance counters back to zero.
 *
 * Called by the host odometer-reset command. Interrupts are briefly masked so TIM6
 * cannot add counts halfway through the reset.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void    CONTROL_ResetOdometer(void);

/**
 * @brief Read the control-loop execution count for timing diagnostics.
 *
 * Available to application diagnostics; each TIM6 update adds one. The counter starts at
 * zero at boot and wraps as a 32-bit unsigned value.
 * @note No arguments are required.
 * @retval value Number of control ticks; nominally one tick per millisecond.
 */
uint32_t CONTROL_GetLoopTick(void);

/**
 * @brief Choose the speed the closed-loop controller should aim for on one wheel.
 *
 * Called by the host closed-loop motor command. TIM6 later compares the target with
 * encoder speed; this setter does not itself switch the control mode.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right.
 * @param speed_d100 Signed target in 0.01 cm/s; positive=forward, negative=reverse.
 *                   Caller must enforce its allowed speed range.
 * @retval None This function does not return a value.
 */
void CONTROL_SetTargetSpeed(uint8_t motor, int32_t speed_d100);

/**
 * @brief Publish four wheel speed targets together.
 *
 * Called by GAMEPAD_Process. A short critical section prevents TIM6 from seeing a
 * mixture of old and new wheel targets.
 * @param speed_d100 Non-NULL array of four signed targets in 0.01 cm/s, ordered M1
 *                   through M4.
 * @retval None This function does not return a value.
 * @note Writes all four targets inside a critical section so the 1 kHz loop never sees a
 *       partially-updated four-wheel target.
 */
void CONTROL_SetTargetSpeedAll(const int32_t speed_d100[4]);

/**
 * @brief Read the requested closed-loop speed for one wheel.
 *
 * Available to diagnostics to compare the request with the encoder measurement.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right.
 * @retval value Signed target in 0.01 cm/s; zero for an invalid index.
 */
int32_t CONTROL_GetTargetSpeed(uint8_t motor);

/**
 * @brief Read the saved PI output for one wheel.
 *
 * Diagnostic getter for controller tuning. It reports PI state; in open-loop mode it
 * does not track direct MOTOR_SetSpeed writes.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right.
 * @retval value PI output truncated to signed PWM per-mille, -1000..1000; zero for an
 *               invalid index.
 * @note Diagnostic only - watch this while tuning to see how hard the loop is driving
 *       each wheel.
 */
int16_t CONTROL_GetPwm(uint8_t motor);

/**
 * @brief Replace the two PI tuning gains shared by all four wheels.
 *
 * Called by the host PID command after validating its values. TIM6 uses the new pair
 * together after this short critical section.
 * @param kp Proportional PWM change per unit change in speed error (error unit: 0.01
 *           cm/s).
 * @param ki PWM increment per unit speed error per 1 ms control tick; this already
 *           includes the discrete time-step convention.
 * @retval None This function does not return a value.
 * @note All four motors share a single set of PI gains.
 */
void CONTROL_SetGains(float kp, float ki);

/**
 * @brief Clear speed targets and PI memory, then coast all four motors.
 *
 * Called for STOP commands, ownership changes, or link/input loss. Selects closed-loop
 * mode; TIM6 remains enabled and can subsequently regulate the zero-speed targets.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note Clears every target to 0, resets the PI integrators, then coasts (motor
 *       terminals floating). Use for emergency stop.
 */
void CONTROL_StopAll(void);

/* CONTROL MODE: closed-loop speed control or open-loop direct PWM.
   Closed loop (default): PI follows a target speed (MOTOR_CLOSED / peo.drive()).
   Open loop: MOTOR / peo.motor() directly sets PWM, bypassing PI; this is
   a fallback while closed-loop controller gains are being tuned.
   Switching is implicit: MOTOR selects open loop; MOTOR_CLOSED selects closed
   loop; STOP_ALL calls CONTROL_StopAll() and returns to zero-target closed loop.
   Encoders keep measuring actual speed in open loop. Only PI and its
   MOTOR_SetSpeed() write are skipped. Switching resets PI memory so stale
   output is not reused when closed-loop operation resumes. */
#define CONTROL_MODE_CLOSED     (0U)   /* Closed-loop mode: PI adjusts PWM to follow speed targets (default). */
#define CONTROL_MODE_OPEN       (1U)   /* Open-loop mode: commands set PWM directly. */

/**
 * @brief Select direct PWM or speed feedback control and clear old PI memory.
 *
 * Called by host movement commands. Invalid modes and requests for the current mode are
 * ignored; existing speed targets are retained.
 * @param mode CONTROL_MODE_CLOSED for speed PI, or CONTROL_MODE_OPEN for direct PWM.
 * @retval None This function does not return a value.
 * @note Clearing PI memory avoids reusing stale accumulated output when returning to
 *       closed loop; new speed error can still produce a new PWM request.
 */
void CONTROL_SetMode(uint8_t mode);

/**
 * @brief Read whether direct PWM or speed feedback currently owns motor output.
 *
 * Used by telemetry and available to command/diagnostic code.
 * @note No arguments are required.
 * @retval value CONTROL_MODE_CLOSED (0) or CONTROL_MODE_OPEN (1).
 */
uint8_t CONTROL_GetMode(void);

#endif /* __CONTROL_H__ */
