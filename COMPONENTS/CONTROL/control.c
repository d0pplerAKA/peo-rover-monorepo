/**
 * @file  control.c
 * @brief Measure wheel speeds and run the four incremental PI controllers on TIM6.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * TIM6 update interrupt = the 1 kHz control loop scheduler. The ISR reads the
 * 4 encoders, accumulates odometry, computes wheel speeds, and runs a speed
 * PI controller per wheel that drives MOTOR_SetSpeed().
 */
#include "include/control.h"
#include "COMPONENTS/ENCODER/include/encoder.h"
#include "COMPONENTS/MOTOR/include/motor.h"
#include "COMPONENTS/SENSOR_HUB/include/sensor_hub.h"

/* TIM6 interrupts run independently of the main loop. volatile asks the compiler
 * to actually read/write shared state each time; it does not make a whole array
 * update indivisible. Multi-wheel writes below therefore briefly mask interrupts.
 * Wheel order throughout: M1 front left, M2 front right, M3 rear left, M4 rear right.
 */
/* ---- Loop state (ISR writes, app/debug reads) ---- */
static volatile uint32_t g_loop_tick     = 0;  /* Completed TIM6 control iterations since boot; nominally one per ms. */
static volatile int32_t  g_odometer[4]   = { 0, 0, 0, 0 };   /* accumulated count (32-bit) */
static volatile int32_t  g_speed_d100[4] = { 0, 0, 0, 0 };   /* speed, 0.01 cm/s */

/* Low-speed encoders can report zero counts on one tick and one count on the
 * next even at steady motion. Keeping the last N samples reduces this jumpiness.
 * Replacing the oldest sample and adjusting a running sum avoids adding all N
 * samples again on every interrupt. The trade-off is extra measurement delay.
 */
/* ---- Sliding-window speed measurement (1 tick = 1 ms) ----
 * g_delta_hist : last N delta samples (circular), g_delta_idx : write index,
 * g_delta_sum  : running sum of the window (speed = sum * 1031 / N). */
/* Per-wheel ring buffers of encoder-count changes from the last N control ticks. */
static int16_t  g_delta_hist[4][PEO_CTRL_SPEED_WINDOW_TICKS];
static uint8_t  g_delta_idx[4];  /* Per-wheel next history slot to replace, from 0 to N-1. */
static int32_t  g_delta_sum[4];  /* Per-wheel total encoder counts within the current speed-measurement window. */

/* ---- Speed PI state ----
 * g_target_d100 : target speed (0.01 cm/s), written by the app, read in ISR.
 * g_pi_u        : accumulated PI output (PWM per-mille), written in ISR,
 *                 read by CONTROL_GetPwm() for diagnostics.
 * g_pi_kp / ki  : gains, written by CONTROL_Init()/CONTROL_SetGains(),
 *                 read in ISR.
 * g_pi_e_prev   : previous error, ISR-only (no cross-context sharing).
 * ------------------------------------------------------------------------ */
static volatile int32_t g_target_d100[4] = { 0, 0, 0, 0 };  /* Requested wheel speeds in 0.01 cm/s, written by commands and read by TIM6. */
static volatile float   g_pi_u[4]        = { 0.0f, 0.0f, 0.0f, 0.0f };  /* Accumulated PI output for each wheel, in signed PWM per-mille. */
static volatile float   g_pi_kp;                                   /* shared gains: all 4 motors */
static volatile float   g_pi_ki;  /* Shared integral gain: PWM per-mille per speed-error unit per 1 ms tick. */
/* Previous speed errors in 0.01 cm/s, used to calculate the proportional change. */
static          float   g_pi_e_prev[4]   = { 0.0f, 0.0f, 0.0f, 0.0f };
static volatile uint8_t g_mode = CONTROL_MODE_CLOSED;              /* 0 = closed-loop speed control (default); 1 = direct, open-loop PWM. */

/* ----------------------------------------------------------------------------
   PUBLIC API
   ---------------------------------------------------------------------------- */

/**
 * @brief Start the dedicated 1 kHz wheel-speed controller on TIM6.
 *
 * Called by main after motor, encoder, and sensor setup. Loads shared PI gains and
 * clears targets and filter history. RCC and NVIC must already be configured.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note Initialises the PI gains to their defaults and clears all PI state.
 */
void CONTROL_Init(void)
{
    TIM_TimeBaseInitTypeDef tb;  /* Timer setup record containing prescaler and period counts for the requested rate. */
    uint8_t m;

    tb.TIM_Prescaler         = PEO_CTRL_TIM_PSC;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = PEO_CTRL_TIM_ARR;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(PEO_CTRL_TIM, &tb);

    TIM_ClearITPendingBit(PEO_CTRL_TIM, TIM_IT_Update);
    TIM_ITConfig(PEO_CTRL_TIM, TIM_IT_Update, ENABLE);
    TIM_Cmd(PEO_CTRL_TIM, ENABLE);

    /* Load default gains (shared by all four motors) and clear PI state. */
    g_pi_kp = PEO_CTRL_KP_DEFAULT;
    g_pi_ki = PEO_CTRL_KI_DEFAULT;

    for (m = 0; m < 4; m++)
    {
        uint8_t i;
        g_pi_u[m]        = 0.0f;
        g_pi_e_prev[m]   = 0.0f;
        g_target_d100[m] = 0;
        g_delta_idx[m]   = 0;
        g_delta_sum[m]   = 0;
        for (i = 0; i < PEO_CTRL_SPEED_WINDOW_TICKS; i++)
            g_delta_hist[m][i] = 0;
    }
    g_mode = CONTROL_MODE_CLOSED;
}

/**
 * @brief Measure wheel movement and adjust motor PWM once per control tick.
 *
 * The processor calls this vector on TIM6 updates. It sums encoder changes over the
 * sliding window, updates odometry, runs incremental PI only in closed-loop mode, and
 * publishes wheel data. NVIC_InitAll assigns preemption priority 1, above sensor I2C
 * handlers.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note Overrides the [WEAK] TIM6_DAC_IRQHandler in startup_stm32f40xx.s. Runs at NVIC
 *       preemption priority 1, above sensor data-ready handlers, so blocking sensor I2C
 *       reads do not delay wheel sampling.
 */
void TIM6_DAC_IRQHandler(void)
{
    uint8_t m;

    if (TIM_GetITStatus(PEO_CTRL_TIM, TIM_IT_Update) == RESET)
        return;

    TIM_ClearITPendingBit(PEO_CTRL_TIM, TIM_IT_Update);
    g_loop_tick++;

    for (m = 0; m < 4; m++)
    {
        int16_t delta = ENCODER_GetDelta(m);  /* Signed encoder counts accumulated since this wheel was read on the previous tick. */

        g_odometer[m]  += delta;                                        /* 32-bit odometry */

        /* Sliding-window speed: sum the encoder changes from the latest N ticks
           to reduce coarse low-speed steps (see control.h). Speed = sum * 1031 / N
           in 0.01 cm/s; one-count resolution is 1031/N (N=10: about 1.03 cm/s). */
        g_delta_sum[m] += delta - g_delta_hist[m][g_delta_idx[m]];
        g_delta_hist[m][g_delta_idx[m]] = delta;
        g_delta_idx[m] = (uint8_t)((g_delta_idx[m] + 1) % PEO_CTRL_SPEED_WINDOW_TICKS);
        g_speed_d100[m] = g_delta_sum[m] * PEO_CTRL_CM_S_D100_PER_DELTA
                        / (int32_t)PEO_CTRL_SPEED_WINDOW_TICKS;

        /* Only closed-loop mode runs PI and replaces PWM. In open-loop mode,
           encoders still report actual speed, but PI is skipped and the PWM
           remains at the value requested by the direct motor command. */
        if (g_mode == CONTROL_MODE_CLOSED)
        {
            /* ---- Speed PI controller (velocity / incremental form) ----
             *   e[n] = target - actual                  (0.01 cm/s)
             *   u[n] = u[n-1] + Kp*(e[n]-e[n-1]) + Ki*e[n]
             *
             * The velocity form accumulates the OUTPUT u directly, so clamping
             * u also clamps the integral term = built-in anti-windup (important
             * when a wheel is stalled against an obstacle).                    */
            float e      = (float)(g_target_d100[m] - g_speed_d100[m]);  /* Current target-minus-measured speed error in 0.01 cm/s. */
            float e_prev = g_pi_e_prev[m];  /* Speed error saved from the previous PI step, in 0.01 cm/s. */

            g_pi_u[m] += g_pi_kp * (e - e_prev) + g_pi_ki * e;

            /* Clamp to the PWM range (anti-windup: also clamps the integral). */
            if (g_pi_u[m] > (float)MOTOR_SPEED_MAX)
                g_pi_u[m] = (float)MOTOR_SPEED_MAX;
            else if (g_pi_u[m] < -(float)MOTOR_SPEED_MAX)
                g_pi_u[m] = -(float)MOTOR_SPEED_MAX;

            g_pi_e_prev[m] = e;

            MOTOR_SetSpeed(m, (int16_t)g_pi_u[m]);
        }

        /* Publish wheel measurements to the sensor hub on every 1 kHz tick. */
        SENSOR_PushWheel(m, g_speed_d100[m], g_odometer[m]);
    }
}

/**
 * @brief Read the latest filtered speed for one wheel.
 *
 * Available to application diagnostics; the TIM6 handler supplies the measurement.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right.
 * @retval value Signed speed in 0.01 cm/s; zero also returned for an invalid index.
 */
int32_t CONTROL_GetSpeedD100(uint8_t motor)
{
    if (motor > 3)
        return 0;
    return g_speed_d100[motor];
}

/**
 * @brief Read how many signed encoder counts a wheel has accumulated.
 *
 * Available to diagnostics; TIM6 adds counts and CONTROL_ResetOdometer clears them.
 * Divide by 1980 to obtain wheel revolutions.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right.
 * @retval value Accumulated encoder counts; zero for an invalid index.
 */
int32_t CONTROL_GetOdometer(uint8_t motor)
{
    if (motor > 3)
        return 0;
    return g_odometer[motor];
}

/**
 * @brief Set all four wheel distance counters back to zero.
 *
 * Called by the host odometer-reset command. Interrupts are briefly masked so TIM6
 * cannot add counts halfway through the reset.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note Called from the main loop; the 1 kHz TIM6 ISR also writes g_odometer[], so this
 *       runs inside a brief critical section.
 */
void CONTROL_ResetOdometer(void)
{
    uint8_t  m;
    /* Critical section: prevent the 1 kHz interrupt from changing these values during the reset. */
    uint32_t primask = __disable_irq();
    for (m = 0; m < 4; m++)
        g_odometer[m] = 0;
    if (!primask)
        __enable_irq();
}

/**
 * @brief Read the control-loop execution count for timing diagnostics.
 *
 * Available to application diagnostics; each TIM6 update adds one. The counter starts at
 * zero at boot and wraps as a 32-bit unsigned value.
 * @note No arguments are required.
 * @retval value Number of control ticks; nominally one tick per millisecond.
 */
uint32_t CONTROL_GetLoopTick(void)
{
    return g_loop_tick;
}

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
void CONTROL_SetTargetSpeed(uint8_t motor, int32_t speed_d100)
{
    if (motor > 3)
        return;
    g_target_d100[motor] = speed_d100;
}

/**
 * @brief Publish four wheel speed targets together.
 *
 * Called by GAMEPAD_Process. A short critical section prevents TIM6 from seeing a
 * mixture of old and new wheel targets.
 * @param speed_d100 Non-NULL array of four signed targets in 0.01 cm/s, ordered M1
 *                   through M4.
 * @retval None This function does not return a value.
 */
void CONTROL_SetTargetSpeedAll(const int32_t speed_d100[4])
{
    uint8_t  m;
    uint32_t primask = __disable_irq();      /* Critical section: publish all four wheel targets together. */

    for (m = 0; m < 4; m++)
        g_target_d100[m] = speed_d100[m];

    if (!primask)
        __enable_irq();
}

/**
 * @brief Read the requested closed-loop speed for one wheel.
 *
 * Available to diagnostics to compare the request with the encoder measurement.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right.
 * @retval value Signed target in 0.01 cm/s; zero for an invalid index.
 */
int32_t CONTROL_GetTargetSpeed(uint8_t motor)
{
    if (motor > 3)
        return 0;
    return g_target_d100[motor];
}

/**
 * @brief Read the saved PI output for one wheel.
 *
 * Diagnostic getter for controller tuning. It reports PI state; in open-loop mode it
 * does not track direct MOTOR_SetSpeed writes.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right.
 * @retval value PI output truncated to signed PWM per-mille, -1000..1000; zero for an
 *               invalid index.
 */
int16_t CONTROL_GetPwm(uint8_t motor)
{
    if (motor > 3)
        return 0;
    return (int16_t)g_pi_u[motor];
}

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
void CONTROL_SetGains(float kp, float ki)
{
    /* Saved interrupt-mask state; restore it so callers that already masked interrupts stay protected. */
    uint32_t primask = __disable_irq();

    g_pi_kp = kp;
    g_pi_ki = ki;

    if (!primask)
        __enable_irq();
}

/**
 * @brief Clear speed targets and PI memory, then coast all four motors.
 *
 * Called for STOP commands, ownership changes, or link/input loss. Selects closed-loop
 * mode; TIM6 remains enabled and can subsequently regulate the zero-speed targets.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void CONTROL_StopAll(void)
{
    uint8_t m;
    /* Saved interrupt-mask state; restore it so callers that already masked interrupts stay protected. */
    uint32_t primask = __disable_irq();

    for (m = 0; m < 4; m++)
    {
        g_target_d100[m] = 0;
        g_pi_u[m]        = 0.0f;
        g_pi_e_prev[m]   = 0.0f;
        MOTOR_Coast(m);
    }
    g_mode = CONTROL_MODE_CLOSED;   /* An emergency stop returns to closed-loop mode with zero targets. */

    if (!primask)
        __enable_irq();
}

/**
 * @brief Select direct PWM or speed feedback control and clear old PI memory.
 *
 * Called by host movement commands. Invalid modes and requests for the current mode are
 * ignored; existing speed targets are retained.
 * @param mode CONTROL_MODE_CLOSED for speed PI, or CONTROL_MODE_OPEN for direct PWM.
 * @retval None This function does not return a value.
 */
void CONTROL_SetMode(uint8_t mode)
{
    uint8_t m;
    uint32_t primask;  /* Saved interrupt-mask state; restore it so callers that already masked interrupts stay protected. */

    if (mode != CONTROL_MODE_OPEN && mode != CONTROL_MODE_CLOSED)
        return;

    if (g_mode == mode)
        return;   /* Already in this mode; no change is needed. */

    primask = __disable_irq();
    g_mode = mode;

    /* Reset PI memory when changing mode, so old accumulated output does not
       cause an abrupt PWM change when closed-loop control resumes. */
    for (m = 0; m < 4; m++)
    {
        g_pi_u[m]      = 0.0f;
        g_pi_e_prev[m] = 0.0f;
    }

    if (!primask)
        __enable_irq();
}

/**
 * @brief Read whether direct PWM or speed feedback currently owns motor output.
 *
 * Used by telemetry and available to command/diagnostic code.
 * @note No arguments are required.
 * @retval value CONTROL_MODE_CLOSED (0) or CONTROL_MODE_OPEN (1).
 */
uint8_t CONTROL_GetMode(void)
{
    return g_mode;
}
