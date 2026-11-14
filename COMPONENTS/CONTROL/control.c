/**
 ******************************************************************************
 * @file    control.c
 * @brief   PEO Rover - CONTROL component: 1 kHz control loop on TIM6
 *
 * TIM6 update interrupt = the 1 kHz control loop scheduler. The ISR reads the
 * 4 encoders, accumulates odometry, computes wheel speeds, and runs a speed
 * PI controller per wheel that drives MOTOR_SetSpeed().
 ******************************************************************************
 */
#include "include/control.h"
#include "COMPONENTS/ENCODER/include/encoder.h"
#include "COMPONENTS/MOTOR/include/motor.h"
#include "COMPONENTS/SENSOR_HUB/include/sensor_hub.h"

/* ---- Loop state (ISR writes, app/debug reads) ---- */
static volatile uint32_t g_loop_tick     = 0;
static volatile int32_t  g_odometer[4]   = { 0, 0, 0, 0 };   /* accumulated count (32-bit) */
static volatile int32_t  g_speed_d100[4] = { 0, 0, 0, 0 };   /* speed, 0.01 cm/s */

/* ---- Speed PI state ----
 * g_target_d100 : target speed (0.01 cm/s), written by the app, read in ISR.
 * g_pi_u        : accumulated PI output (PWM per-mille), written in ISR,
 *                 read by CONTROL_GetPwm() for diagnostics.
 * g_pi_kp / ki  : gains, written by CONTROL_Init()/CONTROL_SetGains(),
 *                 read in ISR.
 * g_pi_e_prev   : previous error, ISR-only (no cross-context sharing).
 * ------------------------------------------------------------------------ */
static volatile int32_t g_target_d100[4] = { 0, 0, 0, 0 };
static volatile float   g_pi_u[4]        = { 0.0f, 0.0f, 0.0f, 0.0f };
static volatile float   g_pi_kp;                                   /* shared gains: all 4 motors */
static volatile float   g_pi_ki;
static          float   g_pi_e_prev[4]   = { 0.0f, 0.0f, 0.0f, 0.0f };

/* ----------------------------------------------------------------------------
   PUBLIC API
   ---------------------------------------------------------------------------- */

/**
 * @brief  Configure TIM6 for a 1 kHz update interrupt and start the loop.
 * @note   Initialises the PI gains to their defaults and clears all PI state.
 */
void CONTROL_Init(void)
{
    TIM_TimeBaseInitTypeDef tb;
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
        g_pi_u[m]        = 0.0f;
        g_pi_e_prev[m]   = 0.0f;
        g_target_d100[m] = 0;
    }
}

/**
 * @brief  TIM6 update interrupt handler = the 1 kHz control loop.
 * @note   Overrides the [WEAK] TIM6_DAC_IRQHandler in startup_stm32f40xx.s.
 *         Runs at NVIC preemption priority 3 (lowest) so the sensor
 *         data-ready INTs (MPU6050/VL53L1X) can still preempt it.
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
        int16_t delta = ENCODER_GetDelta(m);

        g_odometer[m]  += delta;                                        /* 32-bit odometry */
        g_speed_d100[m] = (int32_t)delta * PEO_CTRL_CM_S_D100_PER_DELTA; /* 0.01 cm/s      */

        /* ---- Speed PI controller (velocity / incremental form) ----
         *   e[n] = target - actual                  (0.01 cm/s)
         *   u[n] = u[n-1] + Kp*(e[n]-e[n-1]) + Ki*e[n]
         *
         * The velocity form accumulates the OUTPUT u directly, so clamping
         * u also clamps the integral term = built-in anti-windup (important
         * when a wheel is stalled against an obstacle).                    */
        {
            float e      = (float)(g_target_d100[m] - g_speed_d100[m]);
            float e_prev = g_pi_e_prev[m];

            g_pi_u[m] += g_pi_kp * (e - e_prev) + g_pi_ki * e;

            /* Clamp to the PWM range (anti-windup: also clamps the integral). */
            if (g_pi_u[m] > (float)MOTOR_SPEED_MAX)
                g_pi_u[m] = (float)MOTOR_SPEED_MAX;
            else if (g_pi_u[m] < -(float)MOTOR_SPEED_MAX)
                g_pi_u[m] = -(float)MOTOR_SPEED_MAX;

            g_pi_e_prev[m] = e;

            MOTOR_SetSpeed(m, (int16_t)g_pi_u[m]);
        }

        /* 推送到 sensor hub（1kHz 自动更新） */
        SENSOR_PushWheel(m, g_speed_d100[m], g_odometer[m]);
    }
}

/**
 * @brief  Latest wheel speed in 0.01 cm/s (negative = reverse).
 */
int32_t CONTROL_GetSpeedD100(uint8_t motor)
{
    if (motor > 3)
        return 0;
    return g_speed_d100[motor];
}

/**
 * @brief  Accumulated odometry count for one wheel.
 */
int32_t CONTROL_GetOdometer(uint8_t motor)
{
    if (motor > 3)
        return 0;
    return g_odometer[motor];
}

/**
 * @brief  Number of 1 kHz loop ticks since CONTROL_Init().
 */
uint32_t CONTROL_GetLoopTick(void)
{
    return g_loop_tick;
}

/**
 * @brief  Set the closed-loop target speed for one wheel (0.01 cm/s).
 */
void CONTROL_SetTargetSpeed(uint8_t motor, int32_t speed_d100)
{
    if (motor > 3)
        return;
    g_target_d100[motor] = speed_d100;
}

/**
 * @brief  Return the current target speed for one wheel (0.01 cm/s).
 */
int32_t CONTROL_GetTargetSpeed(uint8_t motor)
{
    if (motor > 3)
        return 0;
    return g_target_d100[motor];
}

/**
 * @brief  Return the PI output currently written to the motor (-1000..+1000).
 */
int16_t CONTROL_GetPwm(uint8_t motor)
{
    if (motor > 3)
        return 0;
    return (int16_t)g_pi_u[motor];
}

/**
 * @brief  Override the PI gains for ALL motors at runtime (for tuning).
 * @note   All four motors share a single set of PI gains.
 */
void CONTROL_SetGains(float kp, float ki)
{
    g_pi_kp = kp;
    g_pi_ki = ki;
}

/**
 * @brief  Stop the closed loop and coast ALL motors (emergency stop).
 */
void CONTROL_StopAll(void)
{
    uint8_t m;

    for (m = 0; m < 4; m++)
    {
        g_target_d100[m] = 0;
        g_pi_u[m]        = 0.0f;
        g_pi_e_prev[m]   = 0.0f;
        MOTOR_Coast(m);
    }
}
