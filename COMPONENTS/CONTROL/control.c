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

/* ---- Sliding-window speed measurement (1 tick = 1 ms) ----
 * g_delta_hist : last N delta samples (circular), g_delta_idx : write index,
 * g_delta_sum  : running sum of the window (speed = sum * 1031 / N). */
static int16_t  g_delta_hist[4][PEO_CTRL_SPEED_WINDOW_TICKS];
static uint8_t  g_delta_idx[4];
static int32_t  g_delta_sum[4];

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
static volatile uint8_t g_mode = CONTROL_MODE_CLOSED;              /* 0=闭环(默认) 1=开环 */

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

        /* 滑动窗口测速：sum = 最近 N tick 的 delta 和（改善低速量化，见 control.h）。
           speed = sum × 1031 / N；量化步长 = 1031/N（N=10 → 1.03 cm/s）。 */
        g_delta_sum[m] += delta - g_delta_hist[m][g_delta_idx[m]];
        g_delta_hist[m][g_delta_idx[m]] = delta;
        g_delta_idx[m] = (uint8_t)((g_delta_idx[m] + 1) % PEO_CTRL_SPEED_WINDOW_TICKS);
        g_speed_d100[m] = g_delta_sum[m] * PEO_CTRL_CM_S_D100_PER_DELTA
                        / (int32_t)PEO_CTRL_SPEED_WINDOW_TICKS;

        /* 仅闭环模式运行速度 PI 并覆盖 PWM。开环模式下编码器照常测速
           （speed 仍回显真实转速），但 PI 不跑、PWM 保持开环命令设的值。 */
        if (g_mode == CONTROL_MODE_CLOSED)
        {
            /* ---- Speed PI controller (velocity / incremental form) ----
             *   e[n] = target - actual                  (0.01 cm/s)
             *   u[n] = u[n-1] + Kp*(e[n]-e[n-1]) + Ki*e[n]
             *
             * The velocity form accumulates the OUTPUT u directly, so clamping
             * u also clamps the integral term = built-in anti-windup (important
             * when a wheel is stalled against an obstacle).                    */
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
 * @brief  Reset all four wheel odometers to zero.
 * @note   Called from the main loop; the 1 kHz TIM6 ISR also writes
 *         g_odometer[], so this runs inside a brief critical section.
 */
void CONTROL_ResetOdometer(void)
{
    uint8_t  m;
    uint32_t primask = __disable_irq();      /* 临界区：与 1kHz 中断竞争保护 */
    for (m = 0; m < 4; m++)
        g_odometer[m] = 0;
    if (!primask)
        __enable_irq();
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
 * @brief  Set all four closed-loop target speeds atomically (0.01 cm/s).
 */
void CONTROL_SetTargetSpeedAll(const int32_t speed_d100[4])
{
    uint8_t  m;
    uint32_t primask = __disable_irq();      /* 临界区：四轮目标一次写完 */

    for (m = 0; m < 4; m++)
        g_target_d100[m] = speed_d100[m];

    if (!primask)
        __enable_irq();
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
    uint32_t primask = __disable_irq();

    g_pi_kp = kp;
    g_pi_ki = ki;

    if (!primask)
        __enable_irq();
}

/**
 * @brief  Stop the closed loop and coast ALL motors (emergency stop).
 */
void CONTROL_StopAll(void)
{
    uint8_t m;
    uint32_t primask = __disable_irq();

    for (m = 0; m < 4; m++)
    {
        g_target_d100[m] = 0;
        g_pi_u[m]        = 0.0f;
        g_pi_e_prev[m]   = 0.0f;
        MOTOR_Coast(m);
    }
    g_mode = CONTROL_MODE_CLOSED;   /* 急停回到闭环安全态 */

    if (!primask)
        __enable_irq();
}

void CONTROL_SetMode(uint8_t mode)
{
    uint8_t m;
    uint32_t primask;

    if (mode != CONTROL_MODE_OPEN && mode != CONTROL_MODE_CLOSED)
        return;

    if (g_mode == mode)
        return;   /* 同模式无操作 */

    primask = __disable_irq();
    g_mode = mode;

    /* 切模式复位 PI 积分，避免切回闭环瞬间 PWM 突变。 */
    for (m = 0; m < 4; m++)
    {
        g_pi_u[m]      = 0.0f;
        g_pi_e_prev[m] = 0.0f;
    }

    if (!primask)
        __enable_irq();
}

uint8_t CONTROL_GetMode(void)
{
    return g_mode;
}
