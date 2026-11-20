/**
 * @file  motor.c
 * @brief Generate eight PWM signals for the four DRV8871 wheel drivers.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * Only the PWM output side is implemented. The encoders (TIM1/3/4/8) are
 * deliberately not touched here (see PEO_PINMAP.H section 4 vs section 5).
 */
#include "include/motor.h"

/* PWM CHANNEL TABLE: eight outputs = four motors with IN1 and IN2 each.
   Each entry stores the timer, channel number (1..4), and period count.
   APB1 timers use 4200 counts; APB2 timers use 8400 counts; both produce
   20 kHz PWM. Hardware ARR is one less than the period count. */
typedef struct {
    TIM_TypeDef *tim;       /* timer peripheral                 */
    uint8_t      channel;   /* compare channel 1..4             */
    uint16_t     arr;       /* Counts per PWM period; also the CCR compare value for 100% duty. */
} motor_pwm_ch_t;

static const motor_pwm_ch_t pwm_ch[8] = {  /* Eight timer-output descriptions, ordered IN1/IN2 for M1, then M2, M3, and M4. */
    /* M1:IN1/IN2 = TIM5 CH3/CH4 (PA2/PA3), APB1 */
    { TIM5,  3, PEO_PM_ARR_APB1 },
    { TIM5,  4, PEO_PM_ARR_APB1 },
    /* M2:IN1/IN2 = TIM9 CH1/CH2 (PE5/PE6), APB2 */
    { TIM9,  1, PEO_PM_ARR_APB2 },
    { TIM9,  2, PEO_PM_ARR_APB2 },
    /* M3:IN1/IN2 = TIM5 CH1/CH2 (PA0/PA1), APB1 */
    { TIM5,  1, PEO_PM_ARR_APB1 },
    { TIM5,  2, PEO_PM_ARR_APB1 },
    /* M4: IN1 = TIM10 CH1 (PB8), IN2 = TIM11 CH1 (PB9) */
    { TIM10, 1, PEO_PM_ARR_APB2 },
    { TIM11, 1, PEO_PM_ARR_APB2 },
};

/* Each motor's IN1/IN2 are two consecutive channels: IN1 = 2*m, IN2 = 2*m+1. */
#define MOTOR_IN1_CH(m) ((m) * 2U)  /* Map zero-based motor index m to its forward-drive PWM table entry. */
#define MOTOR_IN2_CH(m) ((m) * 2U + 1U)  /* Map zero-based motor index m to its reverse-drive PWM table entry. */

/* ----------------------------------------------------------------------------
   LOW-LEVEL HELPERS
   ---------------------------------------------------------------------------- */

/**
 * @brief Write one PWM timer compare register.
 *
 * Internal helper for motor output functions; compare values select how long the output
 * stays high within each PWM period.
 * @param ch PWM table index 0..7; caller must supply a valid index.
 * @param ccr Timer counts from zero (off) through the period count (fully on).
 * @retval None This function does not return a value.
 */
static void pwm_set_raw(uint8_t ch, uint16_t ccr)
{
    const motor_pwm_ch_t *c = &pwm_ch[ch];  /* Routing record for the selected PWM output. */

    switch (c->channel) {
        case 1: TIM_SetCompare1(c->tim, ccr); break;
        case 2: TIM_SetCompare2(c->tim, ccr); break;
        case 3: TIM_SetCompare3(c->tim, ccr); break;
        case 4: TIM_SetCompare4(c->tim, ccr); break;
        default: break;
    }
}

/**
 * @brief Translate a PWM duty request into timer counts.
 *
 * Internal helper used by MOTOR_SetSpeed; scales by the selected timer period so APB1
 * and APB2 channels have the same duty percentage.
 * @param ch PWM table index 0..7.
 * @param duty Duty in per-mille (parts per thousand), 0..MOTOR_SPEED_MAX.
 * @retval None This function does not return a value.
 */
static void pwm_set_duty(uint8_t ch, uint16_t duty)
{
    uint32_t ccr = ((uint32_t)duty * pwm_ch[ch].arr) / MOTOR_SPEED_MAX;  /* Duty converted to timer compare counts for this output. */
    pwm_set_raw(ch, (uint16_t)ccr);
}

/**
 * @brief Connect one motor-driver input pin to its timer output.
 *
 * Called by MOTOR_Init for all eight PWM pins. Alternate function means the timer,
 * rather than ordinary GPIO writes, drives the pin.
 * @param port GPIO peripheral that owns the pin, such as GPIOA.
 * @param pin GPIO_Pin_* bit mask identifying the pin.
 * @param src GPIO_PinSource* pin number used by the alternate-function selector.
 * @param af GPIO_AF_* routing choice for the intended timer.
 * @retval None This function does not return a value.
 */
static void gpio_af_init(GPIO_TypeDef *port, uint16_t pin,
                         uint8_t src, uint8_t af)
{
    GPIO_InitTypeDef gpio;  /* GPIO setup record: selects pin routing, electrical drive, pulls, and switching speed. */

    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_Pin   = pin;
    GPIO_Init(port, &gpio);
    GPIO_PinAFConfig(port, src, af);
}

/**
 * @brief Start a timer that defines the motor PWM repetition period.
 *
 * Called by MOTOR_Init for the four PWM timers. Uses an up counter with no prescaling
 * and enables automatic reload buffering.
 * @param tim Timer peripheral to configure.
 * @param arr Counts per PWM period; hardware auto-reload register receives arr-1.
 * @retval None This function does not return a value.
 * @note Timer clock is 84 MHz on APB1 or 168 MHz on APB2, producing 20 kHz PWM.
 */
static void timer_pwm_init(TIM_TypeDef *tim, uint16_t arr)
{
    TIM_TimeBaseInitTypeDef tb;  /* Timer setup record containing prescaler and period counts for the requested rate. */

    tb.TIM_Prescaler         = 0;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = arr - 1U;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(tim, &tb);

    TIM_ARRPreloadConfig(tim, ENABLE);
    TIM_Cmd(tim, ENABLE);
}

/**
 * @brief Configure one timer channel to produce a PWM waveform, initially off.
 *
 * Called by MOTOR_Init. PWM1 compares the counter with CCR; preloading applies compare
 * changes at an update boundary.
 * @param tim Timer peripheral owning this output.
 * @param channel Output-compare channel number 1..4.
 * @retval None This function does not return a value.
 */
static void oc_init(TIM_TypeDef *tim, uint8_t channel)
{
    TIM_OCInitTypeDef oc;  /* Timer output-compare setup describing PWM mode, polarity, and initial zero duty. */

    oc.TIM_OCMode       = TIM_OCMode_PWM1;
    oc.TIM_OutputState  = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;  /* general timers: unused */
    oc.TIM_Pulse        = 0;                         /* start at 0% duty       */
    oc.TIM_OCPolarity   = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity  = TIM_OCPolarity_High;       /* general timers: unused */
    oc.TIM_OCIdleState  = TIM_OCIdleState_Reset;     /* general timers: unused */
    oc.TIM_OCNIdleState = TIM_OCIdleState_Reset;     /* general timers: unused */

    switch (channel) {
        case 1: TIM_OC1Init(tim, &oc); TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable); break;
        case 2: TIM_OC2Init(tim, &oc); TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable); break;
        case 3: TIM_OC3Init(tim, &oc); TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable); break;
        case 4: TIM_OC4Init(tim, &oc); TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable); break;
        default: break;
    }
}

/* ----------------------------------------------------------------------------
   PUBLIC API
   ---------------------------------------------------------------------------- */

/**
 * @brief Prepare eight PWM outputs for four DRV8871 drivers and coast all wheels.
 *
 * Called by main before CONTROL starts. Configures GPIO routing and four timers at 20
 * kHz; clock enabling belongs to RCC.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void MOTOR_Init(void)
{
    /* GPIO: 8 PWM pins -> timer AF push-pull output. */
    gpio_af_init(PEO_PM_PWM1_PORT, PEO_PM_PWM1_PIN, PEO_PM_PWM1_SRC, PEO_PM_PWM1_2_AF);
    gpio_af_init(PEO_PM_PWM2_PORT, PEO_PM_PWM2_PIN, PEO_PM_PWM2_SRC, PEO_PM_PWM1_2_AF);
    gpio_af_init(PEO_PM_PWM3_PORT, PEO_PM_PWM3_PIN, PEO_PM_PWM3_SRC, PEO_PM_PWM3_4_AF);
    gpio_af_init(PEO_PM_PWM4_PORT, PEO_PM_PWM4_PIN, PEO_PM_PWM4_SRC, PEO_PM_PWM3_4_AF);
    gpio_af_init(PEO_PM_PWM5_PORT, PEO_PM_PWM5_PIN, PEO_PM_PWM5_SRC, PEO_PM_PWM5_6_AF);
    gpio_af_init(PEO_PM_PWM6_PORT, PEO_PM_PWM6_PIN, PEO_PM_PWM6_SRC, PEO_PM_PWM5_6_AF);
    gpio_af_init(PEO_PM_PWM7_PORT, PEO_PM_PWM7_PIN, PEO_PM_PWM7_SRC, PEO_PM_PWM7_8_AF);
    gpio_af_init(PEO_PM_PWM8_PORT, PEO_PM_PWM8_PIN, PEO_PM_PWM8_SRC, PEO_PM_PWM7_8_AF);

    /* Timers: time-base + OC channels. */
    timer_pwm_init(TIM5,  PEO_PM_ARR_APB1);  oc_init(TIM5,  1);  oc_init(TIM5,  2);  oc_init(TIM5,  3);  oc_init(TIM5,  4);
    timer_pwm_init(TIM9,  PEO_PM_ARR_APB2);  oc_init(TIM9,  1);  oc_init(TIM9,  2);
    timer_pwm_init(TIM10, PEO_PM_ARR_APB2);  oc_init(TIM10, 1);
    timer_pwm_init(TIM11, PEO_PM_ARR_APB2);  oc_init(TIM11, 1);

    /* Power-up state: all motors coast (IN1 = IN2 = 0). */
    MOTOR_Coast(MOTOR_M1);
    MOTOR_Coast(MOTOR_M2);
    MOTOR_Coast(MOTOR_M3);
    MOTOR_Coast(MOTOR_M4);
}

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
void MOTOR_SetSpeed(uint8_t motor, int16_t speed)
{
    uint8_t in1, in2; /* PWM table indices for the selected motor forward/reverse bridge inputs. */

    if (motor > MOTOR_M4)
        return;

    in1 = MOTOR_IN1_CH(motor);
    in2 = MOTOR_IN2_CH(motor);

    if (speed >= 0) {
        uint16_t duty = (speed > MOTOR_SPEED_MAX) ? MOTOR_SPEED_MAX : (uint16_t)speed;  /* Clamped nonnegative PWM duty in per-mille. */
        pwm_set_duty(in1, duty);   /* IN1 = PWM */
        pwm_set_raw(in2, 0);       /* IN2 = 0   */
    } else {
        int32_t  mag  = -(int32_t)speed;  /* Nonnegative magnitude of reverse PWM duty; 32-bit arithmetic avoids negating int16 minimum. */
        uint16_t duty = (mag > MOTOR_SPEED_MAX) ? MOTOR_SPEED_MAX : (uint16_t)mag;  /* Clamped nonnegative PWM duty in per-mille. */
        pwm_set_raw(in1, 0);       /* IN1 = 0   */
        pwm_set_duty(in2, duty);   /* IN2 = PWM */
    }
}

/**
 * @brief Request electrical braking by holding both driver inputs high.
 *
 * Available to application code; a later speed-controller write may replace this setting
 * if CONTROL remains active.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right. Invalid indices are ignored.
 * @retval None This function does not return a value.
 */
void MOTOR_Brake(uint8_t motor)
{
    if (motor > MOTOR_M4)
        return;

    pwm_set_raw(MOTOR_IN1_CH(motor), pwm_ch[MOTOR_IN1_CH(motor)].arr);
    pwm_set_raw(MOTOR_IN2_CH(motor), pwm_ch[MOTOR_IN2_CH(motor)].arr);
}

/**
 * @brief Release one motor by driving both bridge inputs low.
 *
 * Used at initialization and by CONTROL_StopAll. The wheel can slow naturally; this is
 * different from actively braking it.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right. Invalid indices are ignored.
 * @retval None This function does not return a value.
 */
void MOTOR_Coast(uint8_t motor)
{
    if (motor > MOTOR_M4)
        return;

    pwm_set_raw(MOTOR_IN1_CH(motor), 0);
    pwm_set_raw(MOTOR_IN2_CH(motor), 0);
}
