/**
 ******************************************************************************
 * @file    motor.c
 * @brief   PEO Rover - MOTOR component: 8-channel DRV8871 PWM output
 *
 * Only the PWM output side is implemented. The encoders (TIM1/3/4/8) are
 * deliberately not touched here (see PEO_PINMAP.H section 4 vs section 5).
 ******************************************************************************
 */
#include "include/motor.h"

/* ----------------------------------------------------------------------------
   PWM CHANNEL TABLE
   ----------------------------------------------------------------------------
   8 channels = 4 motors x (IN1, IN2). Each entry names the timer, the
   compare channel (1..4) and the timer's ARR (period) - which depends on the
   APB bus the timer sits on (4200 on APB1, 8400 on APB2, both = 20 kHz).
   ---------------------------------------------------------------------------- */
typedef struct {
    TIM_TypeDef *tim;       /* timer peripheral                 */
    uint8_t      channel;   /* compare channel 1..4             */
    uint16_t     arr;       /* auto-reload value (= period)     */
} motor_pwm_ch_t;

static const motor_pwm_ch_t pwm_ch[8] = {
    /* M1: IN1/IN2 = TIM2 CH3/CH4 (PB10/PB11) */
    { TIM2,  3, PEO_PM_ARR_APB1 },
    { TIM2,  4, PEO_PM_ARR_APB1 },
    /* M2: IN1/IN2 = TIM5 CH3/CH4 (PA2/PA3) */
    { TIM5,  3, PEO_PM_ARR_APB1 },
    { TIM5,  4, PEO_PM_ARR_APB1 },
    /* M3: IN1/IN2 = TIM9 CH1/CH2 (PE5/PE6) */
    { TIM9,  1, PEO_PM_ARR_APB2 },
    { TIM9,  2, PEO_PM_ARR_APB2 },
    /* M4: IN1 = TIM10 CH1 (PB8), IN2 = TIM11 CH1 (PB9) */
    { TIM10, 1, PEO_PM_ARR_APB2 },
    { TIM11, 1, PEO_PM_ARR_APB2 },
};

/* Each motor's IN1/IN2 are two consecutive channels: IN1 = 2*m, IN2 = 2*m+1. */
#define MOTOR_IN1_CH(m) ((m) * 2U)
#define MOTOR_IN2_CH(m) ((m) * 2U + 1U)

/* ----------------------------------------------------------------------------
   LOW-LEVEL HELPERS
   ---------------------------------------------------------------------------- */

/**
 * @brief  Write a raw CCR value to one PWM channel.
 * @param  ch   Channel index 0..7.
 * @param  ccr  Compare value (0 .. ARR).
 */
static void pwm_set_raw(uint8_t ch, uint16_t ccr)
{
    const motor_pwm_ch_t *c = &pwm_ch[ch];

    switch (c->channel) {
        case 1: TIM_SetCompare1(c->tim, ccr); break;
        case 2: TIM_SetCompare2(c->tim, ccr); break;
        case 3: TIM_SetCompare3(c->tim, ccr); break;
        case 4: TIM_SetCompare4(c->tim, ccr); break;
        default: break;
    }
}

/**
 * @brief  Write a duty cycle to one PWM channel.
 * @param  ch    Channel index 0..7.
 * @param  duty  0..MOTOR_SPEED_MAX (per-mille). CCR = duty * ARR / 1000.
 */
static void pwm_set_duty(uint8_t ch, uint16_t duty)
{
    uint32_t ccr = ((uint32_t)duty * pwm_ch[ch].arr) / MOTOR_SPEED_MAX;
    pwm_set_raw(ch, (uint16_t)ccr);
}

/**
 * @brief  Configure one GPIO pin as a timer AF push-pull output.
 */
static void gpio_af_init(GPIO_TypeDef *port, uint16_t pin,
                         uint8_t src, uint8_t af)
{
    GPIO_InitTypeDef gpio;

    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_Pin   = pin;
    GPIO_Init(port, &gpio);
    GPIO_PinAFConfig(port, src, af);
}

/**
 * @brief  Configure one timer's time-base (edge-aligned up-counting) and
 *         enable it. Prescaler = 0, so the timer counts at its bus clock
 *         (84 MHz APB1 / 168 MHz APB2) and ARR sets the 20 kHz period.
 */
static void timer_pwm_init(TIM_TypeDef *tim, uint16_t arr)
{
    TIM_TimeBaseInitTypeDef tb;

    tb.TIM_Prescaler         = 0;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = arr;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(tim, &tb);

    TIM_ARRPreloadConfig(tim, ENABLE);
    TIM_Cmd(tim, ENABLE);
}

/**
 * @brief  Configure one timer's output-compare channel as PWM1 (edge-aligned),
 *         0% duty, active-high, with compare preload enabled.
 */
static void oc_init(TIM_TypeDef *tim, uint8_t channel)
{
    TIM_OCInitTypeDef oc;

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
 * @brief  Initialize all 8 PWM channels (GPIO AF + 5 timers), all coast.
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
    timer_pwm_init(TIM2,  PEO_PM_ARR_APB1);  oc_init(TIM2,  3);  oc_init(TIM2,  4);
    timer_pwm_init(TIM5,  PEO_PM_ARR_APB1);  oc_init(TIM5,  3);  oc_init(TIM5,  4);
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
 * @brief  Drive a motor at a signed speed (see motor.h for the sign mapping).
 */
void MOTOR_SetSpeed(uint8_t motor, int16_t speed)
{
    uint8_t in1, in2;

    if (motor > MOTOR_M4)
        return;

    in1 = MOTOR_IN1_CH(motor);
    in2 = MOTOR_IN2_CH(motor);

    if (speed >= 0) {
        uint16_t duty = (speed > MOTOR_SPEED_MAX) ? MOTOR_SPEED_MAX : (uint16_t)speed;
        pwm_set_duty(in1, duty);   /* IN1 = PWM */
        pwm_set_raw(in2, 0);       /* IN2 = 0   */
    } else {
        int32_t  mag  = -(int32_t)speed;
        uint16_t duty = (mag > MOTOR_SPEED_MAX) ? MOTOR_SPEED_MAX : (uint16_t)mag;
        pwm_set_raw(in1, 0);       /* IN1 = 0   */
        pwm_set_duty(in2, duty);   /* IN2 = PWM */
    }
}

/**
 * @brief  Low-side brake: IN1 = IN2 = 100%.
 */
void MOTOR_Brake(uint8_t motor)
{
    if (motor > MOTOR_M4)
        return;

    pwm_set_raw(MOTOR_IN1_CH(motor), pwm_ch[MOTOR_IN1_CH(motor)].arr);
    pwm_set_raw(MOTOR_IN2_CH(motor), pwm_ch[MOTOR_IN2_CH(motor)].arr);
}

/**
 * @brief  Coast: IN1 = IN2 = 0.
 */
void MOTOR_Coast(uint8_t motor)
{
    if (motor > MOTOR_M4)
        return;

    pwm_set_raw(MOTOR_IN1_CH(motor), 0);
    pwm_set_raw(MOTOR_IN2_CH(motor), 0);
}
