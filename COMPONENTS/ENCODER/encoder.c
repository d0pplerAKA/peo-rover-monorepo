/**
 * @file  encoder.c
 * @brief Read four quadrature encoders with hardware timers for wheel feedback.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * Encoder interface mode TI12 (both edges = x4). Each timer counts full
 * 16-bit and is read as a signed 16-bit delta, which self-handles wraparound.
 */
#include "include/encoder.h"

/* ----------------------------------------------------------------------------
   ENCODER CONFIG TABLE
   ----------------------------------------------------------------------------
   One entry per motor. GPIO is AF input with NOPULL (the hall encoder modules
   carry their own bus pull-ups - see PEO_PINMAP.H section 4); the AF is set
   explicitly with GPIO_PinAFConfig (StdPeriph does not auto-map, unlike HAL).
   ---------------------------------------------------------------------------- */
/** Hardware wiring for one wheel encoder; used only during initialization. */
typedef struct {
    TIM_TypeDef *tim;       /* Timer that counts this wheel encoder edges. */
    GPIO_TypeDef *ch1_port; /* GPIO bank receiving encoder A. */
    uint16_t ch1_pin;       /* GPIO bit mask for encoder A. */
    uint8_t ch1_src;        /* Pin index for encoder A alternate-function routing. */
    GPIO_TypeDef *ch2_port; /* GPIO bank receiving encoder B. */
    uint16_t ch2_pin;       /* GPIO bit mask for encoder B. */
    uint8_t ch2_src;        /* Pin index for encoder B alternate-function routing. */
    uint8_t       af;       /* Alternate function linking both pins to the timer. */
} encoder_cfg_t;

static const encoder_cfg_t enc_cfg[4] = {  /* Fixed hardware routing for M1..M4 encoders in wheel-index order. */
    /* M1: TIM1 CH1/CH2 = PE9/PE11 (AF1) */
    { PEO_PM_ENC1_TIM, PEO_PM_ENC1_CH1_PORT, PEO_PM_ENC1_CH1_PIN, PEO_PM_ENC1_CH1_SRC,
      PEO_PM_ENC1_CH2_PORT, PEO_PM_ENC1_CH2_PIN, PEO_PM_ENC1_CH2_SRC, PEO_PM_ENC1_AF },
    /* M2: TIM3 CH1/CH2 = PA6/PA7 (AF2) */
    { PEO_PM_ENC2_TIM, PEO_PM_ENC2_CH1_PORT, PEO_PM_ENC2_CH1_PIN, PEO_PM_ENC2_CH1_SRC,
      PEO_PM_ENC2_CH2_PORT, PEO_PM_ENC2_CH2_PIN, PEO_PM_ENC2_CH2_SRC, PEO_PM_ENC2_AF },
    /* M3 encoder: TIM4 CH1/CH2 on PD12/PD13 (AF2). */
    { PEO_PM_ENC3_TIM, PEO_PM_ENC3_CH1_PORT, PEO_PM_ENC3_CH1_PIN, PEO_PM_ENC3_CH1_SRC,
      PEO_PM_ENC3_CH2_PORT, PEO_PM_ENC3_CH2_PIN, PEO_PM_ENC3_CH2_SRC, PEO_PM_ENC3_AF },
    /* M4 encoder: TIM8 CH1/CH2 on PC6/PC7 (AF3). */
    { PEO_PM_ENC4_TIM, PEO_PM_ENC4_CH1_PORT, PEO_PM_ENC4_CH1_PIN, PEO_PM_ENC4_CH1_SRC,
      PEO_PM_ENC4_CH2_PORT, PEO_PM_ENC4_CH2_PIN, PEO_PM_ENC4_CH2_SRC, PEO_PM_ENC4_AF },
};

/* Last counter value per motor, for the delta read. */
static uint16_t g_last_cnt[4] = { 0, 0, 0, 0 };  /* Previous 16-bit counter reading per wheel; the baseline for the next delta. */

/* ----------------------------------------------------------------------------
   INTERNAL
   ---------------------------------------------------------------------------- */

/**
 * @brief Connect one pair of encoder signals to a hardware counter.
 *
 * Called four times by ENCODER_Init. Configures alternate-function pins, input
 * filtering, and x4 quadrature counting before starting the timer.
 * @param cfg Non-NULL configuration describing this wheel timer, its two GPIO pins, and
 *            alternate function.
 * @retval None This function does not return a value.
 */
static void encoder_init_one(const encoder_cfg_t *cfg)
{
    GPIO_InitTypeDef     gpio;  /* GPIO setup record: selects pin routing, electrical drive, pulls, and switching speed. */
    TIM_TimeBaseInitTypeDef tb;  /* Timer setup record containing prescaler and period counts for the requested rate. */
    TIM_ICInitTypeDef    ic;  /* Encoder input-capture setup, including the digital filter that rejects short noise pulses. */

    /* GPIO CH1/CH2 -> AF input, NOPULL (module has its own bus pull-ups). */
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;

    gpio.GPIO_Pin = cfg->ch1_pin;
    GPIO_Init(cfg->ch1_port, &gpio);
    GPIO_PinAFConfig(cfg->ch1_port, cfg->ch1_src, cfg->af);

    gpio.GPIO_Pin = cfg->ch2_pin;
    GPIO_Init(cfg->ch2_port, &gpio);
    GPIO_PinAFConfig(cfg->ch2_port, cfg->ch2_src, cfg->af);

    /* Time-base: full 16-bit period. CounterMode is ignored in encoder mode
       (the counter is clocked by the TI1/TI2 edges, not by CK_CNT). */
    tb.TIM_Prescaler         = 0;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = 0xFFFF;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(cfg->tim, &tb);

    /* Encoder interface mode TI12 = both edges of both channels = x4. */
    TIM_EncoderInterfaceConfig(cfg->tim, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    /* IC filter on both channels (anti-EMI digital filter). */
    ic.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter    = 0x0F;

    ic.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(cfg->tim, &ic);
    ic.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(cfg->tim, &ic);

    /* Clear the counter and start it. */
    TIM_SetCounter(cfg->tim, 0);
    TIM_Cmd(cfg->tim, ENABLE);
}

/* ----------------------------------------------------------------------------
   PUBLIC API
   ---------------------------------------------------------------------------- */

/**
 * @brief Prepare all four hardware wheel counters and clear their reading baselines.
 *
 * Called once from main before CONTROL starts; clocks and NVIC setup must already be
 * complete. No encoder interrupts are needed.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 */
void ENCODER_Init(void)
{
    uint8_t i;

    encoder_init_one(&enc_cfg[0]);   /* M1 (TIM1) */
    encoder_init_one(&enc_cfg[1]);   /* M2 (TIM3) */
    encoder_init_one(&enc_cfg[2]);   /* M3 encoder timer (TIM4). */
    encoder_init_one(&enc_cfg[3]);   /* M4 encoder timer (TIM8). */

    for (i = 0; i < 4; i++)
        g_last_cnt[i] = 0;
}

/**
 * @brief Measure signed wheel movement since the previous read for this wheel.
 *
 * Called once per wheel per TIM6 tick. Reading advances that wheel baseline, so extra
 * calls would consume counts intended for the controller. Wrap handling assumes fewer
 * than 32768 counts of movement between reads.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right.
 * @retval value Signed encoder-count change; zero for an invalid wheel index.
 */
int16_t ENCODER_GetDelta(uint8_t motor)
{
    uint16_t now;  /* Current raw 16-bit hardware count; wraps from 65535 to zero. */
    int16_t  delta;  /* Signed difference from the last reading, measured in encoder counts. */

    if (motor > ENCODER_M4)
        return 0;

    now   = (uint16_t)TIM_GetCounter(enc_cfg[motor].tim);
    delta = (int16_t)(now - g_last_cnt[motor]);   /* uint16 sub -> int16, wraparound-safe */
    g_last_cnt[motor] = now;

    return delta;
}

/**
 * @brief Read the raw hardware encoder count without changing the delta baseline.
 *
 * Available for diagnostics without consuming the next CONTROL delta measurement.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right.
 * @retval value Raw count 0..65535, wrapping at the hardware limit; zero for an invalid
 *               index.
 */
uint16_t ENCODER_GetCount(uint8_t motor)
{
    if (motor > ENCODER_M4)
        return 0;

    return (uint16_t)TIM_GetCounter(enc_cfg[motor].tim);
}
