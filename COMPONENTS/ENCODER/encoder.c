/**
 ******************************************************************************
 * @file    encoder.c
 * @brief   PEO Rover - ENCODER component: 4x quadrature encoders (TIM1/3/4/8)
 *
 * Encoder interface mode TI12 (both edges = x4). Each timer counts full
 * 16-bit and is read as a signed 16-bit delta, which self-handles wraparound.
 ******************************************************************************
 */
#include "include/encoder.h"

/* ----------------------------------------------------------------------------
   ENCODER CONFIG TABLE
   ----------------------------------------------------------------------------
   One entry per motor. GPIO is AF input with NOPULL (the hall encoder modules
   carry their own bus pull-ups - see PEO_PINMAP.H section 4); the AF is set
   explicitly with GPIO_PinAFConfig (StdPeriph does not auto-map, unlike HAL).
   ---------------------------------------------------------------------------- */
typedef struct {
    TIM_TypeDef *tim;
    GPIO_TypeDef *ch1_port; uint16_t ch1_pin; uint8_t ch1_src;
    GPIO_TypeDef *ch2_port; uint16_t ch2_pin; uint8_t ch2_src;
    uint8_t       af;
} encoder_cfg_t;

static const encoder_cfg_t enc_cfg[4] = {
    /* M1: TIM1 CH1/CH2 = PE9/PE11 (AF1) */
    { PEO_PM_ENC1_TIM, PEO_PM_ENC1_CH1_PORT, PEO_PM_ENC1_CH1_PIN, PEO_PM_ENC1_CH1_SRC,
      PEO_PM_ENC1_CH2_PORT, PEO_PM_ENC1_CH2_PIN, PEO_PM_ENC1_CH2_SRC, PEO_PM_ENC1_AF },
    /* M2: TIM3 CH1/CH2 = PA6/PA7 (AF2) */
    { PEO_PM_ENC2_TIM, PEO_PM_ENC2_CH1_PORT, PEO_PM_ENC2_CH1_PIN, PEO_PM_ENC2_CH1_SRC,
      PEO_PM_ENC2_CH2_PORT, PEO_PM_ENC2_CH2_PIN, PEO_PM_ENC2_CH2_SRC, PEO_PM_ENC2_AF },
    /* M3 编码器：TIM4 CH1/CH2 = PD12/PD13 (AF2) */
    { PEO_PM_ENC3_TIM, PEO_PM_ENC3_CH1_PORT, PEO_PM_ENC3_CH1_PIN, PEO_PM_ENC3_CH1_SRC,
      PEO_PM_ENC3_CH2_PORT, PEO_PM_ENC3_CH2_PIN, PEO_PM_ENC3_CH2_SRC, PEO_PM_ENC3_AF },
    /* M4 编码器：TIM8 CH1/CH2 = PC6/PC7 (AF3) */
    { PEO_PM_ENC4_TIM, PEO_PM_ENC4_CH1_PORT, PEO_PM_ENC4_CH1_PIN, PEO_PM_ENC4_CH1_SRC,
      PEO_PM_ENC4_CH2_PORT, PEO_PM_ENC4_CH2_PIN, PEO_PM_ENC4_CH2_SRC, PEO_PM_ENC4_AF },
};

/* Last counter value per motor, for the delta read. */
static uint16_t g_last_cnt[4] = { 0, 0, 0, 0 };

/* ----------------------------------------------------------------------------
   INTERNAL
   ---------------------------------------------------------------------------- */

/**
 * @brief  Configure one encoder timer: GPIO AF (CH1+CH2), time-base,
 *         encoder interface mode TI12, IC filter, and enable the counter.
 */
static void encoder_init_one(const encoder_cfg_t *cfg)
{
    GPIO_InitTypeDef     gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_ICInitTypeDef    ic;

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
 * @brief  Initialize all 4 encoders and reset the delta baseline.
 */
void ENCODER_Init(void)
{
    uint8_t i;

    encoder_init_one(&enc_cfg[0]);   /* M1 (TIM1) */
    encoder_init_one(&enc_cfg[1]);   /* M2 (TIM3) */
    encoder_init_one(&enc_cfg[2]);   /* M3 编码器 (TIM4) */
    encoder_init_one(&enc_cfg[3]);   /* M4 编码器 (TIM8) */

    for (i = 0; i < 4; i++)
        g_last_cnt[i] = 0;
}

/**
 * @brief  Return the 16-bit counter delta since the previous call.
 */
int16_t ENCODER_GetDelta(uint8_t motor)
{
    uint16_t now;
    int16_t  delta;

    if (motor > ENCODER_M4)
        return 0;

    now   = (uint16_t)TIM_GetCounter(enc_cfg[motor].tim);
    delta = (int16_t)(now - g_last_cnt[motor]);   /* uint16 sub -> int16, wraparound-safe */
    g_last_cnt[motor] = now;

    return delta;
}

/**
 * @brief  Return the current raw 16-bit counter value.
 */
uint16_t ENCODER_GetCount(uint8_t motor)
{
    if (motor > ENCODER_M4)
        return 0;

    return (uint16_t)TIM_GetCounter(enc_cfg[motor].tim);
}
