/**
 ******************************************************************************
 * @file    encoder.h
 * @brief   PEO Rover - ENCODER component public API
 *
 * 4x quadrature encoders on TIM1/3/4/8, encoder interface mode TI12
 * (both edges of both channels = x4). Each timer counts full 16-bit
 * (ARR = 0xFFFF) and is read as a 16-bit DELTA, which self-handles the
 * counter wraparound with no overflow interrupt.
 *
 * Routing (PEO_PINMAP.H section 4):
 *   M1 -> TIM1 CH1/CH2 = PE9/PE11   (AF1)
 *   M2 -> TIM3 CH1/CH2 = PA6/PA7    (AF2)
 *   M3 -> TIM8 CH1/CH2 = PC6/PC7    (AF3)
 *   M4 -> TIM4 CH1/CH2 = PD12/PD13  (AF2)
 *
 * Hall encoder: 11 PPR on the motor shaft -> x4 = 44 counts/motor-rev;
 * gearbox 45:1 -> 1980 counts/output-rev; 65 mm wheel -> ~0.103 mm/count.
 *
 * Architecture rule (see PEO_PINMAP.H / rcc.h):
 *   - This component ONLY configures its own peripherals (GPIO + TIM).
 *   - Timer clocks are enabled by RCC_Clock_Init() (TIM1/8 on APB2,
 *     TIM3/4 on APB1) - never re-enabled here.
 *   - Pure 16-bit delta read uses NO timer interrupt, so no NVIC channel
 *     is configured here.
 ******************************************************************************
 */
#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stm32f4xx.h"
#include "PEO_PINMAP.H"

/* ============================================================================
   ENCODER INDICES (0-based, same as the MOTOR component's M1..M4)
   ============================================================================ */
#define ENCODER_M1      (0U)
#define ENCODER_M2      (1U)
#define ENCODER_M3      (2U)
#define ENCODER_M4      (3U)

/* ============================================================================
   PUBLIC API
   ============================================================================ */

/**
 * @brief  Initialize all 4 encoders (GPIO AF + TIM encoder interface mode).
 * @note   Call once in main(), after RCC_Clock_Init()/NVIC_InitAll().
 *         Counters start at 0.
 */
void ENCODER_Init(void);

/**
 * @brief  Return the 16-bit counter delta since the previous call.
 * @param  motor  ENCODER_M1..ENCODER_M4.
 * @return Signed count delta (positive/negative = rotation direction).
 *         The subtraction is done on uint16 then cast to int16, so it stays
 *         correct across the 16-bit wraparound.
 * @note   The meaning of "delta" is the count change since the LAST call to
 *         this function for the same motor - call it at a fixed rate (the
 *         1 kHz control loop) so the delta maps to a fixed time window.
 *         Full speed is ~4.3 counts/ms, so a 1 kHz call is far from the
 *         int16 overflow limit (32767 counts).
 */
int16_t ENCODER_GetDelta(uint8_t motor);

/**
 * @brief  Return the current raw 16-bit counter value.
 * @param  motor  ENCODER_M1..ENCODER_M4.
 * @return Current counter (0..0xFFFF).
 */
uint16_t ENCODER_GetCount(uint8_t motor);

#endif /* __ENCODER_H__ */
