/**
 * @file  encoder.h
 * @brief Expose raw encoder counts and per-sample wheel movement.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * 4x quadrature encoders on TIM1/3/4/8, encoder interface mode TI12
 * (both edges of both channels = x4). Each timer counts full 16-bit
 * (ARR = 0xFFFF) and is read as a 16-bit DELTA, which self-handles the
 * counter wraparound with no overflow interrupt.
 *
 * Routing (PEO_PINMAP.H section 4):
 *   M1 -> TIM1 CH1/CH2 = PE9/PE11   (AF1)
 *   M2 -> TIM3 CH1/CH2 = PA6/PA7    (AF2)
 *   M3 encoder -> TIM4 CH1/CH2 = PD12/PD13 (AF2)
 *   M4 encoder -> TIM8 CH1/CH2 = PC6/PC7   (AF3)
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
 */
#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stm32f4xx.h"
#include "COMPONENTS/PEO_PINMAP.H"

/* ============================================================================
   ENCODER INDICES (0-based, same as the MOTOR component's M1..M4)
   ============================================================================ */
#define ENCODER_M1      (0U)  /* Zero-based index for the M1 front left wheel encoder. */
#define ENCODER_M2      (1U)  /* Zero-based index for the M2 front right wheel encoder. */
#define ENCODER_M3      (2U)  /* Zero-based index for the M3 rear left wheel encoder. */
#define ENCODER_M4      (3U)  /* Zero-based index for the M4 rear right wheel encoder. */

/* ============================================================================
   PUBLIC API
   ============================================================================ */

/**
 * @brief Prepare all four hardware wheel counters and clear their reading baselines.
 *
 * Called once from main before CONTROL starts; clocks and NVIC setup must already be
 * complete. No encoder interrupts are needed.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note Call once in main(), after RCC_Clock_Init()/NVIC_InitAll(). Counters start at 0.
 */
void ENCODER_Init(void);

/**
 * @brief Measure signed wheel movement since the previous read for this wheel.
 *
 * Called once per wheel per TIM6 tick. Reading advances that wheel baseline, so extra
 * calls would consume counts intended for the controller. Wrap handling assumes fewer
 * than 32768 counts of movement between reads.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right.
 * @retval value Signed encoder-count change; zero for an invalid wheel index.
 * @note The meaning of "delta" is the count change since the LAST call to this function
 *       for the same motor - call it at a fixed rate (the 1 kHz control loop) so the
 *       delta maps to a fixed time window. Full speed is ~4.3 counts/ms, so a 1 kHz call
 *       is far from the int16 overflow limit (32767 counts).
 */
int16_t ENCODER_GetDelta(uint8_t motor);

/**
 * @brief Read the raw hardware encoder count without changing the delta baseline.
 *
 * Available for diagnostics without consuming the next CONTROL delta measurement.
 * @param motor Wheel index 0..3: M1 front left, M2 front right, M3 rear left, M4 rear
 *              right.
 * @retval value Raw count 0..65535, wrapping at the hardware limit; zero for an invalid
 *               index.
 */
uint16_t ENCODER_GetCount(uint8_t motor);

#endif /* __ENCODER_H__ */
