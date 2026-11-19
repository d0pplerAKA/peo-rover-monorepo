/**
 * @file  timer.h
 * @brief Expose the rover timebase and motion-driver timing adapters.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * Design rule (architecture decision 2026-11-12):
 *   - The single millisecond timebase lives HERE, on the Cortex-M4 SysTick.
 *   - Every timestamp in the firmware reads this counter (never its own timer).
 *   - The eMPL Motion Driver (MPU6050 DMP, COMPONENTS/motion_driver_dmp)
 *     maps its get_ms()/delay_ms() onto get_tick()/mdelay() below:
 *         inv_mpu.c:  #define get_ms     get_tick
 *         inv_mpu.c:  #define delay_ms   mdelay
 *
 * main() call order:
 *   RCC_Clock_Init();   <- COMPONENTS/RCC
 *   NVIC_InitAll();     <- COMPONENTS/NVIC
 *   TIMER_Init();       <- this component (1 ms SysTick timebase)
 *   ... module inits ... (motion driver / TOF may now use get_tick / mdelay)
 */
#ifndef __TIMER_H__
#define __TIMER_H__

#include "stm32f4xx.h"

/* ============================================================================
   SYSTICK TIMEBASE
   ----------------------------------------------------------------------------
   SysTick counts down from RELOAD to 0 at the core clock (HCLK = SYSCLK =
   168 MHz), then reloads and fires SysTick_Handler. With PEO_TMR_TICK_HZ =
   1000 the handler runs every 1 ms and increments the millisecond counter:
       ticks per interrupt = SYSCLK / 1000 = 168000; hardware LOAD = 167999
   SysTick is a Cortex-M4 CORE peripheral (not an RCC/AHB/APB peripheral), so
   no clock-enable call is required - only LOAD + the 3 CTRL bits.
   SysTick_Config() sets the SysTick interrupt to the LOWEST priority (15):
   fine for a background counter, it never preempts VL53L1X/MPU interrupts.
   ============================================================================ */
#define PEO_TMR_TICK_HZ         (1000UL)   /* SysTick interrupt frequency */
#define PEO_TMR_TICK_MS         (1UL)      /* milliseconds per SysTick tick */

/* ============================================================================
   PUBLIC API
   ============================================================================ */

/**
 * @brief Start the millisecond timestamp source and the CPU-cycle delay counter.
 *
 * Called early by main after RCC/NVIC setup. SysTick updates timestamps; DWT CYCCNT
 * measures mdelay waits independently of SysTick interrupts.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note Call once in main(), after RCC_Clock_Init()/NVIC_InitAll(). mdelay()/get_tick()
 *       are only meaningful after this call.
 */
void TIMER_Init(void);

/**
 * @brief Read the common millisecond timestamp used throughout the rover.
 *
 * Used by telemetry and link timeouts. The 32-bit counter starts at boot and wraps
 * naturally; elapsed-time comparisons use unsigned subtraction.
 * @note No arguments are required.
 * @retval value Current SysTick count in milliseconds, modulo 2^32.
 */
uint32_t TIMER_GetTick(void);

/**
 * @brief Provide the millisecond clock in the form expected by the motion-driver
 *        library.
 *
 * Called through the eMPL get_ms platform mapping; returns the same time source as
 * TIMER_GetTick.
 * @param count Optional output pointer receiving the timestamp; NULL skips this extra
 *              write.
 * @retval value Current millisecond counter, also written to count when non-NULL.
 */
unsigned long get_tick(unsigned long *count);

/**
 * @brief Wait for a duration measured by the CPU-cycle counter.
 *
 * Called by main and the eMPL delay_ms platform hook after TIMER_Init. Splits long
 * delays into at most 1000 ms chunks so 32-bit cycle arithmetic fits. It can complete in
 * an interrupt because DWT does not depend on SysTick servicing.
 * @param ms Blocking delay in milliseconds; zero returns immediately.
 * @retval None This function does not return a value.
 * @note Blocking, DWT-cycle-counter based. Requires TIMER_Init() first. It can finish
 *       inside an interrupt without waiting for SysTick, but occupies the CPU throughout
 *       the wait.
 */
void mdelay(unsigned long ms);

#endif /* __TIMER_H__ */
