/**
 ******************************************************************************
 * @file    timer.h
 * @brief   PEO Rover - TIMER component public API (SysTick millisecond timebase)
 *
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
 ******************************************************************************
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
       RELOAD = SYSCLK / 1000 = 168000000 / 1000 = 168000
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
 * @brief  Configure SysTick for a 1 ms interrupt and start it.
 * @note   Call once in main(), after RCC_Clock_Init()/NVIC_InitAll().
 *         mdelay()/get_tick() are only meaningful after this call.
 */
void TIMER_Init(void);

/**
 * @brief  Return the current millisecond counter (ms since TIMER_Init()).
 */
uint32_t TIMER_GetTick(void);

/**
 * @brief  eMPL Motion Driver platform function (inv_mpu.c maps get_ms to it).
 * @param  count  If non-NULL, the current ms value is also written here.
 * @return Current millisecond counter.
 */
unsigned long get_tick(unsigned long *count);

/**
 * @brief  eMPL Motion Driver platform function (inv_mpu.c maps delay_ms to it).
 * @param  ms  Delay duration in milliseconds (>= 1 for full resolution).
 * @note   Blocking, SysTick-counter based. Requires TIMER_Init() first and
 *         interrupts enabled (the counter advances in SysTick_Handler).
 */
void mdelay(unsigned long ms);

#endif /* __TIMER_H__ */
