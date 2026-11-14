/**
 ******************************************************************************
 * @file    timer.c
 * @brief   PEO Rover - TIMER component: SysTick millisecond timebase
 *
 * The ONLY place in the firmware that owns the SysTick core timer. All
 * timestamps (motion driver, ranging, control-loop timing) read this counter.
 ******************************************************************************
 */
#include "include/timer.h"
#include "PEO_PINMAP.H"          /* PEO_PM_SYSCLK_HZ */

/** Millisecond counter, incremented by SysTick_Handler every 1 ms. */
static volatile uint32_t g_ms_tick = 0;

/**
 * @brief  SysTick interrupt handler (1 ms). Increments the timebase.
 * @note   Overrides the [WEAK] default in startup_stm32f40xx.s. Must NOT be
 *         defined again in stm32f4xx_it.c (it currently is not).
 */
void SysTick_Handler(void)
{
    g_ms_tick++;
}

/**
 * @brief  Configure SysTick for a 1 ms interrupt and start it.
 * @note   CLKSOURCE = HCLK (168 MHz), so RELOAD = 168000000 / 1000 = 168000.
 *         SysTick_Config() writes RELOAD = ticks-1, clears VAL, enables the
 *         counter + interrupt, and sets SysTick to the lowest priority.
 *         It returns 1 only if ticks-1 overflows the 24-bit reload, which
 *         cannot happen here (168000 << 0x1000000).
 */
void TIMER_Init(void)
{
    if(SysTick_Config(PEO_PM_SYSCLK_HZ / PEO_TMR_TICK_HZ))
    {
        while (1);   /* unreachable: 168000 reload always fits */
    }
}

/**
 * @brief  Return the current millisecond counter.
 */
uint32_t TIMER_GetTick(void)
{
    return g_ms_tick;
}

/**
 * @brief  eMPL Motion Driver platform function (inv_mpu.c: #define get_ms get_tick).
 *         Returns the ms counter; writes it to *count when non-NULL.
 */
unsigned long get_tick(unsigned long *count)
{
    if (count != 0)
        *count = g_ms_tick;
    return g_ms_tick;
}

/**
 * @brief  eMPL Motion Driver platform function (inv_mpu.c: #define delay_ms mdelay).
 *         Blocking ms delay on the SysTick counter. The subtraction is
 *         unsigned, so it stays correct across a 32-bit counter wrap.
 */
void mdelay(unsigned long ms)
{
    unsigned long start = g_ms_tick;
    while ((g_ms_tick - start) < ms)
    {
        /* spin until the SysTick counter advances by 'ms' */
    }
}
