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
#include "COMPONENTS/PEO_PINMAP.H"          /* PEO_PM_SYSCLK_HZ */

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

    /* 启用 DWT 周期计数器（mdelay 忙等用，独立于 SysTick，可在中断上下文安全使用）。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;
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
 *         Blocking ms delay on DWT, split into chunks of at most 1000 ms.
 *         Unsigned subtraction handles a 32-bit counter wrap.
 */
void mdelay(unsigned long ms)
{
    /* DWT 周期计数器忙等（不依赖 SysTick）：可在中断上下文安全使用。
       否则 EXTI3 内 FIFO 溢出 -> mpu_reset_fifo -> mdelay(50) 会因 SysTick
       优先级更低、抢不进 EXTI3 而永久等待（死锁）。无符号减法处理 CYCCNT 回绕。 */
    while (ms > 0)
    {
        uint32_t chunk = (ms > 1000UL) ? 1000UL : (uint32_t)ms;
        uint32_t cycles = chunk * (PEO_PM_SYSCLK_HZ / 1000UL);
        uint32_t start = DWT->CYCCNT;
        while ((DWT->CYCCNT - start) < cycles)
        {
            /* spin */
        }
        ms -= chunk;
    }
}
