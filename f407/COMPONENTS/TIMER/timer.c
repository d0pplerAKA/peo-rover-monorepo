/**
 * @file  timer.c
 * @brief Provide shared millisecond timestamps and CPU-cycle-based blocking delays.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */
/*
 * The ONLY place in the firmware that owns the SysTick core timer. All
 * timestamps (motion driver, ranging, control-loop timing) read this counter.
 */
#include "include/timer.h"
#include "COMPONENTS/PEO_PINMAP.H"          /* PEO_PM_SYSCLK_HZ */

/** Millisecond counter, incremented by SysTick_Handler every 1 ms. */
static volatile uint32_t g_ms_tick = 0;  /* Shared unsigned millisecond counter; advances in SysTick and wraps at 2^32. */

/**
 * @brief Advance the shared millisecond counter on each SysTick interrupt.
 *
 * Called by the processor every nominal millisecond after TIMER_Init; this replaces the
 * weak startup handler.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note Overrides the [WEAK] default in startup_stm32f40xx.s. Must NOT be defined again
 *       in stm32f4xx_it.c (it currently is not).
 */
void SysTick_Handler(void)
{
    g_ms_tick++;
}

/**
 * @brief Start the millisecond timestamp source and the CPU-cycle delay counter.
 *
 * Called early by main after RCC/NVIC setup. SysTick updates timestamps; DWT CYCCNT
 * measures mdelay waits independently of SysTick interrupts.
 * @note No arguments are required.
 * @retval None This function does not return a value.
 * @note CLKSOURCE = HCLK (168 MHz), so RELOAD = 168000000 / 1000 = 168000.
 *       SysTick_Config() writes RELOAD = ticks-1, clears VAL, enables the counter +
 *       interrupt, and sets SysTick to the lowest priority. It returns 1 only if ticks-1
 *       overflows the 24-bit reload, which cannot happen here (168000 << 0x1000000).
 */
void TIMER_Init(void)
{
    if(SysTick_Config(PEO_PM_SYSCLK_HZ / PEO_TMR_TICK_HZ))
    {
        while (1);   /* unreachable: 168000 reload always fits */
    }

    /* Enable the DWT CPU-cycle counter for mdelay() busy waits. It runs
       independently of SysTick, allowing a delay to complete inside an interrupt. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief Read the common millisecond timestamp used throughout the rover.
 *
 * Used by telemetry and link timeouts. The 32-bit counter starts at boot and wraps
 * naturally; elapsed-time comparisons use unsigned subtraction.
 * @note No arguments are required.
 * @retval value Current SysTick count in milliseconds, modulo 2^32.
 */
uint32_t TIMER_GetTick(void)
{
    return g_ms_tick;
}

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
unsigned long get_tick(unsigned long *count)
{
    if (count != 0)
        *count = g_ms_tick;
    return g_ms_tick;
}

/**
 * @brief Wait for a duration measured by the CPU-cycle counter.
 *
 * Called by main and the eMPL delay_ms platform hook after TIMER_Init. Splits long
 * delays into at most 1000 ms chunks so 32-bit cycle arithmetic fits. It can complete in
 * an interrupt because DWT does not depend on SysTick servicing.
 * @param ms Blocking delay in milliseconds; zero returns immediately.
 * @retval None This function does not return a value.
 */
void mdelay(unsigned long ms)
{
    /* Busy-wait on DWT cycles, independently of SysTick, so it can finish
       inside an interrupt. Otherwise EXTI3 -> FIFO overflow -> mpu_reset_fifo()
       -> mdelay(50) could deadlock: lower-priority SysTick cannot interrupt EXTI3.
       Unsigned subtraction handles CYCCNT wraparound. The wait still occupies
       the CPU and delays lower-priority work. */
    while (ms > 0)
    {
        uint32_t chunk = (ms > 1000UL) ? 1000UL : (uint32_t)ms;  /* Current delay slice in milliseconds, limited to 1000 ms. */
        uint32_t cycles = chunk * (PEO_PM_SYSCLK_HZ / 1000UL);  /* Number of CPU clock cycles corresponding to the current delay slice. */
        uint32_t start = DWT->CYCCNT;  /* DWT cycle-counter value at the start of this slice; subtraction handles wrapping. */
        while ((DWT->CYCCNT - start) < cycles)
        {
            /* spin */
        }
        ms -= chunk;
    }
}
