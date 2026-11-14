/**
 * @file    builtin_led.c
 * @brief   片上 LED D2（PA1，sink：低电平点亮）驱动。
 *
 *  GPIOA 时钟由 RCC 组件统一使能，本组件不自行开启。
 *  LED D2 阴极接 PA1、阳极经限流电阻接 VCC，故 PA1 拉低点亮、拉高熄灭。
 */
#include "builtin_led.h"

/**
 * @brief  初始化片上 LED：推挽输出，初始熄灭。
 */
void BUILTIN_LED_Init(void)
{
    GPIO_InitTypeDef led;

    led.GPIO_Mode  = GPIO_Mode_OUT;      /* 通用推挽输出 */
    led.GPIO_OType = GPIO_OType_PP;
    led.GPIO_PuPd  = GPIO_PuPd_NOPULL;   /* 输出模式，上拉/下拉无影响 */
    led.GPIO_Speed = GPIO_Speed_2MHz;    /* LED 无需高速 */
    led.GPIO_Pin   = PEO_PM_LED_PIN;

    GPIO_Init(PEO_PM_LED_PORT, &led);
    BUILTIN_LED_Off();                   /* 上电默认熄灭 */
}

/**
 * @brief  点亮 LED。
 */
void BUILTIN_LED_On(void)
{
    if (PEO_PM_LED_ON_LEVEL == Bit_RESET)
        GPIO_ResetBits(PEO_PM_LED_PORT, PEO_PM_LED_PIN);   /* sink：拉低点亮 */
    else
        GPIO_SetBits(PEO_PM_LED_PORT, PEO_PM_LED_PIN);
}

/**
 * @brief  熄灭 LED。
 */
void BUILTIN_LED_Off(void)
{
    if (PEO_PM_LED_ON_LEVEL == Bit_RESET)
        GPIO_SetBits(PEO_PM_LED_PORT, PEO_PM_LED_PIN);     /* sink：拉高熄灭 */
    else
        GPIO_ResetBits(PEO_PM_LED_PORT, PEO_PM_LED_PIN);
}

/**
 * @brief  翻转 LED。
 */
void BUILTIN_LED_Toggle(void)
{
    GPIO_ToggleBits(PEO_PM_LED_PORT, PEO_PM_LED_PIN);
}
