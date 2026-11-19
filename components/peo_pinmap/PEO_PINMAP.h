#ifndef __PEO_PINMAP_H__
#define __PEO_PINMAP_H__

/**
 * @file    PEO_PINMAP.h
 * @brief   PEO Rover - ESP32-C3 SuperMini pin assignment header
 *          (single source of truth for all pin/peripheral macros)
 *
 * C3 -> F407 链路（直连 F407 USART3，绕过 S3；TX/RX 交叉、共地）：
 *   C3 UART1 GPIO4(TX)/GPIO5(RX) <-> F407 USART3 PD8(TX)/PD9(RX)
 * 注意：C3 GPIO20/21 是 UART0，被板载 USB 转串口占用，不可作链路引脚。
 */

#include "driver/gpio.h"
#include "driver/uart.h"

/* ============================================================================
   ESP32-C3 -> F407 USART3 链路（c3_uart 组件使用）
   波特率与 F407 端 USART3(115200) 一致。
   ============================================================================ */
#define PEO_PM_C3_UART      UART_NUM_1
#define PEO_PM_C3_TX        GPIO_NUM_4
#define PEO_PM_C3_RX        GPIO_NUM_5
#define PEO_PM_C3_BAUDRATE  (115200U)

/* 板载蓝色状态灯：GPIO8，低电平点亮（active LOW，从 3V3 经限流电阻到引脚）。
   GPIO8 是 strapping 脚，LED 接法保证上电为高电平（灯灭），上电后 LEDC 驱动无碍。 */
#define PEO_PM_LED_GPIO     GPIO_NUM_8

#endif /* __PEO_PINMAP_H__ */
