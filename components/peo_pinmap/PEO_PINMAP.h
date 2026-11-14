#ifndef __PEO_PINMAP_H__
#define __PEO_PINMAP_H__

/**
 * @file    PEO_PINMAP.h
 * @brief   PEO Rover - ESP32-C3 SuperMini pin assignment header
 *          (single source of truth for all pin/peripheral macros)
 *
 * C3 <-> S3 链路引脚占位（待用户确认接线后修改；S3 端对应宏在
 * peo_rover_esp32s3/components/peo_pinmap/PEO_PINMAP.h §C3 链路）：
 *   C3 UART1 GPIO4(TX)/GPIO5(RX) <-> S3 UART_NUM_2 GPIO18(RX)/GPIO17(TX)
 * 注意：C3 GPIO20/21 是 UART0，被板载 USB 转串口占用，不可作链路引脚。
 */

#include "driver/gpio.h"
#include "driver/uart.h"

/* ============================================================================
   ESP32-C3 <-> ESP32-S3 UART 链路（c3_uart 组件使用）
   波特率与 S3 端 PEO_PM_C3_BAUDRATE(115200) 一致。
   ============================================================================ */
#define PEO_PM_C3_UART      UART_NUM_1
#define PEO_PM_C3_TX        GPIO_NUM_4
#define PEO_PM_C3_RX        GPIO_NUM_5
#define PEO_PM_C3_BAUDRATE  (115200U)

#endif /* __PEO_PINMAP_H__ */
