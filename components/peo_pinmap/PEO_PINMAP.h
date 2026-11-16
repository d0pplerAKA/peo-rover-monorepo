#ifndef __PEO_PINMAP_H__
#define __PEO_PINMAP_H__

/**
 * @file    PEO_PINMAP.h
 * @brief   PEO Rover - ESP32-S3 N8R8 pin assignment header
 *          (8 MB Quad Flash + 8 MB Octal PSRAM)
 *          GPIO35/36/37 are reserved for Octal PSRAM; do not assign peripherals.
 *          (single source of truth for all pin/peripheral macros)
 *
 * 本文件从 main/ 移入 components/peo_pinmap/（独立组件），
 * 供 esp_link / camera_service 等所有组件共享引用。
 */

#include "driver/gpio.h"

/* ============================================================================
   OV5640 摄像头（camera_service 使用）
   ============================================================================ */
#define PEO_PM_CAM_SCL      GPIO_NUM_1
#define PEO_PM_CAM_SDA      GPIO_NUM_2
#define PEO_PM_CAM_D0       GPIO_NUM_39
#define PEO_PM_CAM_D2       GPIO_NUM_38
#define PEO_PM_CAM_D4       GPIO_NUM_47
#define PEO_PM_CAM_D6       GPIO_NUM_21
#define PEO_PM_CAM_PCLK     GPIO_NUM_13
#define PEO_PM_CAM_PWDN     GPIO_NUM_14
#define PEO_PM_CAM_VS       GPIO_NUM_4
#define PEO_PM_CAM_HS       GPIO_NUM_5
#define PEO_PM_CAM_RST      GPIO_NUM_6
#define PEO_PM_CAM_D1       GPIO_NUM_7
#define PEO_PM_CAM_D3       GPIO_NUM_15
#define PEO_PM_CAM_D5       GPIO_NUM_16
#define PEO_PM_CAM_D7       GPIO_NUM_9
/* 模组自带 XCLK 时钟：GPIO_NUM_NC = 不占用引脚，ESP32 不输出时钟。
   注意：xclk_freq_hz 仍需按模组晶振实际频率填写（OV5640 PLL 计算用，见 camera_service）。 */
#define PEO_PM_CAM_XCLK     GPIO_NUM_NC

/* ============================================================================
   ESP32-S3 <-> F407 UART 链路（esp_link 组件使用）
   ----------------------------------------------------------------------------
   对应 F407 侧 USART2：PD5=F407_TX -> ESP32_RX、PD6=F407_RX <- ESP32_TX。
   波特率与 F407 PEO_PM_ESP_BAUDRATE(921600) 一致。
   ============================================================================ */
/* 用户确认 2026-11-14：GPIO11/12（GPIO Matrix 任意映射）。
   TX=11 -> F407 USART2_RX(PD6)，RX=12 <- F407 USART2_TX(PD5) */
#define PEO_PM_ESP_UART        UART_NUM_1
#define PEO_PM_ESP_TX          GPIO_NUM_11
#define PEO_PM_ESP_RX          GPIO_NUM_12
#define PEO_PM_ESP_BAUDRATE    (921600U)

/* ============================================================================
   ESP32-S3 <-> ESP32-C3 UART 链路（c3_link 组件使用，PROTOCOL.md §1.4）
   ----------------------------------------------------------------------------
   引脚占位（待用户确认接线后修改；C3 端对应宏在 peo_rover_esp32c3 工程）：
   S3 UART_NUM_2 GPIO17(TX)/GPIO18(RX) <-> C3 UART1 GPIO5(RX)/GPIO4(TX)。
   波特率与 C3 端 PEO_PM_C3_BAUDRATE(115200) 一致。
   ============================================================================ */
#define PEO_PM_C3_UART      UART_NUM_2
#define PEO_PM_C3_TX        GPIO_NUM_17   /* S3 TX -> C3 RX（当前单向链路，TX 预留） */
#define PEO_PM_C3_RX        GPIO_NUM_18   /* S3 RX <- C3 TX */
#define PEO_PM_C3_BAUDRATE  (115200U)

#endif /* __PEO_PINMAP_H__ */
