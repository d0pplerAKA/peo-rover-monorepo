/**
 ******************************************************************************
 * @file    esp_link.h
 * @brief   PEO Rover - ESP32 link (USART2) 帧格式 / 手柄 / 命令 定义
 *
 * 链路：F407 <-> ESP32-S3，一套 USART2 RX/TX 全双工（PD5=TX / PD6=RX, AF7）。
 *
 * 帧格式（小端）：
 *   SOF(2) + LEN(1) + PAYLOAD(LEN) + CRC16(2)
 *
 *   方向1（F407 -> ESP32）：PAYLOAD = SensorData_t（持续上报）。
 *   方向2（ESP32 -> F407）：PAYLOAD = GamepadData_t(20B 固定) + 命令(可选变长)。
 *
 * CRC16-CCITT：多项式 0x1021，初始 0xFFFF，覆盖 SOF~PAYLOAD 全部字节。
 ******************************************************************************
 */
#ifndef __ESP_LINK_H__
#define __ESP_LINK_H__

#include "stm32f4xx.h"
#include "PEO_PINMAP.H"

/* ============================================================================
   帧格式常量
   ============================================================================ */
#define ESP_LINK_SOF0           0xAA    /* 帧头高字节 */
#define ESP_LINK_SOF1           0x55    /* 帧头低字节 */
#define ESP_LINK_MAX_PAYLOAD    64      /* payload 上限：手柄20 + 命令 <= 44 */
#define ESP_LINK_FRAME_OVH      5       /* SOF(2) + LEN(1) + CRC(2) */

/* ============================================================================
   命令 ID（ESP32 -> F407，追加在手柄数据之后）
   ----------------------------------------------------------------------------
   命令体格式：cmd_id(1) + param(变长)。param 长度由 cmd_id 约定：
     LED_TOGGLE    无参数
     SENSOR_RESET  无参数
     MOTOR         motor(1) + speed(2, int16)         开环直驱
     PID           kp(4, float) + ki(4, float)        共享增益
     MOTOR_CLOSED  motor(1) + speed_d100(4, int32)    闭环目标速度
   ============================================================================ */
#define ESP_CMD_LED_TOGGLE      0x01
#define ESP_CMD_SENSOR_RESET    0x02
#define ESP_CMD_MOTOR           0x03
#define ESP_CMD_PID             0x04
#define ESP_CMD_MOTOR_CLOSED    0x05

/* 命令执行结果码（回显到 sensorhub.last_cmd_status） */
#define ESP_CMD_OK              0x00    /* 成功 */
#define ESP_CMD_ERR_INVALID     0x01    /* 未知命令 ID */
#define ESP_CMD_ERR_BADLEN      0x02    /* 参数长度错误 */
#define ESP_CMD_ERR_BADMOTOR    0x03    /* 电机号越界 */

/* ============================================================================
   手柄数据包 GamepadData_t（ESP32 -> F407，固定 20 字节）
   ----------------------------------------------------------------------------
   无论手柄是否连接都整包发送（未连接时 connected=0，其余为 0/默认值）。
   rumble_* 为「当前振动状态回显」——振动只由电脑触发（电脑 -> ESP32 -> 手柄），
   F407 不控制振动，这些字段仅回显。
   ============================================================================ */
typedef struct
{
    uint16_t joy_lx;        /* 左摇杆 X（0~65535） */
    uint16_t joy_ly;        /* 左摇杆 Y            */
    uint16_t joy_rx;        /* 右摇杆 X            */
    uint16_t joy_ry;        /* 右摇杆 Y            */
    uint16_t trigger_lt;    /* 左扳机              */
    uint16_t trigger_rt;    /* 右扳机              */
    uint16_t buttons;       /* 按键位图（见下）    */
    uint8_t  connected;     /* 0=未连接 1=已连接   */
    uint8_t  battery;       /* 电量 0~100 %        */
    uint8_t  rumble_left;   /* 左粗振动电机        */
    uint8_t  rumble_right;  /* 右细振动电机        */
    uint8_t  rumble_lt;     /* 左扳机振动          */
    uint8_t  rumble_rt;     /* 右扳机振动          */
} GamepadData_t;

/* 按键位图 bit 定义（buttons 字段） */
#define GAMEPAD_BTN_A          ((uint16_t)(1U << 0))
#define GAMEPAD_BTN_B          ((uint16_t)(1U << 1))
#define GAMEPAD_BTN_X          ((uint16_t)(1U << 2))
#define GAMEPAD_BTN_Y          ((uint16_t)(1U << 3))
#define GAMEPAD_BTN_LB         ((uint16_t)(1U << 4))
#define GAMEPAD_BTN_RB         ((uint16_t)(1U << 5))
#define GAMEPAD_BTN_START      ((uint16_t)(1U << 6))
#define GAMEPAD_BTN_SELECT     ((uint16_t)(1U << 7))
#define GAMEPAD_BTN_SHARE      ((uint16_t)(1U << 8))
#define GAMEPAD_BTN_LS         ((uint16_t)(1U << 9))
#define GAMEPAD_BTN_RS         ((uint16_t)(1U << 10))
#define GAMEPAD_DPAD_UP        ((uint16_t)(1U << 11))
#define GAMEPAD_DPAD_DOWN      ((uint16_t)(1U << 12))
#define GAMEPAD_DPAD_LEFT      ((uint16_t)(1U << 13))
#define GAMEPAD_DPAD_RIGHT     ((uint16_t)(1U << 14))

/* ============================================================================
   PUBLIC API
   ============================================================================ */

/**
 * @brief  初始化 USART2（PD5/PD6，RX 中断）。时钟/NVIC 由 RCC/NVIC 组件负责。
 */
void ESP_LINK_Init(void);

/**
 * @brief  启动 TIM14 上报定时器：按 SENSOR_RATE_REPORT_HZ 频率中断触发发送。
 * @note   在 ESP_LINK_Init() 之后调用；发送用 DMA 非阻塞，不卡控制循环。
 */
void ESP_LINK_StartReport(void);

/**
 * @brief  计算 CRC16-CCITT（多项式 0x1021，初始 0xFFFF）。供两端共用。
 */
uint16_t ESP_LINK_CRC16(const uint8_t *data, uint16_t len);

#endif /* __ESP_LINK_H__ */
