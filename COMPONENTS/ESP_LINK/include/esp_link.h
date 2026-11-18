/**
 ******************************************************************************
 * @file    esp_link.h
 * @brief   PEO Rover - ESP32 link (USART2) 帧格式 / 命令 定义
 *
 * 链路：F407 <-> ESP32-S3，一套 USART2 RX/TX 全双工（PD5=TX / PD6=RX, AF7）。
 *
 * 帧格式（小端）：
 *   SOF(2) + LEN(1) + PAYLOAD(LEN) + CRC16(2)
 *
 *   方向1（F407 -> ESP32）：PAYLOAD = SensorData_t（持续上报）。
 *   方向2（ESP32 -> F407）：PAYLOAD = 命令帧（cmd_id + param，每帧一条）。
 *
 * CRC16-CCITT：多项式 0x1021，初始 0xFFFF，覆盖 SOF~PAYLOAD 全部字节。
 *
 * ESP32 端（发送方）打包约定：
 *   1. 每帧恰好一条命令：cmd_id(1B) + param(变长，长度由 cmd_id 约定)。
 *   2. 手柄数据不再下发 F407：Xbox 手柄在 ESP32 端本地解析（BLE HID），
 *      映射成 MOTOR / MOTOR_CLOSED / STOP_ALL 等命令帧后下发。
 *   3. 全帧 payload 后算 CRC16-CCITT（覆盖 SOF~PAYLOAD，不含 CRC 本身），
 *      低字节在前附在末尾。
 *   4. 发完一帧后停顿（>= 1 字节时间）触发 F407 的 USART2 IDLE 中断组帧。
 ******************************************************************************
 */
#ifndef __ESP_LINK_H__
#define __ESP_LINK_H__

#include "stm32f4xx.h"
#include "COMPONENTS/PEO_PINMAP.H"

/* ============================================================================
   帧格式常量
   ============================================================================ */
#define ESP_LINK_SOF0           0xAA    /* 帧头高字节 */
#define ESP_LINK_SOF1           0x55    /* 帧头低字节 */
#define ESP_LINK_MAX_PAYLOAD    64      /* payload 上限：命令 <= 9B，留余量 */
#define ESP_LINK_FRAME_OVH      5       /* SOF(2) + LEN(1) + CRC(2) */

/* ============================================================================
   命令 ID（ESP32 -> F407，帧 payload 的全部内容）
   ----------------------------------------------------------------------------
   命令体格式：cmd_id(1) + param(变长)。param 长度由 cmd_id 约定：
     LED_TOGGLE    无参数
     SENSOR_RESET  无参数
     MOTOR         motor(1) + speed(2, int16)         开环直驱
     PID           kp(4, float) + ki(4, float)        共享增益
     MOTOR_CLOSED  motor(1) + speed_d100(4, int32)    闭环目标速度
     STOP_ALL      无参数                            急停
     ODOMETER_RESET 无参数                           清零四轮里程
   ============================================================================ */
#define ESP_CMD_LED_TOGGLE      0x01
#define ESP_CMD_SENSOR_RESET    0x02
#define ESP_CMD_MOTOR           0x03
#define ESP_CMD_PID             0x04
#define ESP_CMD_MOTOR_CLOSED    0x05
#define ESP_CMD_STOP_ALL        0x06
#define ESP_CMD_ODOMETER_RESET  0x07
#define ESP_CMD_SYS_RESET       0x08    /* 全车软复位：NVIC_SystemReset()（2026-11-16 定案） */

/* 命令执行结果码（回显到 sensorhub.last_cmd_status） */
#define ESP_CMD_OK              0x00    /* 成功 */
#define ESP_CMD_ERR_INVALID     0x01    /* 未知命令 ID */
#define ESP_CMD_ERR_BADLEN      0x02    /* 参数长度错误 */
#define ESP_CMD_ERR_BADMOTOR    0x03    /* 电机号越界 */

/* ============================================================================
   命令结构体（命令队列元素）
   ----------------------------------------------------------------------------
   上位机 / 手柄映射命令经 ESP32 转成「cmd_id + param」帧发来，F407 在
   USART2 IDLE 中断里解析后入队，main 主循环出队执行（中断不执行业务）。
   ============================================================================ */
#define ESP_CMD_MAX_PARAM       8   /* 最大参数长度（PID 的 kp+ki = 8B） */

typedef struct
{
    uint8_t id;              /* 命令 ID（ESP_CMD_*） */
    uint8_t param_len;       /* 参数长度（字节）    */
    uint8_t param[ESP_CMD_MAX_PARAM];
} ESP_Cmd_t;

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

/* ============================================================================
   命令队列 API（中断入队，主循环出队执行）
   ============================================================================ */

/**
 * @brief  命令入队（在 USART2 IDLE 中断里调，不执行业务）。
 * @param  id         命令 ID（ESP_CMD_*）。
 * @param  param      参数指针（可为 NULL 当 param_len=0）。
 * @param  param_len  参数字节数（<= ESP_CMD_MAX_PARAM）。
 * @return 1 = 入队成功；0 = 队满或参数超长（丢弃）。
 */
uint8_t ESP_LINK_PushCommand(uint8_t id, const uint8_t *param, uint8_t param_len);

/**
 * @brief  命令出队（在 main 主循环调）。
 * @param  out  输出命令结构体。
 * @return 1 = 取到一条命令；0 = 队列空。
 */
uint8_t ESP_LINK_PopCommand(ESP_Cmd_t *out);

/**
 * @brief  执行一条命令并回显 ACK（在 main 主循环调）。
 * @param  cmd  要执行的命令（来自 ESP_LINK_PopCommand）。
 */
void ESP_LINK_ExecCommand(const ESP_Cmd_t *cmd);

#endif /* __ESP_LINK_H__ */
