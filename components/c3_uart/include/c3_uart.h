#ifndef __C3_UART_H__
#define __C3_UART_H__

/**
 * @file    c3_uart.h
 * @brief   PEO Rover - ESP32-C3 UART 链路（C3 -> S3 手柄状态帧，PROTOCOL.md §1.4）
 *
 * 帧格式与 esp_link 同构：SOF(0xAA 0x55) + LEN(1) + PAYLOAD + CRC16-CCITT(2)。
 * 小端；CRC 覆盖 SOF~PAYLOAD，帧尾低字节在前。
 *
 * PAYLOAD = type(1B) + body；当前唯一类型 GAMEPAD_STATE(0x10)：
 *   body = c3_gamepad_payload_t（24B）→ payload 25B，帧总长 30B。
 * 50Hz 周期发送，兼作 C3 存活心跳（S3 端 500ms 无帧 = C3 失联 → 锁 STOP）。
 */

#include <stdint.h>
#include <stdbool.h>

#define C3_FRAME_SOF0   0xAA
#define C3_FRAME_SOF1   0x55

#define C3_TYPE_GAMEPAD_STATE   0x10

/** GAMEPAD_STATE body（24B，与 PROTOCOL.md §1.4 偏移表逐字节一致） */
typedef struct __attribute__((packed))
{
    uint32_t seq;        /* 有效 HID 通知序号（跨重连递增；电池/短报告不递增） */
    uint32_t disc_gen;   /* 断连代次 */
    uint8_t  connected;  /* 0=未连接 1=已连接且有数据 */
    uint8_t  battery;    /* 电量 0~100 */
    uint16_t joy_lx;     /* 左摇杆水平 0~65535，中值 32768 */
    uint16_t joy_ly;     /* 左摇杆垂直 */
    uint16_t joy_rx;     /* 右摇杆水平 */
    uint16_t joy_ry;     /* 右摇杆垂直 */
    uint16_t trigger_lt; /* 左扳机 0~1023 */
    uint16_t trigger_rt; /* 右扳机 0~1023 */
    uint16_t buttons;    /* 按键位图（XB_BTN_*，xbox_service.h） */
} c3_gamepad_payload_t;

/** UART 初始化（8-N-1，无流控，波特率/引脚见 PEO_PINMAP.h） */
void c3_uart_init(void);

/** 打包发送一条 GAMEPAD_STATE 状态帧；返回 false = 发送失败（FIFO 未收完/超时） */
bool c3_uart_send_gamepad(const c3_gamepad_payload_t *p);

#endif /* __C3_UART_H__ */
