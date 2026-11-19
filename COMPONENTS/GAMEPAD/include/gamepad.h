/**
 ******************************************************************************
 * @file    gamepad.h
 * @brief   PEO Rover - GAMEPAD component public API (Xbox 手柄手操链路)
 *
 * 手操链路（方案 A，物理隔离于 WiFi/自主链路）：
 *   Xbox 手柄 --BLE--> ESP32-C3 --USART3(PD8/PD9, 115200)--> F407
 *
 * C3 发原始手柄状态帧（100Hz），F407 端解析 + 锁仲裁 + 摇杆映射。
 * 摇杆原始值保留到 F407，映射（死区/曲线/斜坡）集中在 F407，手感可调、
 * 控制最细腻，且调手感不必重烧 C3。
 *
 * 锁（控制权）：lock=0 手操（手柄控制），lock=1 自主（上位机控制）。
 * 锁状态由 C3 长按锁键（Share）翻转，随状态帧上报，F407 据此仲裁。
 ******************************************************************************
 */
#ifndef __GAMEPAD_H__
#define __GAMEPAD_H__

#include "stm32f4xx.h"
#include "COMPONENTS/PEO_PINMAP.H"

/* ============================================================================
   帧格式（与 C3 c3_uart 逐字节一致）
   ----------------------------------------------------------------------------
   SOF(0xAA 0x55) + LEN(1=27) + type(0x10) + body(26B) + CRC16-CCITT(2, 低字节在前)
   帧总长 32B；CRC 覆盖 SOF~PAYLOAD（30 字节），标准向量 crc("123456789")==0x29B1。
   ============================================================================ */
#define GAMEPAD_SOF0          (0xAAU)
#define GAMEPAD_SOF1          (0x55U)
#define GAMEPAD_TYPE_STATE    (0x10U)
#define GAMEPAD_BODY_LEN      (26U)     /* gamepad_payload_t 大小 */
#define GAMEPAD_PAYLOAD_LEN   (27U)     /* type(1) + body(26) */
#define GAMEPAD_FRAME_LEN     (32U)     /* SOF2 + LEN1 + payload27 + CRC2 */

/* ============================================================================
   配置
   ============================================================================ */
#define GAMEPAD_BAUDRATE      (115200U) /* 与 C3 PEO_PM_C3_BAUDRATE 一致 */
#define GAMEPAD_STALE_MS      (500U)    /* C3 失联判定（>此值无有效帧 -> 停车） */
#define GAMEPAD_DEADZONE      (500)     /* 摇杆死区（中值 32768，±500 内归零） */
#define GAMEPAD_SPEED_MAX     (4424)    /* 满速 0.01 cm/s（≈44.24 cm/s） */

/* ============================================================================
   锁状态
   ============================================================================ */
#define GAMEPAD_LOCK_MANUAL   (0U)      /* 锁关闭 = 手操（手柄控制） */
#define GAMEPAD_LOCK_AUTO     (1U)      /* 锁开启 = 自主（上位机控制） */

/* ============================================================================
   按键位图（与 C3 xbox_service.h XB_BTN_* 位序一致）
   ============================================================================ */
#define GAMEPAD_BTN_XBOX      ((uint16_t)(1U << 15))   /* 急停键（双源，永远生效） */

/* ============================================================================
   手柄 payload（26B packed，与 C3 c3_gamepad_payload_t 逐字节一致）
   ----------------------------------------------------------------------------
   AC5(armcc) 用 __packed 关键字；AC6/GCC 用 __attribute__((packed))。
   ============================================================================ */
typedef __packed struct
{
    uint32_t seq;            /* 通知序列号 */
    uint32_t disc_gen;       /* 断连代数 */
    uint8_t  connected;      /* 手柄是否连接 */
    uint8_t  battery;        /* 电量 */
    uint8_t  input_fresh;    /* 输入新鲜度（5s 内有 HID 通知） */
    uint16_t joy_lx;         /* 左摇杆水平 */
    uint16_t joy_ly;         /* 左摇杆垂直（前进/后退油门） */
    uint16_t joy_rx;         /* 右摇杆水平（单边差速转向） */
    uint16_t joy_ry;         /* 右摇杆垂直 */
    uint16_t trigger_lt;     /* 左扳机 */
    uint16_t trigger_rt;     /* 右扳机 */
    uint16_t buttons;        /* 按键位图 */
    uint8_t  lock;           /* 控制权锁：0=手操，1=自主 */
} gamepad_payload_t;

/* ============================================================================
   PUBLIC API
   ============================================================================ */

/** 初始化 USART3（PD8/PD9, AF7）+ RXNE 逐字节中断。 */
void GAMEPAD_Init(void);

/** 读当前锁状态：GAMEPAD_LOCK_MANUAL(0)=手操 / GAMEPAD_LOCK_AUTO(1)=自主。 */
uint8_t GAMEPAD_GetLock(void);

/** 手柄是否已连接。 */
uint8_t GAMEPAD_IsConnected(void);

/**
 * @brief  失联检测 + 锁仲裁 + 摇杆映射（main 主循环调，~100Hz 节拍）。
 * @note   锁关闭（手操）时把摇杆映射成四轮目标速度；锁开启（自主）时跳过
 *         （上位机命令接管）。急停键（XBOX）双源永远生效，越过锁。
 */
void GAMEPAD_Process(void);

#endif /* __GAMEPAD_H__ */
