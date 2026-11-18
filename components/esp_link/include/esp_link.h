#ifndef __ESP_LINK_H__
#define __ESP_LINK_H__

/**
 * @file    esp_link.h
 * @brief   PEO Rover - ESP32-S3 <-> F407 串口链路（esp_link 帧协议）
 *
 * 帧格式（小端，与 F407 COMPONENTS/ESP_LINK 完全一致）：
 *   SOF(2) + LEN(1) + PAYLOAD(LEN) + CRC16(2)
 *
 *   方向1（F407 -> ESP32，~50Hz）：PAYLOAD = SensorData_t 112B
 *     -> RX 任务逐字节解析，CRC 校验后最新快照入 esp_link_sensor_queue。
 *   方向2（ESP32 -> F407）：PAYLOAD = 命令帧（cmd_id + param，每帧一条）
 *     -> 生产者（cmd_parser）经封装接口入 esp_link_cmd_queue，
 *        TX 任务打包发送，每帧后停顿 >= 1 字节时间触发 F407 USART2 IDLE 组帧。
 *
 * CRC16-CCITT：多项式 0x1021，初始 0xFFFF，低字节在前。
 *
 * 队列创建约定：全部队列由各自组件的 xxx_queues_init() 在 app_main 里、
 * 任务创建之前统一创建（跨核绑定时优先级不保证时序——core1 消费者可能在
 * core0 创建者运行前抢跑）。任务内部不再创建队列。
 */

#include <stdint.h>
#include <stdbool.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

/* ============================================================================
   帧常量（与 F407 esp_link.h 对齐）
   ============================================================================ */
#define ESP_LINK_SOF0           0xAA
#define ESP_LINK_SOF1           0x55
#define ESP_LINK_FRAME_OVH      5       /* SOF(2) + LEN(1) + CRC(2) */
#define ESP_LINK_MAX_PAYLOAD    64      /* F407 command RX limit; NOT telemetry */
#define ESP_LINK_RX_PAYLOAD_SIZE 112U    /* F407 SensorData_t wire size */
#define ESP_LINK_STALE_MS        500U    /* 50Hz 上报下约 25 帧；按本地接收年龄判断 */

/* ============================================================================
   命令 ID（与 F407 esp_link.h 一致）
   ============================================================================ */
#define ESP_CMD_LED_TOGGLE      0x01
#define ESP_CMD_SENSOR_RESET    0x02
#define ESP_CMD_MOTOR           0x03    /* motor(1) + speed(2, int16 LE) */
#define ESP_CMD_PID             0x04    /* kp(4, f32 LE) + ki(4, f32 LE) */
#define ESP_CMD_MOTOR_CLOSED    0x05    /* motor(1) + speed_d100(4, int32 LE) */
#define ESP_CMD_STOP_ALL        0x06
#define ESP_CMD_ODOMETER_RESET  0x07
#define ESP_CMD_SYS_RESET       0x08    /* 全车软复位：F407 NVIC_SystemReset()（2026-11-16 定案，
                                          PROTOCOL §1.3；上位机 SDK 检测 fresh 恢复沿重发 PID/CAM_RES） */

#define ESP_CMD_PARAM_MAX       8

/* ============================================================================
   结构体
   ============================================================================ */

/** 命令帧（TX 队列元素） */
typedef struct
{
    uint8_t cmd_id;
    uint8_t param_len;
    uint8_t param[ESP_CMD_PARAM_MAX];
    uint32_t generation;          /* local safety epoch; never serialized */
} esp_link_cmd_t;

/**
 * SensorData_t —— 与 F407 SensorData_t / PROTOCOL.md §1.2 偏移表逐字节对齐。
 * 112 字节（packed，含 3+2 字节 padding）。
 */
typedef struct __attribute__((packed))
{
    uint16_t dist_front_mm;         /* 前测距 mm */
    uint16_t dist_left_mm;          /* 左测距 mm */
    uint16_t dist_right_mm;         /* 右测距 mm */
    uint8_t  dist_front_status;     /* 0=有效 */
    uint8_t  dist_left_status;
    uint8_t  dist_right_status;
    uint8_t  pad1[3];               /* 对齐 padding */
    uint32_t dist_timestamp_ms;
    float    quat_w, quat_x, quat_y, quat_z;   /* MPU6050 四元数 */
    float    accel_x, accel_y, accel_z;        /* 加速度 g */
    uint32_t mpu_timestamp_ms;
    int32_t  speed_d100[4];         /* 四轮速度 0.01 cm/s，正=前进 */
    int32_t  odometer[4];           /* 四轮里程计数 */
    uint32_t ctrl_timestamp_ms;
    int16_t  current_mA[4];         /* 四轮电流 mA（有符号） */
    uint16_t bus_voltage_mV[4];     /* 四路总线电压 mV */
    uint32_t ina_timestamp_ms;
    uint32_t sys_tick_ms;
    uint8_t  last_cmd_id;           /* 最近命令 ID（回显） */
    uint8_t  last_cmd_status;       /* 0=OK，非0=错误码 */
    uint8_t  pad2[2];               /* 尾部对齐 padding */
} esp_link_sensor_t;

/** 本地快照信封（仅用于队列；不参与 112B SensorData_t 线协议）。 */
typedef struct
{
    esp_link_sensor_t sensor;
    uint32_t received_ms;         /* FreeRTOS 单调毫秒时间，消费者须使用相同时间基准 */
    uint32_t seq;                 /* RX 单生产者有效帧序号，自增并允许 uint32_t 回绕 */
} esp_link_snapshot_t;

/* ============================================================================
   任务入口（app_main 创建；队列已统一创建，RX 任务负责 UART 初始化）
   ============================================================================ */
void esp_link_rx_task(void *arg);   /* UART 初始化 + 传感器帧接收解析 */
void esp_link_tx_task(void *arg);   /* 命令队列消费 + 帧间停顿发送 */

/** 队列初始化（app_main 里、创建任务之前调用；幂等） */
void esp_link_queues_init(void);

/* ============================================================================
   队列句柄（app_main 统一建队；命令生产者使用发送封装，快照消费者 peek）
   ============================================================================ */
extern QueueHandle_t esp_link_cmd_queue;      /* 入：命令帧（cmd_parser 经封装生产） */
extern QueueHandle_t esp_link_sensor_queue;   /* 出：esp_link_snapshot_t（1 深 overwrite；所有消费者 peek） */

/** Safety barrier: STOP is latched independently of the ordinary command queue.
 * TX commits a normal frame at its allowed/generation check under the safety lock.
 * Once STOP is latched, no new normal frame can commit until an explicit resume.
 * At most one previously committed old frame, possibly not yet physically sent,
 * can precede STOP; the single TX task preserves their UART order. UART calls are
 * outside the safety lock. Resume requires the observed generation and a transmitted STOP.
 * Caller must require deliberate fresh input, never automatically resume old work.
 */
void esp_link_request_stop(void);
/** 启动健康故障：锁 STOP 且永久禁止 resume；普通输入/STOP 不可清除，仅重启恢复。 */
void esp_link_fatal_fault(void);
uint32_t esp_link_command_generation(void);
bool esp_link_resume_commands(uint32_t observed_generation);
/** Enqueue only if the producer's captured epoch is still current. */
bool esp_link_send_command_for_generation(uint8_t cmd_id, const uint8_t *param,
                                          uint8_t param_len, uint32_t generation);

#endif /* __ESP_LINK_H__ */
