#ifndef __MACRO_ENGINE_H__
#define __MACRO_ENGINE_H__

/**
 * @file    macro_engine.h
 * @brief   PEO Rover - 离线宏引擎（PROTOCOL.md §2.2 的 act/wait/goto 执行器）
 *
 * macro_task（优先级 4）：单槽位状态机，执行 cmd_parser 预编译的宏程序：
 *   act  : 进入步骤时发一次命令帧；duration_ms > 0 计时后续行，= 0 立即下一步
 *          （命令持续生效，配合 wait 条件步骤）
 *   wait : every 10ms, peek the shared latest sensor snapshot without removing it;
 *          telemetry also peeks. Condition or timeout advances the step.
 *   goto : 跳转（可循环）；越界 = 宏结束（PROTOCOL §2.2）
 *
 * STOP from any source invalidates the active/queued generation. Abort/disconnect
 * callers latch STOP directly, even when queues are full. Replacement START waits
 * for its STOP to reach UART before resuming. STOP_ALL inside an ACT ends the macro.
 *
 * 进度：xQueueOverwrite 到 macro_progress_queue（1 深），telemetry 附加进
 * 50Hz JSON 流（只写 running/done/aborted，不写 idle）。
 */

#include <stdint.h>
#include <stdbool.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#define MACRO_MAX_STEPS     64      /* 单宏 steps 数组上限（用户拍板 2026-11-14） */
#define MACRO_PARAM_MAX     8       /* 与 ESP_CMD_PARAM_MAX 一致 */

/* ============================================================================
   步骤结构体（cmd_parser 解析 JSON 后填充，堆分配，所有权移交宏引擎）
   ============================================================================ */
typedef enum { MACRO_STEP_ACT = 0, MACRO_STEP_WAIT, MACRO_STEP_GOTO } macro_step_type_t;

typedef struct
{
    macro_step_type_t type;
    union
    {
        struct                  /* ACT */
        {
            uint8_t  cmd_id;
            uint8_t  param_len;
            uint8_t  param[MACRO_PARAM_MAX];
            uint32_t duration_ms;
        } act;
        struct                  /* WAIT */
        {
            uint8_t  field;     /* macro_field_t */
            uint8_t  op;        /* macro_op_t   */
            double   value;
            uint32_t timeout_ms;
        } wait;
        struct                  /* GOTO */
        {
            int16_t target;     /* 0-based；越界 = 宏结束 */
        } got;
    } u;
} macro_step_t;

typedef struct
{
    uint16_t id;
    uint16_t count;             /* 实际步数 */
    macro_step_t steps[MACRO_MAX_STEPS];
} macro_prog_t;

/* ============================================================================
   控制消息（多生产者：cmd_parser / esp_net）
   ============================================================================ */
typedef enum
{
    MACRO_CTRL_START = 0,       /* 启动新宏（prog 所有权移交，引擎 free） */
    MACRO_CTRL_ABORT,           /* 上位机 macro_abort */
    MACRO_CTRL_DISCONNECT,      /* TCP 断连（无论宏是否在跑都发 STOP_ALL） */
} macro_ctrl_type_t;

typedef struct
{
    macro_ctrl_type_t type;
    macro_prog_t     *prog;     /* 仅 START 有效 */
    uint32_t generation;        /* START cannot cross a STOP/disconnect boundary. */
} macro_ctrl_t;

/* ============================================================================
   进度快照（引擎 -> telemetry）
   ============================================================================ */
typedef enum
{
    MACRO_STATE_IDLE = 0,
    MACRO_STATE_RUNNING,
    MACRO_STATE_DONE,
    MACRO_STATE_ABORTED,
} macro_state_t;

typedef struct
{
    uint16_t id;
    uint16_t step;              /* 当前步（0-based） */
    uint16_t total;
    uint8_t  state;             /* macro_state_t */
} macro_progress_t;

/* ============================================================================
   传感器字段 / 比较符（wait 步骤；名字表与 PROTOCOL §2.2 field 列表一致）
   ============================================================================ */
typedef enum
{
    MACRO_FIELD_DIST_FRONT_MM = 0,
    MACRO_FIELD_DIST_LEFT_MM,
    MACRO_FIELD_DIST_RIGHT_MM,
    MACRO_FIELD_DIST_FRONT_STATUS,
    MACRO_FIELD_DIST_LEFT_STATUS,
    MACRO_FIELD_DIST_RIGHT_STATUS,
    MACRO_FIELD_QUAT_W,
    MACRO_FIELD_QUAT_X,
    MACRO_FIELD_QUAT_Y,
    MACRO_FIELD_QUAT_Z,
    MACRO_FIELD_ACCEL_X,
    MACRO_FIELD_ACCEL_Y,
    MACRO_FIELD_ACCEL_Z,
    MACRO_FIELD_SPEED_D100_0,
    MACRO_FIELD_SPEED_D100_1,
    MACRO_FIELD_SPEED_D100_2,
    MACRO_FIELD_SPEED_D100_3,
    MACRO_FIELD_ODOMETER_0,
    MACRO_FIELD_ODOMETER_1,
    MACRO_FIELD_ODOMETER_2,
    MACRO_FIELD_ODOMETER_3,
    MACRO_FIELD_CURRENT_MA_0,
    MACRO_FIELD_CURRENT_MA_1,
    MACRO_FIELD_CURRENT_MA_2,
    MACRO_FIELD_CURRENT_MA_3,
    MACRO_FIELD_BUS_VOLTAGE_MV_0,
    MACRO_FIELD_BUS_VOLTAGE_MV_1,
    MACRO_FIELD_BUS_VOLTAGE_MV_2,
    MACRO_FIELD_BUS_VOLTAGE_MV_3,
    MACRO_FIELD_SYS_TICK_MS,
    MACRO_FIELD_COUNT,
} macro_field_t;

typedef enum
{
    MACRO_OP_LT = 0,
    MACRO_OP_LE,
    MACRO_OP_GT,
    MACRO_OP_GE,
    MACRO_OP_EQ,
    MACRO_OP_NE,
} macro_op_t;

/* ============================================================================
   队列 + 任务入口
   ============================================================================ */
extern QueueHandle_t macro_ctrl_queue;      /* START/ABORT/DISCONNECT -> 引擎 */
extern QueueHandle_t macro_progress_queue;  /* 引擎 -> telemetry（1 深 overwrite） */

/** 队列初始化（app_main 里、创建任务之前调用） */
void macro_queues_init(void);

void macro_task(void *arg);

/** 名字解析（cmd_parser 解析/校验宏 JSON 用） */
bool macro_field_from_name(const char *name, uint8_t *field);
bool macro_op_from_name(const char *name, uint8_t *op);

#endif /* __MACRO_ENGINE_H__ */
