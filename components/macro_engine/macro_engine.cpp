/**
 * @file    macro_engine.cpp
 * @brief   PEO Rover - 离线宏引擎实现（act/wait/goto 状态机，10ms 节拍）
 */

#include "macro_engine.h"
#include "esp_link.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "macro";

QueueHandle_t macro_ctrl_queue     = NULL;
QueueHandle_t macro_progress_queue = NULL;

/* ============================================================================
   1. 字段/比较符名字表（PROTOCOL §2.2 field 列表）
   ============================================================================ */
static const struct { const char *name; uint8_t field; } k_field_names[] = {
    { "dist_front_mm",     MACRO_FIELD_DIST_FRONT_MM },
    { "dist_left_mm",      MACRO_FIELD_DIST_LEFT_MM },
    { "dist_right_mm",     MACRO_FIELD_DIST_RIGHT_MM },
    { "dist_front_status", MACRO_FIELD_DIST_FRONT_STATUS },
    { "dist_left_status",  MACRO_FIELD_DIST_LEFT_STATUS },
    { "dist_right_status", MACRO_FIELD_DIST_RIGHT_STATUS },
    { "quat_w",            MACRO_FIELD_QUAT_W },
    { "quat_x",            MACRO_FIELD_QUAT_X },
    { "quat_y",            MACRO_FIELD_QUAT_Y },
    { "quat_z",            MACRO_FIELD_QUAT_Z },
    { "accel_x",           MACRO_FIELD_ACCEL_X },
    { "accel_y",           MACRO_FIELD_ACCEL_Y },
    { "accel_z",           MACRO_FIELD_ACCEL_Z },
    { "speed_d100_0",      MACRO_FIELD_SPEED_D100_0 },
    { "speed_d100_1",      MACRO_FIELD_SPEED_D100_1 },
    { "speed_d100_2",      MACRO_FIELD_SPEED_D100_2 },
    { "speed_d100_3",      MACRO_FIELD_SPEED_D100_3 },
    { "odometer_0",        MACRO_FIELD_ODOMETER_0 },
    { "odometer_1",        MACRO_FIELD_ODOMETER_1 },
    { "odometer_2",        MACRO_FIELD_ODOMETER_2 },
    { "odometer_3",        MACRO_FIELD_ODOMETER_3 },
    { "current_mA_0",      MACRO_FIELD_CURRENT_MA_0 },
    { "current_mA_1",      MACRO_FIELD_CURRENT_MA_1 },
    { "current_mA_2",      MACRO_FIELD_CURRENT_MA_2 },
    { "current_mA_3",      MACRO_FIELD_CURRENT_MA_3 },
    { "bus_voltage_mV_0",  MACRO_FIELD_BUS_VOLTAGE_MV_0 },
    { "bus_voltage_mV_1",  MACRO_FIELD_BUS_VOLTAGE_MV_1 },
    { "bus_voltage_mV_2",  MACRO_FIELD_BUS_VOLTAGE_MV_2 },
    { "bus_voltage_mV_3",  MACRO_FIELD_BUS_VOLTAGE_MV_3 },
    { "sys_tick_ms",       MACRO_FIELD_SYS_TICK_MS },
};

static const struct { const char *name; uint8_t op; } k_op_names[] = {
    { "<",  MACRO_OP_LT },
    { "<=", MACRO_OP_LE },
    { ">",  MACRO_OP_GT },
    { ">=", MACRO_OP_GE },
    { "==", MACRO_OP_EQ },
    { "!=", MACRO_OP_NE },
};

bool macro_field_from_name(const char *name, uint8_t *field)
{
    for (unsigned i = 0; i < sizeof(k_field_names) / sizeof(k_field_names[0]); i++)
    {
        if (strcmp(k_field_names[i].name, name) == 0)
        {
            *field = k_field_names[i].field;
            return true;
        }
    }
    return false;
}

bool macro_op_from_name(const char *name, uint8_t *op)
{
    for (unsigned i = 0; i < sizeof(k_op_names) / sizeof(k_op_names[0]); i++)
    {
        if (strcmp(k_op_names[i].name, name) == 0)
        {
            *op = k_op_names[i].op;
            return true;
        }
    }
    return false;
}

/** 读传感器字段值（统一 double，int32 字段转 double 精度无损） */
static double field_value(const esp_link_sensor_t *s, uint8_t field)
{
    switch (field)
    {
    case MACRO_FIELD_DIST_FRONT_MM:     return s->dist_front_mm;
    case MACRO_FIELD_DIST_LEFT_MM:      return s->dist_left_mm;
    case MACRO_FIELD_DIST_RIGHT_MM:     return s->dist_right_mm;
    case MACRO_FIELD_DIST_FRONT_STATUS: return s->dist_front_status;
    case MACRO_FIELD_DIST_LEFT_STATUS:  return s->dist_left_status;
    case MACRO_FIELD_DIST_RIGHT_STATUS: return s->dist_right_status;
    case MACRO_FIELD_QUAT_W:            return s->quat_w;
    case MACRO_FIELD_QUAT_X:            return s->quat_x;
    case MACRO_FIELD_QUAT_Y:            return s->quat_y;
    case MACRO_FIELD_QUAT_Z:            return s->quat_z;
    case MACRO_FIELD_ACCEL_X:           return s->accel_x;
    case MACRO_FIELD_ACCEL_Y:           return s->accel_y;
    case MACRO_FIELD_ACCEL_Z:           return s->accel_z;
    case MACRO_FIELD_SPEED_D100_0:      return s->speed_d100[0];
    case MACRO_FIELD_SPEED_D100_1:      return s->speed_d100[1];
    case MACRO_FIELD_SPEED_D100_2:      return s->speed_d100[2];
    case MACRO_FIELD_SPEED_D100_3:      return s->speed_d100[3];
    case MACRO_FIELD_ODOMETER_0:        return s->odometer[0];
    case MACRO_FIELD_ODOMETER_1:        return s->odometer[1];
    case MACRO_FIELD_ODOMETER_2:        return s->odometer[2];
    case MACRO_FIELD_ODOMETER_3:        return s->odometer[3];
    case MACRO_FIELD_CURRENT_MA_0:      return s->current_mA[0];
    case MACRO_FIELD_CURRENT_MA_1:      return s->current_mA[1];
    case MACRO_FIELD_CURRENT_MA_2:      return s->current_mA[2];
    case MACRO_FIELD_CURRENT_MA_3:      return s->current_mA[3];
    case MACRO_FIELD_BUS_VOLTAGE_MV_0:  return s->bus_voltage_mV[0];
    case MACRO_FIELD_BUS_VOLTAGE_MV_1:  return s->bus_voltage_mV[1];
    case MACRO_FIELD_BUS_VOLTAGE_MV_2:  return s->bus_voltage_mV[2];
    case MACRO_FIELD_BUS_VOLTAGE_MV_3:  return s->bus_voltage_mV[3];
    case MACRO_FIELD_SYS_TICK_MS:       return s->sys_tick_ms;
    default:                            return 0.0;
    }
}

/** 按 WAIT 字段选择 F407 子系统采样时间；与同帧 sys_tick_ms 使用同一时间基准。 */
static uint32_t field_subsystem_timestamp_ms(const esp_link_sensor_t *s, uint8_t field)
{
    switch (field)
    {
    case MACRO_FIELD_DIST_FRONT_MM:
    case MACRO_FIELD_DIST_LEFT_MM:
    case MACRO_FIELD_DIST_RIGHT_MM:
    case MACRO_FIELD_DIST_FRONT_STATUS:
    case MACRO_FIELD_DIST_LEFT_STATUS:
    case MACRO_FIELD_DIST_RIGHT_STATUS:
        return s->dist_timestamp_ms;
    case MACRO_FIELD_QUAT_W:
    case MACRO_FIELD_QUAT_X:
    case MACRO_FIELD_QUAT_Y:
    case MACRO_FIELD_QUAT_Z:
    case MACRO_FIELD_ACCEL_X:
    case MACRO_FIELD_ACCEL_Y:
    case MACRO_FIELD_ACCEL_Z:
        return s->mpu_timestamp_ms;
    case MACRO_FIELD_SPEED_D100_0:
    case MACRO_FIELD_SPEED_D100_1:
    case MACRO_FIELD_SPEED_D100_2:
    case MACRO_FIELD_SPEED_D100_3:
    case MACRO_FIELD_ODOMETER_0:
    case MACRO_FIELD_ODOMETER_1:
    case MACRO_FIELD_ODOMETER_2:
    case MACRO_FIELD_ODOMETER_3:
        return s->ctrl_timestamp_ms;
    case MACRO_FIELD_CURRENT_MA_0:
    case MACRO_FIELD_CURRENT_MA_1:
    case MACRO_FIELD_CURRENT_MA_2:
    case MACRO_FIELD_CURRENT_MA_3:
    case MACRO_FIELD_BUS_VOLTAGE_MV_0:
    case MACRO_FIELD_BUS_VOLTAGE_MV_1:
    case MACRO_FIELD_BUS_VOLTAGE_MV_2:
    case MACRO_FIELD_BUS_VOLTAGE_MV_3:
        return s->ina_timestamp_ms;
    case MACRO_FIELD_SYS_TICK_MS:
        return s->sys_tick_ms;
    default:
        return 0;
    }
}

static bool field_compare(double v, double ref, uint8_t op)
{
    switch (op)
    {
    case MACRO_OP_LT: return v <  ref;
    case MACRO_OP_LE: return v <= ref;
    case MACRO_OP_GT: return v >  ref;
    case MACRO_OP_GE: return v >= ref;
    case MACRO_OP_EQ: return v == ref;
    case MACRO_OP_NE: return v != ref;
    default:          return false;
    }
}

/* ============================================================================
   2. 引擎状态
   ============================================================================ */
#define MACRO_TICK_MS       10      /* 节拍 */

static macro_prog_t *s_cur    = NULL;
static uint16_t      s_pc     = 0;
static uint32_t      s_step_start_ms = 0; /* 步骤进入时的 FreeRTOS 单调毫秒时间 */
static bool          s_step_started = false; /* 不用时间戳/elapsed==0 判断首次进入 */
static uint8_t       s_state  = MACRO_STATE_IDLE;
static uint32_t      s_generation = 0;
static bool          s_resume_pending = false;
static bool          s_completion_pending = false;

/** 进度上报（只报 running/done/aborted；不写 idle，保留最后状态给 telemetry） */
static void report_progress(uint8_t state)
{
    macro_progress_t p;
    p.id    = (s_cur != NULL) ? s_cur->id : 0;
    p.step  = s_pc;
    p.total = (s_cur != NULL) ? s_cur->count : 0;
    p.state = state;
    xQueueOverwrite(macro_progress_queue, &p);
}

/** 中止/结束公共收尾：发 STOP_ALL + 释放程序 + 上报 */
static void finish_macro(uint8_t state, bool request_stop = true)
{
    if (request_stop) esp_link_request_stop();
    report_progress(state);  /* Capture id/pc/count before releasing the program. */
    if (s_cur != NULL)
    {
        free(s_cur);
        s_cur = NULL;
    }
    s_pc         = 0;
    s_step_start_ms = 0;
    s_step_started = false;
    s_resume_pending = false;
    s_completion_pending = false;
    s_state      = MACRO_STATE_IDLE;
}

void macro_queues_init(void)
{
    if (!macro_ctrl_queue) macro_ctrl_queue = xQueueCreate(4, sizeof(macro_ctrl_t));
    if (!macro_progress_queue) macro_progress_queue = xQueueCreate(1, sizeof(macro_progress_t));
    configASSERT(macro_ctrl_queue && macro_progress_queue);
}

void macro_task(void *arg)
{
    ESP_LOGI(TAG, "task up");

    while (1)
    {
        /* A STOP from any source invalidates a running or queued program. */
        if (s_cur && s_generation != esp_link_command_generation())
            finish_macro(MACRO_STATE_ABORTED, false);

        macro_ctrl_t c;
        for (int batch = 0; batch < 4 && xQueueReceive(macro_ctrl_queue, &c, 0) == pdTRUE; ++batch)
        {
            if (c.type != MACRO_CTRL_START)
            {
                if (c.generation == esp_link_command_generation())
                {
                    esp_link_request_stop();
                    if (s_cur) finish_macro(MACRO_STATE_ABORTED, false);
                }
                continue;
            }
            if (!c.prog || !c.prog->count || c.prog->count > MACRO_MAX_STEPS ||
                c.generation != esp_link_command_generation())
            {
                free(c.prog);
                continue;
            }
            if (s_cur)
            {
                finish_macro(MACRO_STATE_ABORTED);
                /* Only our replacement STOP may advance this START's epoch.
                   A concurrent external STOP increments it again and wins. */
                ++c.generation;
                if (c.generation != esp_link_command_generation())
                {
                    free(c.prog);
                    continue;
                }
            }
            s_cur = c.prog;
            s_generation = c.generation;
            s_pc = 0;
            s_step_start_ms = 0;
            s_step_started = false;
            s_state = MACRO_STATE_RUNNING;
            s_resume_pending = true;
            s_completion_pending = false;
            report_progress(MACRO_STATE_RUNNING);
        }

        if (s_cur && s_generation != esp_link_command_generation())
            finish_macro(MACRO_STATE_ABORTED, false);
        if (s_cur && s_resume_pending)
        {
            /* Retry only this explicitly requested START, until STOP is on wire. */
            if (esp_link_resume_commands(s_generation)) s_resume_pending = false;
        }
        if (s_cur && !s_resume_pending && esp_link_commands_inhibited())
            finish_macro(MACRO_STATE_ABORTED, false);

        /* --- 执行当前步 --- */
        if (s_state == MACRO_STATE_RUNNING && s_cur != NULL && !s_resume_pending &&
            !s_completion_pending)
        {
            macro_step_t *st = &s_cur->steps[s_pc];
            bool advance = true;     /* GOTO 自行改 pc 时置 false */
            bool step_done = false;

            /* 按真实经过时间计时，包含任务工作和调度延迟；uint32_t 差值允许回绕。
               ACT 从步骤进入/提交命令开始计时，不代表 F407 已执行或返回 ACK。 */
            const uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            const bool entering = !s_step_started;
            if (entering) {
                s_step_start_ms = now;
                s_step_started = true;
            }
            const uint32_t elapsed = (uint32_t)(now - s_step_start_ms);

            switch (st->type)
            {
            case MACRO_STEP_ACT:
                if (entering)
                {
                    /* 进入步骤：发一次命令 */
                    if (st->u.act.cmd_id == ESP_CMD_STOP_ALL)
                    {
                        finish_macro(MACRO_STATE_ABORTED);
                        break;
                    }
                    if (!esp_link_send_command_for_generation(st->u.act.cmd_id, st->u.act.param,
                                                              st->u.act.param_len, s_generation))
                    {
                        finish_macro(MACRO_STATE_ABORTED);
                        break;
                    }
                    if (st->u.act.duration_ms == 0)
                        step_done = true;    /* 立即下一步（命令持续生效） */
                }
                if (elapsed >= st->u.act.duration_ms && st->u.act.duration_ms > 0)
                    step_done = true;
                break;

            case MACRO_STEP_WAIT:
            {
                esp_link_snapshot_t sn;
                if (xQueuePeek(esp_link_sensor_queue, &sn, 0) == pdTRUE)
                {
                    /* 在 peek 后采样，避免跨核刚发布的接收时间晚于前面的 now，
                       从而使无符号年龄下溢、把新帧误判为过期。 */
                    const uint32_t snapshot_now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                    const uint32_t age_ms = (uint32_t)(snapshot_now - sn.received_ms);
                    if (age_ms > ESP_LINK_STALE_MS) {
                        ESP_LOGE(TAG, "macro %u feedback stale (%lu ms, seq %lu); aborting and latching STOP",
                                 s_cur->id, (unsigned long)age_ms, (unsigned long)sn.seq);
                        /* finish_macro 默认请求 STOP；只推进一次安全代次，不能走超时续行。 */
                        finish_macro(MACRO_STATE_ABORTED);
                        break;
                    }
                    const uint8_t field = st->u.wait.field;
                    double v = field_value(&sn.sensor, field);
                    if (!isfinite(v))
                    {
                        ESP_LOGE(TAG, "macro %u feedback non-finite (field %u); aborting and latching STOP",
                                 s_cur->id, (unsigned)field);
                        finish_macro(MACRO_STATE_ABORTED);
                        break;
                    }
                    /* 帧接收新鲜不代表采样新鲜；远端年龄只用同帧 F407 时间戳相减。
                       sys_tick_ms 本身无需检查子系统年龄，无符号差允许 SysTick 回绕。 */
                    if (field != MACRO_FIELD_SYS_TICK_MS)
                    {
                        const uint32_t sub_age = sn.sensor.sys_tick_ms -
                            field_subsystem_timestamp_ms(&sn.sensor, field);
                        if (sub_age > ESP_LINK_STALE_MS)
                        {
                            ESP_LOGE(TAG, "macro %u subsystem feedback stale (field %u, %lu ms); aborting and latching STOP",
                                     s_cur->id, (unsigned)field, (unsigned long)sub_age);
                            finish_macro(MACRO_STATE_ABORTED);
                            break;
                        }
                    }
                    /* 只校验距离数值对应的有效状态；status 字段本身仍可用于 WAIT。 */
                    const bool invalid_distance =
                        (field == MACRO_FIELD_DIST_FRONT_MM && sn.sensor.dist_front_status != 0) ||
                        (field == MACRO_FIELD_DIST_LEFT_MM && sn.sensor.dist_left_status != 0) ||
                        (field == MACRO_FIELD_DIST_RIGHT_MM && sn.sensor.dist_right_status != 0);
                    if (invalid_distance)
                    {
                        ESP_LOGE(TAG, "macro %u distance invalid (field %u); aborting and latching STOP",
                                 s_cur->id, (unsigned)field);
                        finish_macro(MACRO_STATE_ABORTED);
                        break;
                    }
                    if (field_compare(v, st->u.wait.value, st->u.wait.op))
                        step_done = true;    /* 条件满足 */
                }
                else if (elapsed > ESP_LINK_STALE_MS)
                {
                    ESP_LOGE(TAG, "macro %u feedback unavailable (%lu ms in WAIT); aborting and latching STOP",
                             s_cur->id, (unsigned long)elapsed);
                    /* 首帧缺失同样锁 STOP；不能落到普通 timeout 续行。 */
                    finish_macro(MACRO_STATE_ABORTED);
                    break;
                }
                if (elapsed >= st->u.wait.timeout_ms)
                    step_done = true;        /* 超时也下一步 */
                break;
            }

            case MACRO_STEP_GOTO:
                if (st->u.got.target < 0 || st->u.got.target >= (int16_t)s_cur->count)
                {
                    s_completion_pending = true;  /* Drain earlier ACTs before normal completion. */
                }
                else
                {
                    s_pc         = (uint16_t)st->u.got.target;
                    s_step_start_ms = 0;
                    s_step_started = false;
                    advance      = false;
                }
                break;
            }

            if (s_cur != NULL && advance && step_done)
            {
                s_pc++;
                s_step_start_ms = 0;
                s_step_started = false;
                if (s_pc >= s_cur->count)
                {
                    s_completion_pending = true;
                }
                else
                {
                    report_progress(MACRO_STATE_RUNNING);
                }
            }
        }

        if (s_cur && s_completion_pending)
        {
            /* Enqueue success is not UART completion. A closing STOP changes the
               generation, so it must not invalidate this program's final ACT. */
            bool drained = esp_link_commands_drained(s_generation);
            if (s_generation != esp_link_command_generation())
                finish_macro(MACRO_STATE_ABORTED, false);
            else if (drained)
            {
                ESP_LOGI(TAG, "macro %u done", s_cur->id);
                finish_macro(MACRO_STATE_DONE);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MACRO_TICK_MS));
    }
}
