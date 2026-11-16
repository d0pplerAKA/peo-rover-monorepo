/**
 * @file    cmd_parser.cpp
 * @brief   PEO Rover - 上位机命令解析：TCP JSON 行 -> esp_link 命令帧
 *
 * 支持命令（PROTOCOL.md §2.1）：
 *   {"cmd":"LED_TOGGLE"} / {"cmd":"SENSOR_RESET"} / {"cmd":"STOP_ALL"}
 *   {"cmd":"ODOMETER_RESET"}
 *   {"cmd":"MOTOR","motor":0,"speed":500}
 *   {"cmd":"MOTOR_CLOSED","motor":1,"speed_d100":4000}
 *   {"cmd":"PID","kp":0.25,"ki":0.005}
 * 解析成功 -> 编码 cmd_id + param(小端) 入 esp_link_cmd_queue；
 * 回发 ACK {"ack":名字,"status":0}（status 1 = 参数错误/忙，2 = stale generation）。
 *
 * Macros are validated/compiled here; START owns its captured safety generation.
 */

#include "cmd_parser.h"
#include "esp_link.h"
#include "esp_net.h"
#include "macro_engine.h"
#include "camera_service.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <cJSON.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

static const char *TAG = "cmd_parser";

/* ============================================================================
   命令表：JSON 名字 <-> 命令 ID + 参数布局
   ============================================================================ */
typedef struct
{
    const char *name;
    uint8_t     id;
} cmd_entry_t;

static const cmd_entry_t k_cmd_table[] = {
    { "LED_TOGGLE",      ESP_CMD_LED_TOGGLE },
    { "SENSOR_RESET",    ESP_CMD_SENSOR_RESET },
    { "MOTOR",           ESP_CMD_MOTOR },
    { "PID",             ESP_CMD_PID },
    { "MOTOR_CLOSED",    ESP_CMD_MOTOR_CLOSED },
    { "STOP_ALL",        ESP_CMD_STOP_ALL },
    { "ODOMETER_RESET",  ESP_CMD_ODOMETER_RESET },
};

static int8_t cmd_find(const char *name)
{
    for (unsigned i = 0; i < sizeof(k_cmd_table) / sizeof(k_cmd_table[0]); i++)
        if (strcmp(k_cmd_table[i].name, name) == 0)
            return (int8_t)i;
    return -1;
}

static bool integer_in_range(const cJSON *v, double low, double high)
{
    return cJSON_IsNumber(v) && isfinite(v->valuedouble) &&
           v->valuedouble >= low && v->valuedouble <= high &&
           floor(v->valuedouble) == v->valuedouble;
}

static bool finite_float(const cJSON *v)
{
    return cJSON_IsNumber(v) && isfinite(v->valuedouble) &&
           fabs(v->valuedouble) <= FLT_MAX;
}

/** 按命令 ID 从 JSON 取参数编码进 param（小端，与 F407 约定一致）；返回 param 长度 */
static uint8_t pack_param(uint8_t id, const cJSON *root, uint8_t *param)
{
    cJSON *jv;

    switch (id)
    {
    case ESP_CMD_MOTOR:
        jv = cJSON_GetObjectItem(root, "motor");
        if (!integer_in_range(jv, 0, 3)) return 0;
        param[0] = (uint8_t)jv->valuedouble;
        jv = cJSON_GetObjectItem(root, "speed");
        if (!integer_in_range(jv, INT16_MIN, INT16_MAX)) return 0;
        {
            int32_t spd = (int32_t)jv->valuedouble;
            param[1] = (uint8_t)(spd & 0xFF);
            param[2] = (uint8_t)((spd >> 8) & 0xFF);
        }
        return 3;

    case ESP_CMD_PID:
        jv = cJSON_GetObjectItem(root, "kp");
        if (!finite_float(jv)) return 0;
        {
            float kp = (float)jv->valuedouble;
            memcpy(param, &kp, 4);
        }
        jv = cJSON_GetObjectItem(root, "ki");
        if (!finite_float(jv)) return 0;
        {
            float ki = (float)jv->valuedouble;
            memcpy(param + 4, &ki, 4);
        }
        return 8;

    case ESP_CMD_MOTOR_CLOSED:
        jv = cJSON_GetObjectItem(root, "motor");
        if (!integer_in_range(jv, 0, 3)) return 0;
        param[0] = (uint8_t)jv->valuedouble;
        jv = cJSON_GetObjectItem(root, "speed_d100");
        if (!integer_in_range(jv, INT32_MIN, INT32_MAX)) return 0;
        {
            int32_t spd = (int32_t)jv->valuedouble;
            param[1] = (uint8_t)(spd & 0xFF);
            param[2] = (uint8_t)((spd >> 8) & 0xFF);
            param[3] = (uint8_t)((spd >> 16) & 0xFF);
            param[4] = (uint8_t)((spd >> 24) & 0xFF);
        }
        return 5;

    default:    /* LED_TOGGLE / SENSOR_RESET / STOP_ALL / ODOMETER_RESET：无参数 */
        return 0;
    }
}

/** 回发 ACK（广播给所有客户端） */
static void send_ack(const char *cmd_name, uint8_t status)
{
    cJSON *ack = cJSON_CreateObject();
    if (!ack) return;
    bool ok = cJSON_AddStringToObject(ack, "ack", cmd_name) &&
              cJSON_AddNumberToObject(ack, "status", status);
    char *out = ok ? cJSON_PrintUnformatted(ack) : NULL;
    cJSON_Delete(ack);
    if (out != NULL)
    {
        if (xQueueSend(esp_net_ack_queue, &out, 0) != pdTRUE)
            free(out);
    }
}

/* PING 独立回包，不改变普通 ACK 格式，也不恢复运动。 */
static void send_ping_ack(double timestamp)
{
    cJSON *ack = cJSON_CreateObject();
    if (!ack) return;
    bool ok = cJSON_AddStringToObject(ack, "ack", "PING") &&
              cJSON_AddNumberToObject(ack, "status", 0) &&
              cJSON_AddNumberToObject(ack, "t", timestamp);
    char *out = ok ? cJSON_PrintUnformatted(ack) : NULL;
    cJSON_Delete(ack);
    if (out != NULL && xQueueSend(esp_net_ack_queue, &out, 0) != pdTRUE)
        free(out);
}

/** v1.5 可选线代次：缺失时保留消息本地代次；存在但不是有限 uint32 整数
 * 统一作为参数错误（status=1），不能降级到无 gen 的兼容路径。 */
static bool resolve_generation(const cJSON *jgen, const char *ack_name, uint32_t *generation)
{
    if (jgen == NULL) return true;
    if (!integer_in_range(jgen, 0, UINT32_MAX))
    {
        send_ack(ack_name, 1);
        return false;
    }
    *generation = (uint32_t)jgen->valuedouble;
    if (*generation != esp_link_command_generation())
    {
        send_ack(ack_name, 2);   /* stale generation：不得恢复或提交命令/宏 */
        return false;
    }
    return true;
}

/** 处理一条 JSON 命令（root 已解析） */
static void handle_command(const cJSON *root, uint32_t generation,
                           uint8_t slot, uint32_t received_ms)
{
    const cJSON *jc = cJSON_GetObjectItem(root, "cmd");
    if (!cJSON_IsString(jc))
        return;

    /* STOP_ALL 无条件接受：不受线 gen 的格式/数值或本地消息代次限制。 */
    if (strcmp(jc->valuestring, "STOP_ALL") == 0)
    {
        esp_link_request_stop();
        send_ack(jc->valuestring, 0);
        return;
    }
    /* 诊断命令无条件回包：在 gen 校验和 STOP 恢复逻辑之前处理。 */
    if (strcmp(jc->valuestring, "PING") == 0)
    {
        esp_net_note_client_activity(slot, received_ms, false);
        const cJSON *jt = cJSON_GetObjectItem(root, "t");
        send_ping_ack(cJSON_IsNumber(jt) ? (double)jt->valuedouble : 0.0);
        return;
    }
    const cJSON *jgen = cJSON_GetObjectItem(root, "gen");
    if (!resolve_generation(jgen, jc->valuestring, &generation)) return;

    /* CAM_RES 是 ESP32-S3 本地命令（不转发 F407）：相机分辨率热切换 */
    if (strcmp(jc->valuestring, "CAM_RES") == 0)
    {
        const cJSON *jr = cJSON_GetObjectItem(root, "res");
        uint8_t fs;
        if (cJSON_IsString(jr) && camera_resolution_from_name(jr->valuestring, &fs) &&
            camera_request_resolution(fs))
        {
            ESP_LOGI(TAG, "camera resolution -> %s", jr->valuestring);
            send_ack(jc->valuestring, 0);
        }
        else
        {
            ESP_LOGW(TAG, "bad camera resolution name");
            send_ack(jc->valuestring, 1);
        }
        return;
    }

    /* 视频开关是 S3 本地命令，复用 CAM_RES 的 gen 校验。 */
    if (strcmp(jc->valuestring, "VIDEO_ON") == 0 ||
        strcmp(jc->valuestring, "VIDEO_OFF") == 0)
    {
        esp_net_video_set_enabled(strcmp(jc->valuestring, "VIDEO_ON") == 0);
        send_ack(jc->valuestring, 0);
        return;
    }

    int8_t idx = cmd_find(jc->valuestring);
    if (idx < 0)
    {
        ESP_LOGW(TAG, "unknown cmd: %s", jc->valuestring);
        send_ack(jc->valuestring, 1);
        return;
    }

    uint8_t param[ESP_CMD_PARAM_MAX] = { 0 };
    uint8_t param_len = pack_param(k_cmd_table[idx].id, root, param);
    if (param_len == 0 &&
        (k_cmd_table[idx].id == ESP_CMD_MOTOR ||
         k_cmd_table[idx].id == ESP_CMD_MOTOR_CLOSED ||
         k_cmd_table[idx].id == ESP_CMD_PID))
    {
        ESP_LOGW(TAG, "bad params for %s", jc->valuestring);
        send_ack(jc->valuestring, 1);
        return;
    }

    if (generation != esp_link_command_generation())
    {
        /* 参数处理期间也可能发生 STOP；无 gen 消息仍按原行为静默丢弃。 */
        if (jgen != NULL) send_ack(jc->valuestring, 2);
        return;
    }
    if (esp_link_resume_commands(generation) &&
        generation == esp_link_command_generation() &&
        esp_link_send_command_for_generation(k_cmd_table[idx].id, param, param_len, generation))
    {
        if (k_cmd_table[idx].id == ESP_CMD_MOTOR ||
            k_cmd_table[idx].id == ESP_CMD_MOTOR_CLOSED ||
            k_cmd_table[idx].id == ESP_CMD_PID)
            esp_net_note_client_activity(slot, received_ms, true);
        ESP_LOGI(TAG, "cmd %s -> F407 (id 0x%02X)", jc->valuestring,
                 k_cmd_table[idx].id);
        send_ack(jc->valuestring, 0);
    }
    else
    {
        ESP_LOGW(TAG, "cmd busy/inhibited, drop %s", jc->valuestring);
        send_ack(jc->valuestring,
                 jgen != NULL && generation != esp_link_command_generation() ? 2 : 1);
    }
}

/** 解析 {"type":"macro"} -> 预编译步骤数组 -> 移交宏引擎（START） */
static void handle_macro(const cJSON *root, uint32_t generation,
                         uint8_t slot, uint32_t received_ms)
{
    const cJSON *jgen = cJSON_GetObjectItem(root, "gen");
    if (!resolve_generation(jgen, "macro", &generation)) return;

    const cJSON *jid    = cJSON_GetObjectItem(root, "id");
    const cJSON *jsteps = cJSON_GetObjectItem(root, "steps");
    if (!integer_in_range(jid, 0, UINT16_MAX) || !cJSON_IsArray(jsteps))
    {
        ESP_LOGW(TAG, "macro: missing id/steps");
        send_ack("macro", 1);
        return;
    }

    int n = cJSON_GetArraySize(jsteps);
    if (n <= 0 || n > MACRO_MAX_STEPS)
    {
        ESP_LOGW(TAG, "macro: steps out of range (%d, max %d)", n, MACRO_MAX_STEPS);
        send_ack("macro", 1);
        return;
    }

    macro_prog_t *prog = (macro_prog_t *)malloc(sizeof(macro_prog_t));
    if (prog == NULL)
    {
        send_ack("macro", 1);
        return;
    }
    memset(prog, 0, sizeof(*prog));
    prog->id    = (uint16_t)jid->valuedouble;
    prog->count = (uint16_t)n;

    for (int i = 0; i < n; i++)
    {
        const cJSON  *js = cJSON_GetArrayItem(jsteps, i);
        macro_step_t *st = &prog->steps[i];
        bool ok = false;

        const cJSON *jg = cJSON_GetObjectItem(js, "goto");
        if (cJSON_IsNumber(jg))
        {
            /* goto：越界 = 宏结束（运行时处理） */
            st->type          = MACRO_STEP_GOTO;
            double target = jg->valuedouble;
            ok = isfinite(target) && floor(target) == target;
            st->u.got.target = ok && target >= 0 && target < n
                              ? (int16_t)target : MACRO_MAX_STEPS;
        }
        else
        {
            const cJSON *jw = cJSON_GetObjectItem(js, "wait");
            if (cJSON_IsObject(jw))
            {
                const cJSON *jf = cJSON_GetObjectItem(jw, "field");
                const cJSON *jo = cJSON_GetObjectItem(jw, "op");
                const cJSON *jv = cJSON_GetObjectItem(jw, "value");
                const cJSON *jt = cJSON_GetObjectItem(jw, "timeout_ms");
                if (cJSON_IsString(jf) && cJSON_IsString(jo) && cJSON_IsNumber(jv) &&
                    isfinite(jv->valuedouble) && integer_in_range(jt, 0, UINT32_MAX) &&
                    macro_field_from_name(jf->valuestring, &st->u.wait.field) &&
                    macro_op_from_name(jo->valuestring, &st->u.wait.op))
                {
                    st->type             = MACRO_STEP_WAIT;
                    st->u.wait.value     = jv->valuedouble;
                    st->u.wait.timeout_ms = (uint32_t)jt->valuedouble;
                    ok = true;
                }
            }
            else
            {
                const cJSON *jc = cJSON_GetObjectItem(js, "cmd");
                if (cJSON_IsString(jc))
                {
                    int8_t idx = cmd_find(jc->valuestring);
                    if (idx >= 0)
                    {
                        uint8_t param[MACRO_PARAM_MAX] = { 0 };
                        uint8_t plen = pack_param(k_cmd_table[idx].id, js, param);
                        bool need_param = (k_cmd_table[idx].id == ESP_CMD_MOTOR ||
                                           k_cmd_table[idx].id == ESP_CMD_MOTOR_CLOSED ||
                                           k_cmd_table[idx].id == ESP_CMD_PID);
                        if (!need_param || plen > 0)
                        {
                            st->type            = MACRO_STEP_ACT;
                            st->u.act.cmd_id    = k_cmd_table[idx].id;
                            st->u.act.param_len = plen;
                            memcpy(st->u.act.param, param, plen);
                            const cJSON *jd = cJSON_GetObjectItem(js, "duration_ms");
                            ok = jd == NULL || integer_in_range(jd, 0, UINT32_MAX);
                            st->u.act.duration_ms = ok && jd ? (uint32_t)jd->valuedouble : 0;
                        }
                    }
                }
            }
        }

        if (!ok)
        {
            ESP_LOGW(TAG, "macro: step %d invalid", i);
            free(prog);
            send_ack("macro", 1);
            return;
        }
    }

    macro_ctrl_t c;
    c.type = MACRO_CTRL_START;
    c.prog = prog;
    c.generation = generation;
    if (generation != esp_link_command_generation() ||
        xQueueSend(macro_ctrl_queue, &c, 0) != pdTRUE)
    {
        ESP_LOGE(TAG, "macro ctrl queue full");
        free(prog);
        send_ack("macro", jgen != NULL && generation != esp_link_command_generation() ? 2 : 1);
        return;
    }
    esp_net_note_client_activity(slot, received_ms, true);
    send_ack("macro", 0);
}

/** {"type":"macro_abort"} -> 宏引擎 ABORT */
static void handle_macro_abort(void)
{
    esp_link_request_stop(); /* Independent of the macro/ordinary command queues. */
}

void cmd_parser_task(void *arg)
{
    ESP_LOGI(TAG, "task up");

    while (1)
    {
        esp_net_rx_message_t msg = {};
        if (xQueueReceive(esp_net_rx_queue, &msg, portMAX_DELAY) != pdTRUE)
            continue;
        if (!msg.line) continue;
        /* 保留无 gen 消息在解析前/后的两次本地代次判定。必须先解析 JSON，
           才能识别 STOP/PING/macro_abort 豁免，以及使用线 gen 的命令/START 宏。 */
        const bool local_generation_current = msg.generation == esp_link_command_generation();
        cJSON *root = cJSON_ParseWithOpts(msg.line, NULL, true);
        /* 正常/非法 JSON 共用释放点：先归还完整行预算，再释放字符串，避免重复归还。 */
        esp_net_rx_release(msg.line_bytes);
        free(msg.line);
        if (root == NULL)
            continue;   /* 非法 JSON：忽略 */

        const cJSON *jc = cJSON_GetObjectItem(root, "cmd");
        const cJSON *jt = cJSON_GetObjectItem(root, "type");
        const bool stop_all = cJSON_IsString(jc) && strcmp(jc->valuestring, "STOP_ALL") == 0;
        const bool ping = cJSON_IsString(jc) && strcmp(jc->valuestring, "PING") == 0;
        const bool macro_abort = cJSON_IsString(jt) && strcmp(jt->valuestring, "macro_abort") == 0;
        const bool tagged_work = cJSON_GetObjectItem(root, "gen") != NULL &&
            (jc != NULL || (cJSON_IsString(jt) && strcmp(jt->valuestring, "macro") == 0));
        if (!stop_all && !ping && !macro_abort && !tagged_work &&
            (!local_generation_current || msg.generation != esp_link_command_generation()))
        {
            cJSON_Delete(root);
            continue;
        }

        if (cJSON_GetObjectItem(root, "cmd") != NULL)
        {
            handle_command(root, msg.generation, msg.slot, msg.received_ms);
        }
        else
        {
            const cJSON *jt = cJSON_GetObjectItem(root, "type");
            if (cJSON_IsString(jt))
            {
                if (strcmp(jt->valuestring, "macro") == 0)
                    handle_macro(root, msg.generation, msg.slot, msg.received_ms);
                else if (strcmp(jt->valuestring, "macro_abort") == 0)
                    handle_macro_abort();
            }
        }

        cJSON_Delete(root);
    }
}
