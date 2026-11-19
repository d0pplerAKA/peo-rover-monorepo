/**
 * @file    cmd_parser.cpp
 * @brief   PEO Rover - 上位机命令解析：TCP JSON 行 -> esp_link 命令帧
 *
 * 支持命令（PROTOCOL.md §2.1）：
 *   {"cmd":"LED_TOGGLE"} / {"cmd":"LED_ON"} / {"cmd":"LED_OFF"} / {"cmd":"STOP_ALL"}
 *   {"cmd":"ODOMETER_RESET"}
 *   {"cmd":"MOTOR","motor":0,"speed":500}
 *   {"cmd":"MOTOR_CLOSED","motor":1,"speed_d100":4000}
 *   {"cmd":"PID","kp":0.25,"ki":0.005}
 * 解析成功 -> 编码 cmd_id + param(小端) 入 esp_link_cmd_queue；
 * 回发 ACK {"ack":名字,"status":0}（status 1 = 参数错误/忙，2 = stale generation）。
 */

#include "cmd_parser.h"
#include "esp_link.h"
#include "esp_net.h"
#include "camera_service.h"
#include "ws2812_led.h"

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
    { "MOTOR",           ESP_CMD_MOTOR },
    { "PID",             ESP_CMD_PID },
    { "MOTOR_CLOSED",    ESP_CMD_MOTOR_CLOSED },
    { "STOP_ALL",        ESP_CMD_STOP_ALL },
    { "ODOMETER_RESET",  ESP_CMD_ODOMETER_RESET },
    { "SYS_RESET",       ESP_CMD_SYS_RESET },
    { "LED_ON",          ESP_CMD_LED_ON },
    { "LED_OFF",         ESP_CMD_LED_OFF },
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

    default:    /* LED_TOGGLE / STOP_ALL / ODOMETER_RESET：无参数 */
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
        esp_net_send_ack(out);  /* Transfers ownership even when the ACK queue is full. */
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
    const cJSON *jgen = cJSON_GetObjectItem(root, "gen");
    if (!resolve_generation(jgen, jc->valuestring, &generation)) return;

    /* CAM_RES 是 ESP32-S3 本地命令（不转发 F407）：相机分辨率热切换 */
    if (strcmp(jc->valuestring, "CAM_RES") == 0)
    {
        const cJSON *jr = cJSON_GetObjectItem(root, "res");
        uint8_t fs;
        if (!cJSON_IsString(jr) || !camera_resolution_from_name(jr->valuestring, &fs))
        {
            ESP_LOGW(TAG, "bad camera resolution name");
            send_ack(jc->valuestring, 1);
        }
        else if (!camera_request_resolution(fs))
        {
            ESP_LOGW(TAG, "camera service unavailable");
            send_ack(jc->valuestring, 1);
        }
        else
        {
            ESP_LOGI(TAG, "camera resolution -> %s", jr->valuestring);
            send_ack(jc->valuestring, 0);
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

    /* WS2812 灯珠是 S3 本地命令（RMT 直驱，不转发 F407） */
    if (strcmp(jc->valuestring, "WS2812") == 0)
    {
        const cJSON *jr = cJSON_GetObjectItem(root, "r");
        const cJSON *jg = cJSON_GetObjectItem(root, "g");
        const cJSON *jb = cJSON_GetObjectItem(root, "b");
        const cJSON *jbr = cJSON_GetObjectItem(root, "brightness");
        /* 校验原始数值后再窄化；只有缺失字段才使用默认亮度。 */
        if (!integer_in_range(jr, 0, 255) ||
            !integer_in_range(jg, 0, 255) ||
            !integer_in_range(jb, 0, 255) ||
            (jbr != NULL && !integer_in_range(jbr, 0, 100)))
        {
            ESP_LOGW(TAG, "bad ws2812 color params");
            send_ack(jc->valuestring, 1);
            return;
        }
        const uint8_t brightness = jbr ? (uint8_t)jbr->valuedouble : 100;
        if (!ws2812_set_color((uint8_t)jr->valuedouble, (uint8_t)jg->valuedouble,
                             (uint8_t)jb->valuedouble, brightness))
        {
            ESP_LOGW(TAG, "ws2812 service unavailable");
            send_ack(jc->valuestring, 1);
        }
        else
        {
            ESP_LOGI(TAG, "ws2812 -> r=%d g=%d b=%d bright=%u%%",
                     jr->valueint, jg->valueint, jb->valueint, (unsigned)brightness);
            send_ack(jc->valuestring, 0);
        }
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
           才能识别 STOP/单向心跳豁免，以及使用线 gen 的命令。 */
        const bool local_generation_current = msg.generation == esp_link_command_generation();
        cJSON *root = cJSON_ParseWithOpts(msg.line, NULL, true);
        /* 正常/非法 JSON 共用释放点：先归还完整行预算，再释放字符串，避免重复归还。 */
        esp_net_rx_release(msg.line_bytes);
        free(msg.line);
        if (root == NULL)
            continue;   /* 非法 JSON：忽略 */

        const cJSON *jc = cJSON_GetObjectItem(root, "cmd");
        const bool stop_all = cJSON_IsString(jc) && strcmp(jc->valuestring, "STOP_ALL") == 0;
        /* 安全心跳仅续期，不回包、不恢复运动，也不执行命令。
           不依赖 gen，保证 STOP 后等待新遥测期间仍可维持连接。 */
        const cJSON *jt = cJSON_GetObjectItem(root, "type");
        const bool heartbeat = jc == NULL && cJSON_IsString(jt) &&
                               strcmp(jt->valuestring, "heartbeat") == 0;
        if (heartbeat)
        {
            esp_net_note_client_activity(msg.slot, msg.received_ms, false);
            cJSON_Delete(root);
            continue;
        }
        const bool tagged_work = cJSON_GetObjectItem(root, "gen") != NULL && jc != NULL;
        if (!stop_all && !tagged_work &&
            (!local_generation_current || msg.generation != esp_link_command_generation()))
        {
            cJSON_Delete(root);
            continue;
        }

        if (cJSON_GetObjectItem(root, "cmd") != NULL)
            handle_command(root, msg.generation, msg.slot, msg.received_ms);

        cJSON_Delete(root);
    }
}
