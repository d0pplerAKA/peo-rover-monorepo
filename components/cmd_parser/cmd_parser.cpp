/**
 * @file  cmd_parser.cpp
 * @brief Upper-computer command parser: TCP JSON lines -> esp_link command frames.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

/*
 * Commands supported (PROTOCOL.md section 2.1):
 *   {"cmd":"LED_TOGGLE"} / {"cmd":"LED_ON"} / {"cmd":"LED_OFF"} / {"cmd":"STOP_ALL"}
 *   {"cmd":"ODOMETER_RESET"}
 *   {"cmd":"MOTOR","motor":0,"speed":500}
 *   {"cmd":"MOTOR_CLOSED","motor":1,"speed_d100":4000}
 *   {"cmd":"PID","kp":0.25,"ki":0.005}
 * A successfully parsed command is encoded as cmd_id + parameter bytes
 * (little-endian) and pushed into the esp_link command queue. We reply with an
 * ACK {"ack":name,"status":0} (status 1 = bad parameter / busy, 2 = stale generation).
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
   Command table: JSON name <-> command ID (parameter layout is handled in
   pack_param() below). Must stay in sync with the ESP_CMD_* macros in
   esp_link.h and with the F407 side of the protocol.
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

/** @brief  Find a command's table index by its JSON name.
 *  @param name  JSON command name, e.g. "MOTOR".
 *  @retval the table index (>= 0), or -1 if the name is unknown. */
static int8_t cmd_find(const char *name)
{
    for (unsigned i = 0; i < sizeof(k_cmd_table) / sizeof(k_cmd_table[0]); i++)
        if (strcmp(k_cmd_table[i].name, name) == 0)
            return (int8_t)i;
    return -1;
}

/** @brief  True if the JSON value is a whole number within [low, high].
 *  @param v     JSON value to check.
 *  @param low   Inclusive lower bound.
 *  @param high  Inclusive upper bound.
 *  @retval true if v is a finite number, in range, and has no fractional part. */
static bool integer_in_range(const cJSON *v, double low, double high)
{
    return cJSON_IsNumber(v) && isfinite(v->valuedouble) &&
           v->valuedouble >= low && v->valuedouble <= high &&
           floor(v->valuedouble) == v->valuedouble;
}

/** @brief  True if the JSON value is a finite float that fits in a C float.
 *  @param v  JSON value to check.
 *  @retval true if v is a finite number whose magnitude fits float range. */
static bool finite_float(const cJSON *v)
{
    return cJSON_IsNumber(v) && isfinite(v->valuedouble) &&
           fabs(v->valuedouble) <= FLT_MAX;
}

/** @brief  Encode a command's JSON parameters into the raw little-endian bytes.
 *  @param id     Command ID (ESP_CMD_*).
 *  @param root   Parsed JSON object.
 *  @param param  Output buffer (at least ESP_CMD_PARAM_MAX bytes).
 *  @retval number of parameter bytes written, or 0 on a parameter error.
 *  Byte order is little-endian to match the F407 side of the protocol. */
static uint8_t pack_param(uint8_t id, const cJSON *root, uint8_t *param)
{
    cJSON *jv;

    switch (id)
    {
    case ESP_CMD_MOTOR:
        /* motor index (0-3), then 16-bit signed speed. */
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
        /* Two 32-bit floats: kp then ki. */
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
        /* motor index (0-3), then 32-bit signed speed in 0.01 cm/s. */
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

    default:    /* LED_TOGGLE / STOP_ALL / ODOMETER_RESET: no parameters. */
        return 0;
    }
}

/** @brief  Send an ACK JSON line back (broadcast to all clients by the network task).
 *  @param cmd_name  Command name echoed in the ACK.
 *  @param status    0 = OK, 1 = bad parameter / busy, 2 = stale generation. */
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

/** @brief  Validate the optional v1.5 wire generation field and update the output.
 *  @param jgen        The parsed "gen" field, or NULL if absent.
 *  @param ack_name    Command name used in the ACK on error.
 *  @param generation  (in/out) Receives the validated generation value.
 *  @retval true  if the command may proceed (no gen field, or gen is current);
 *  @retval false if gen is malformed (status 1) or stale (status 2).
 *  A present-but-invalid gen is a hard parameter error: we never silently
 *  downgrade it to the no-gen compatibility path. */
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
        send_ack(ack_name, 2);   /* Stale generation: must not resume or submit commands. */
        return false;
    }
    return true;
}

/** @brief  Handle one parsed JSON command (root has already been validated).
 *  @param root         Parsed JSON object.
 *  @param generation   Safety generation captured with this message.
 *  @param slot         TCP client slot this message came from.
 *  @param received_ms  When the complete line arrived (FreeRTOS monotonic ms). */
static void handle_command(const cJSON *root, uint32_t generation,
                           uint8_t slot, uint32_t received_ms)
{
    const cJSON *jc = cJSON_GetObjectItem(root, "cmd");
    if (!cJSON_IsString(jc))
        return;

    /* STOP_ALL is accepted unconditionally: it is exempt from wire-gen format/value
     * checks and from the local message generation. Safety always wins. */
    if (strcmp(jc->valuestring, "STOP_ALL") == 0)
    {
        esp_link_request_stop();
        send_ack(jc->valuestring, 0);
        return;
    }
    const cJSON *jgen = cJSON_GetObjectItem(root, "gen");
    if (!resolve_generation(jgen, jc->valuestring, &generation)) return;

    /* CAM_RES is a local ESP32-S3 command (never forwarded to the F407):
     * camera resolution hot-swap. */
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

    /* Video on/off are local S3 commands; they reuse CAM_RES's generation check. */
    if (strcmp(jc->valuestring, "VIDEO_ON") == 0 ||
        strcmp(jc->valuestring, "VIDEO_OFF") == 0)
    {
        esp_net_video_set_enabled(strcmp(jc->valuestring, "VIDEO_ON") == 0);
        send_ack(jc->valuestring, 0);
        return;
    }

    /* WS2812 LED is a local S3 command (RMT direct drive, never forwarded to F407). */
    if (strcmp(jc->valuestring, "WS2812") == 0)
    {
        const cJSON *jr = cJSON_GetObjectItem(root, "r");
        const cJSON *jg = cJSON_GetObjectItem(root, "g");
        const cJSON *jb = cJSON_GetObjectItem(root, "b");
        const cJSON *jbr = cJSON_GetObjectItem(root, "brightness");
        /* Validate the raw values before narrowing the types; only a missing
         * brightness field falls back to the default (100%). */
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
        /* A STOP may also have happened while we were packing parameters; a
         * no-gen message is still silently dropped as before. */
        if (jgen != NULL) send_ack(jc->valuestring, 2);
        return;
    }
    if (esp_link_resume_commands(generation) &&
        generation == esp_link_command_generation() &&
        (k_cmd_table[idx].id == ESP_CMD_PID
            ? esp_link_send_pid_and_save(param, generation)
            : esp_link_send_command_for_generation(k_cmd_table[idx].id, param, param_len, generation)))
    {
        /* Motor / PID commands mark this client as "commanded": the network task
         * only supervises the heartbeat of clients that have actually commanded. */
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

/** @brief  Command parser task: pull JSON lines from the net RX queue and act on them.
 *  @param arg  Unused.
 *  @retval none (runs forever). */
void cmd_parser_task(void *arg)
{
    ESP_LOGI(TAG, "task up");

    while (1)
    {
        esp_net_rx_message_t msg = {};
        if (xQueueReceive(esp_net_rx_queue, &msg, portMAX_DELAY) != pdTRUE)
            continue;
        if (!msg.line) continue;
        /* Keep the two local-generation checks (before and after parsing) for
         * no-gen messages. We must parse the JSON first to recognize the
         * STOP/heartbeat exemptions and commands that carry a wire gen. */
        const bool local_generation_current = msg.generation == esp_link_command_generation();
        cJSON *root = cJSON_ParseWithOpts(msg.line, NULL, true);
        /* Normal and malformed JSON share one release point: return the line's
         * byte budget first, then free the string, to avoid a double release. */
        esp_net_rx_release(msg.line_bytes);
        free(msg.line);
        if (root == NULL)
            continue;   /* Malformed JSON: ignore. */

        const cJSON *jc = cJSON_GetObjectItem(root, "cmd");
        const bool stop_all = cJSON_IsString(jc) && strcmp(jc->valuestring, "STOP_ALL") == 0;
        /* The safety heartbeat only renews the connection: it does not reply, does
         * not resume motion, and does not run a command. It does not depend on gen
         * so the link stays alive even while we wait for fresh telemetry after STOP. */
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
