/**
 * @file  telemetry.cpp
 * @brief Telemetry merge task: SensorData -> JSON -> broadcast to the upper computer.
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

/*
 * JSON structure (PROTOCOL.md section 2.1, the sensor stream):
 *   {"type":"sensor","tick":..,"dist":{..},"imu":{..},"wheels":{..},
 *    "power":{..},"cmd_ack":{..},"sys":{..}}
 *
 * On 2026-11-15 the gamepad (C3 controller bridge) and macro (macro engine)
 * fields were removed together with those components.
 *
 * Every 50 ms we non-destructively peek the latest sensor snapshot, so the
 * nominal report rate is 20 Hz.
 */

#include "telemetry.h"
#include "esp_link.h"
#include "esp_net.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <freertos/idf_additions.h>
#include <cmath>
#include <float.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "telemetry";

/* S3 firmware version (reported with telemetry so the upper computer can verify
 * that all three ends of the system are on matching versions). */
#define PEO_S3_FW  (28U)

static constexpr size_t JSON_MAX = 1024;
/* Binary32 %.4f: sign + 39 integer digits + dot + 4 decimals + NUL = 46.
   Worst complete line (including link_stale and newline):
   prefix 70 + dist 104 + imu 348 + wheels 135 + power 96 + cmd_ack 34
   + control_mode 18 + fw_version 18 + s3_fw 12 + sys/closing/newline 76
   = 911 bytes; with NUL = 912, leaving 112 bytes. */
static constexpr size_t JSON_FLOAT_MAX = 46;
static_assert(FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128,
              "JSON float capacity assumes IEEE binary32 sensors");
static_assert(sizeof(size_t) <= sizeof(uint32_t), "JSON RAM capacity assumes at most 32-bit size_t");
static_assert(JSON_MAX >= 912, "JSON buffer must hold the worst complete sensor line");

/* One sample of the system monitor (CPU load and RAM). */
struct sys_sample_t {
    unsigned cpu0, cpu1;      /* Per-core busy percentage, 0-100. */
    size_t ram_free, ram_total;   /* Internal SRAM heap, in bytes. */
};

/** @brief  Sample CPU load (1 Hz cache) and RAM (fresh every call).
 *  @retval a sys_sample_t with per-core busy percent and internal heap bytes.
 *  Only the telemetry task calls this; the first call returns 0 for CPU, and a
 *  window shorter than 1 s keeps the previous CPU values. */
static sys_sample_t sample_sys(void)
{
    static bool sampled = false;
    static int64_t last_us = 0;
    static configRUN_TIME_COUNTER_TYPE last_idle[2] = {};
    static unsigned cpu[2] = {};
    const int64_t now_us = esp_timer_get_time();
    if (!sampled || now_us - last_us >= 1000000)
    {
        for (int core = 0; core < 2; ++core)
        {
            const configRUN_TIME_COUNTER_TYPE idle = ulTaskGetIdleRunTimeCounterForCore(core);
            if (sampled)
            {
                /* With IDF v6.1's xtensa port.c and ESP_TIMER configuration,
                 * xPortGetRunTimeCounterValue() returns esp_timer_get_time()
                 * directly: 1 tick = 1 us. Per-core idle accumulators update on
                 * task switch; the unsigned subtraction tolerates counter wrap.
                 * Each core's window total is window_us (not multiplied by the
                 * core count); the boundary error is bounded to 0..100. */
                const uint64_t idle_delta = (configRUN_TIME_COUNTER_TYPE)(idle - last_idle[core]);
                const uint64_t idle_pct = idle_delta * 100 / (uint64_t)(now_us - last_us);
                cpu[core] = idle_pct >= 100 ? 0 : 100 - (unsigned)idle_pct;
            }
            last_idle[core] = idle;
        }
        last_us = now_us;
        sampled = true;
    }
    /* RAM stays fresh on every generated frame; only CPU is cached at 1 Hz. */
    return {cpu[0], cpu[1], heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
            heap_caps_get_total_size(MALLOC_CAP_INTERNAL)};
}

/** @brief  Build one complete telemetry JSON line.
 *  @param s           Sensor snapshot to serialize.
 *  @param link_stale  true = the snapshot is old or missing (adds "link_stale":true).
 *  @retval a malloc'd, NUL-terminated JSON string (caller takes ownership), or
 *          NULL on allocation/serialization failure. Never returns a truncated object. */
static char *build_json(const esp_link_sensor_t *s, bool link_stale)
{
    char *out = (char *)malloc(JSON_MAX);
    if (!out) return NULL;
    const uint32_t generation = esp_link_command_generation();
    const float values[] = {s->quat_w, s->quat_x, s->quat_y, s->quat_z,
                            s->accel_x, s->accel_y, s->accel_z};
    char imu[7][JSON_FLOAT_MAX];
    for (size_t i = 0; i < 7; ++i)
    {
        if (!std::isfinite(values[i]))
        {
            strcpy(imu[i], "null");  /* Preserve cJSON's NaN/Inf representation. */
            continue;
        }
        /* This target's Picolibc uses the fixed C locale (decimal point '.'). */
        const int n = snprintf(imu[i], sizeof(imu[i]), i < 4 ? "%.4f" : "%.3f",
                               (double)values[i]);
        if (n < 0 || (size_t)n >= sizeof(imu[i]))
        {
            free(out);
            return NULL;
        }
    }
    const sys_sample_t sys = sample_sys();
    const int n = snprintf(out, JSON_MAX,
        "{\"type\":\"sensor\",\"tick\":%" PRIu32 ",\"gen\":%" PRIu32 "%s,"
        "\"dist\":{\"front\":{\"mm\":%u,\"ok\":%s},"
                    "\"left\":{\"mm\":%u,\"ok\":%s},"
                    "\"right\":{\"mm\":%u,\"ok\":%s}},"
        "\"imu\":{\"quat\":[%s,%s,%s,%s],\"accel_g\":[%s,%s,%s]},"
        "\"wheels\":{\"speed_d100\":[%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 "],"
                      "\"odometer\":[%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 "]},"
        "\"power\":{\"current_mA\":[%d,%d,%d,%d],\"bus_voltage_mV\":[%u,%u,%u,%u]},"
        "\"cmd_ack\":{\"id\":%u,\"status\":%u},"
        "\"control_mode\":%u,"
        "\"lock\":%u,"
        "\"fw_version\":%u,"
        "\"s3_fw\":%u,"
        "\"sys\":{\"cpu0\":%u,\"cpu1\":%u,\"ram_free\":%zu,\"ram_total\":%zu}}\n",
        s->sys_tick_ms, generation, link_stale ? ",\"link_stale\":true" : "",
        (unsigned)s->dist_front_mm, s->dist_front_status == 0 ? "true" : "false",
        (unsigned)s->dist_left_mm, s->dist_left_status == 0 ? "true" : "false",
        (unsigned)s->dist_right_mm, s->dist_right_status == 0 ? "true" : "false",
        imu[0], imu[1], imu[2], imu[3], imu[4], imu[5], imu[6],
        s->speed_d100[0], s->speed_d100[1], s->speed_d100[2], s->speed_d100[3],
        s->odometer[0], s->odometer[1], s->odometer[2], s->odometer[3],
        (int)s->current_mA[0], (int)s->current_mA[1], (int)s->current_mA[2], (int)s->current_mA[3],
        (unsigned)s->bus_voltage_mV[0], (unsigned)s->bus_voltage_mV[1],
        (unsigned)s->bus_voltage_mV[2], (unsigned)s->bus_voltage_mV[3],
        (unsigned)s->last_cmd_id, (unsigned)s->last_cmd_status,
        (unsigned)s->control_mode,
        (unsigned)s->lock,
        (unsigned)s->fw_version, (unsigned)PEO_S3_FW,
        sys.cpu0, sys.cpu1, sys.ram_free, sys.ram_total);
    if (n < 0 || (size_t)n >= JSON_MAX)
    {
        free(out);
        return NULL;
    }
    return out;
}

/** @brief  Telemetry task: peek the sensor snapshot at 20 Hz and broadcast it.
 *  @param arg  Unused.
 *  @retval none (runs forever). */
void telemetry_task(void *arg)
{
    EventGroupHandle_t eg = esp_net_conn_event();
    if (!eg)
    {
        ESP_LOGE(TAG, "connection event group missing; task exits");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "task up");

    /* Lock-driven telemetry throttle counter (manual 2 Hz / autonomous 20 Hz). */
    unsigned tick = 0;

    while (1)
    {
        /* No client -> block silently and broadcast nothing (woken when the
         * network task accepts a client). */
        xEventGroupWaitBits(eg, PEO_NET_EVT_CLIENT, pdFALSE, pdTRUE, portMAX_DELAY);
        TickType_t last_wake = xTaskGetTickCount();
        while (xEventGroupGetBits(eg) & PEO_NET_EVT_CLIENT)
        {
        /* Latest-value mailbox (single reader: telemetry). */
        /* 20 Hz telemetry: fixed 50 ms cadence so JSON build time never creeps
         * into the period. The cadence baseline resets on reconnect; the sys
         * object and the sensor data format stay the same. */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
        esp_link_snapshot_t sn;
        const bool have_sensor = xQueuePeek(esp_link_sensor_queue, &sn, 0) == pdTRUE;
        if (!have_sensor)
        {
            /* No sensor snapshot (F407 unplugged / link down): still broadcast sys
             * (CPU/RAM monitoring is independent of the sensor stream). Distance
             * status is set to 1 = invalid so a zero is not read as a real reading;
             * with no snapshot we flag link_stale directly, without depending on
             * whether the uptime already exceeds the threshold. */
            memset(&sn, 0, sizeof(sn));
            sn.sensor.dist_front_status = 1;
            sn.sensor.dist_left_status  = 1;
            sn.sensor.dist_right_status = 1;
        }
        /* Take now AFTER all peeks, so a snapshot published from the other core
         * with a later timestamp than our earlier sample cannot underflow the age. */
        const uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        const bool link_stale = !have_sensor ||
            (uint32_t)(now - sn.received_ms) > ESP_LINK_STALE_MS;

        /* Keep the snapshot as a peek; with no client, skip JSON alloc/serialize
         * and enqueue, and the next round still delays on the normal cadence. */
        if (!esp_net_has_client()) continue;

        /* Lock-driven telemetry throttle: lock closed (manual, lock=0) -> 2 Hz;
         * lock open (autonomous, lock=1) -> 20 Hz. We still peek on a fixed 50 ms
         * cadence (keeps the snapshot fresh); in manual mode we publish once every
         * 10 ticks (500 ms). */
        if (sn.sensor.lock == 0 && (tick % 10) != 0)
        {
            ++tick;
            continue;
        }
        ++tick;

        const uint32_t gen_before = esp_link_command_generation();
        char *json = build_json(&sn.sensor, link_stale);
        if (gen_before != esp_link_command_generation())
        {
            free(json);  /* STOP/generation changed during the build: drop the mixed-epoch frame. */
            continue;
        }
        if (json == NULL)
            continue;

        esp_net_publish_telemetry(json);  /* Always transfers ownership; do not free here. */
        }
    }
}
