/**
 * @file    telemetry.cpp
 * @brief   PEO Rover - 遥测合并任务：SensorData -> JSON -> 广播
 *
 * JSON 结构见 PROTOCOL.md §2.1（传感器流）：
 *   {"type":"sensor","tick":..,"dist":{..},"imu":{..},"wheels":{..},
 *    "power":{..},"cmd_ack":{..},"sys":{..}}
 *
 * 2026-11-15：gamepad（C3 手柄桥）与 macro（宏引擎）字段已随组件废弃删除。
 *
 * 每 50ms 非破坏性 peek 最新传感器快照，名义回报频率 20Hz。
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

static constexpr size_t JSON_MAX = 1024;
/* Binary32 %.4f: sign + 39 integer digits + dot + 4 decimals + NUL = 46.
   Worst complete line (including link_stale and newline):
   prefix 70 + dist 104 + imu 348 + wheels 135 + power 96 + cmd_ack 34
   + sys/closing/newline 76 = 863 bytes; with NUL = 864, leaving 160 bytes. */
static constexpr size_t JSON_FLOAT_MAX = 46;
static_assert(FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128,
              "JSON float capacity assumes IEEE binary32 sensors");
static_assert(sizeof(size_t) <= sizeof(uint32_t), "JSON RAM capacity assumes at most 32-bit size_t");
static_assert(JSON_MAX >= 864, "JSON buffer must hold the worst complete sensor line");

struct sys_sample_t {
    unsigned cpu0, cpu1;
    size_t ram_free, ram_total;
};

/* 仅由 telemetry 任务采样；首轮为 0，不足 1 秒保持上轮值。 */
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
                /* IDF v6.1 xtensa port.c 的 xPortGetRunTimeCounterValue() 在
                   ESP_TIMER 配置下直接返回 esp_timer_get_time()：1 tick = 1 us。
                   按核 idle 累计值在任务切出时更新；无符号减法兼容计数回绕。
                   每核窗口总量为 window_us，不乘核数；边界误差限制到 0~100。 */
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

/** Return a complete owned JSON string, or NULL; never publish an OOM-truncated object. */
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
        sys.cpu0, sys.cpu1, sys.ram_free, sys.ram_total);
    if (n < 0 || (size_t)n >= JSON_MAX)
    {
        free(out);
        return NULL;
    }
    return out;
}

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

    while (1)
    {
        /* 无客户端 → 阻塞静止、零广播（esp_net accept 时置位唤醒） */
        xEventGroupWaitBits(eg, PEO_NET_EVT_CLIENT, pdFALSE, pdTRUE, portMAX_DELAY);
        TickType_t last_wake = xTaskGetTickCount();
        while (xEventGroupGetBits(eg) & PEO_NET_EVT_CLIENT)
        {
        /* Latest-value mailbox（单读者：telemetry）。 */
        /* 20Hz 遥测：固定 50ms 节拍，避免 JSON 处理时间累积到周期中。
           重新连接时重置节拍基准；sys 对象及传感器数据格式保持不变。 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
        esp_link_snapshot_t sn;
        const bool have_sensor = xQueuePeek(esp_link_sensor_queue, &sn, 0) == pdTRUE;
        if (!have_sensor)
        {
            /* 无传感器快照（F407 未接/链路断）：仍广播 sys（CPU/RAM 监控
               独立于传感器流）。status 置 1=无效，避免零值被解读为有效测距；
               无快照直接标记 link_stale，不依赖启动时间是否已超过阈值。 */
            memset(&sn, 0, sizeof(sn));
            sn.sensor.dist_front_status = 1;
            sn.sensor.dist_left_status  = 1;
            sn.sensor.dist_right_status = 1;
        }
        /* 所有 peek 后再取 now，避免跨核刚发布的快照时间晚于先前采样而导致年龄下溢。 */
        const uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        const bool link_stale = !have_sensor ||
            (uint32_t)(now - sn.received_ms) > ESP_LINK_STALE_MS;

        /* 保留快照 peek；无客户端时跳过 JSON 分配/序列化及入队，下一轮仍按原节拍延时。 */
        if (!esp_net_has_client()) continue;

        const uint32_t gen_before = esp_link_command_generation();
        char *json = build_json(&sn.sensor, link_stale);
        if (gen_before != esp_link_command_generation())
        {
            free(json);  /* 拼接期间发生 STOP/代次变化，丢弃混合代次帧。 */
            continue;
        }
        if (json == NULL)
            continue;

        esp_net_publish_telemetry(json);  /* Always transfers ownership; do not free here. */
        }
    }
}
