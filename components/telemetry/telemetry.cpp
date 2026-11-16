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
 * 每轮延时 40ms 后非破坏性 peek 最新传感器快照，名义采样频率 25Hz。
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
#include <cJSON.h>
#include <stdlib.h>

static const char *TAG = "telemetry";

/* cJSON does not free an item when AddItem fails. Keep ownership explicit. */
static bool add_item(cJSON *parent, const char *name, cJSON *item)
{
    if (!item) return false;
    if (name ? cJSON_AddItemToObject(parent, name, item) : cJSON_AddItemToArray(parent, item))
        return true;
    cJSON_Delete(item);
    return false;
}

static bool add_dist(cJSON *dist, const char *name, uint16_t mm, uint8_t status)
{
    cJSON *node = cJSON_AddObjectToObject(dist, name);
    return node && cJSON_AddNumberToObject(node, "mm", mm) &&
           cJSON_AddBoolToObject(node, "ok", status == 0);
}

/* 仅由 telemetry 任务采样；首轮为 0，不足 1 秒保持上轮值。 */
static bool add_sys(cJSON *root)
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
    cJSON *sys = cJSON_AddObjectToObject(root, "sys");
    return sys && cJSON_AddNumberToObject(sys, "cpu0", cpu[0]) &&
           cJSON_AddNumberToObject(sys, "cpu1", cpu[1]) &&
           cJSON_AddNumberToObject(sys, "ram_free", heap_caps_get_free_size(MALLOC_CAP_INTERNAL)) &&
           cJSON_AddNumberToObject(sys, "ram_total", heap_caps_get_total_size(MALLOC_CAP_INTERNAL));
}

/** Return a complete owned JSON string, or NULL; never publish an OOM-truncated object. */
static char *build_json(const esp_link_sensor_t *s, bool link_stale)
{
    cJSON *root = cJSON_CreateObject();
    if (!root) return NULL;
    bool ok = [&]() -> bool {
        if (!cJSON_AddStringToObject(root, "type", "sensor") ||
            !cJSON_AddNumberToObject(root, "tick", s->sys_tick_ms) ||
            !cJSON_AddNumberToObject(root, "gen", esp_link_command_generation())) return false;
        /* 仅过期时添加可选标记；其余字段/结构保持原样，不把信封元数据外发。 */
        if (link_stale && !cJSON_AddBoolToObject(root, "link_stale", true)) return false;
        cJSON *dist = cJSON_AddObjectToObject(root, "dist");
        if (!dist || !add_dist(dist, "front", s->dist_front_mm, s->dist_front_status) ||
            !add_dist(dist, "left", s->dist_left_mm, s->dist_left_status) ||
            !add_dist(dist, "right", s->dist_right_mm, s->dist_right_status)) return false;

        cJSON *imu = cJSON_AddObjectToObject(root, "imu");
        double quat[] = {s->quat_w, s->quat_x, s->quat_y, s->quat_z};
        double accel[] = {s->accel_x, s->accel_y, s->accel_z};
        if (!imu || !add_item(imu, "quat", cJSON_CreateDoubleArray(quat, 4)) ||
            !add_item(imu, "accel_g", cJSON_CreateDoubleArray(accel, 3))) return false;

        cJSON *wheels = cJSON_AddObjectToObject(root, "wheels");
        int speed[] = {s->speed_d100[0], s->speed_d100[1], s->speed_d100[2], s->speed_d100[3]};
        int odo[] = {s->odometer[0], s->odometer[1], s->odometer[2], s->odometer[3]};
        if (!wheels || !add_item(wheels, "speed_d100", cJSON_CreateIntArray(speed, 4)) ||
            !add_item(wheels, "odometer", cJSON_CreateIntArray(odo, 4))) return false;

        cJSON *power = cJSON_AddObjectToObject(root, "power");
        int current[] = {s->current_mA[0], s->current_mA[1], s->current_mA[2], s->current_mA[3]};
        int voltage[] = {s->bus_voltage_mV[0], s->bus_voltage_mV[1], s->bus_voltage_mV[2], s->bus_voltage_mV[3]};
        if (!power || !add_item(power, "current_mA", cJSON_CreateIntArray(current, 4)) ||
            !add_item(power, "bus_voltage_mV", cJSON_CreateIntArray(voltage, 4))) return false;

        cJSON *ack = cJSON_AddObjectToObject(root, "cmd_ack");
        if (!ack || !cJSON_AddNumberToObject(ack, "id", s->last_cmd_id) ||
            !cJSON_AddNumberToObject(ack, "status", s->last_cmd_status)) return false;

        if (!add_sys(root)) return false;
        return true;
    }();
    char *out = ok ? cJSON_PrintUnformatted(root) : NULL;
    cJSON_Delete(root);
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
        while (xEventGroupGetBits(eg) & PEO_NET_EVT_CLIENT)
        {
        /* Latest-value mailbox（单读者：telemetry）。 */
        /* 25Hz 遥测：50Hz 时 ~30KB/s 生产速率 0.3s 就能顶爆积压上限；25Hz 减半，
           给 TCP 排水留时间（传感器数据本身 ~10Hz 更新，25Hz 已足够顺滑）。 */
        vTaskDelay(pdMS_TO_TICKS(40));
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

        if (xQueueSend(esp_net_tx_queue, &json, 0) != pdTRUE)
            free(json);   /* 广播队列满：丢弃本帧，等下帧 */
        }
    }
}
