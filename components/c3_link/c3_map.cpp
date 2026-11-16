/**
 * @file    c3_map.cpp
 * @brief   PEO Rover - 手柄状态 -> 命令映射任务（~20Hz）
 *
 * 手柄快照来源 = c3_link_rx_task 解析的 C3 UART 状态帧（PROTOCOL.md §1.4）；
 * 本任务逻辑与移除前的 xbox_map_task 完全一致（仅数据源更换）：
 *
 * 映射策略（arcade 差速，占位实现，TODO 实测标定）：
 *   左摇杆纵轴 = 油门（前推 = 前进），横轴 = 转向；
 *   左轮 = 油门 + 转向，右轮 = 油门 - 转向。
 * 电机编号约定：TODO 确认（当前假设 0/1 = 左侧两轮，2/3 = 右侧两轮）。
 *
 * 安全：断连/卡死锁存 STOP；新鲜回中输入才解锁，之后的偏杆才运动。
 */

#include "c3_link.h"
#include "esp_link.h"
#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_log.h>

static const char *TAG = "c3_map";

/** 摇杆去中值 + 死区；返回 -32768..32767，死区内为 0 */
static int32_t joy_delta(uint16_t raw)
{
    int32_t d = (int32_t)raw - XB_JOY_CENTER;
    if (d > -XB_DEADZONE && d < XB_DEADZONE)
        return 0;
    return d;
}

/** 发送一条 MOTOR_CLOSED（motor + int32 LE 目标速度 0.01cm/s） */
static bool send_closed(uint8_t motor, int32_t speed_d100, uint32_t generation)
{
    uint8_t p[5];
    p[0] = motor;
    p[1] = (uint8_t)(speed_d100 & 0xFF);
    p[2] = (uint8_t)((speed_d100 >> 8) & 0xFF);
    p[3] = (uint8_t)((speed_d100 >> 16) & 0xFF);
    p[4] = (uint8_t)((speed_d100 >> 24) & 0xFF);
    return esp_link_send_command_for_generation(ESP_CMD_MOTOR_CLOSED, p, 5, generation);
}

void c3_map_task(void *arg)
{
    if (!xbox_state_queue) {
        ESP_LOGE(TAG, "state queue unavailable");
        esp_link_request_stop();
        vTaskDelete(NULL);
        return;
    }

    xbox_state_t st = {};
    bool was_connected = false, driving = false, armed = false;
    bool saw_inhibited = false;
    uint32_t observed_generation = esp_link_command_generation();
    uint32_t neutral_after_sequence = 0;
    uint32_t neutral_after_ms = 0;
    bool require_post_stop_input = false;
    uint32_t disconnect_generation = 0;
    const auto latch_stop = [&]() {
        esp_link_request_stop();  // Out-of-band latch: never lost to a full queue.
        observed_generation = esp_link_command_generation();
        neutral_after_sequence = st.notification_sequence;
        neutral_after_ms = millis();
        require_post_stop_input = true;
        saw_inhibited = true;
        driving = armed = false;
    };

    ESP_LOGI(TAG, "map task up");
    while (1) {
        do {
            if (xQueuePeek(xbox_state_queue, &st, 0) != pdTRUE) {
                if (was_connected) latch_stop();
                was_connected = false;
                break;
            }
            if (st.disconnect_generation != disconnect_generation) {
                disconnect_generation = st.disconnect_generation;
                latch_stop(); // Even if reconnect occurred between two snapshots.
                was_connected = false;
                break;
            }
            const uint32_t now = millis();
            const bool live = st.connected &&
                (uint32_t)(now - st.snapshot_ms) <= XB_SNAPSHOT_MAX_AGE_MS;
            if (!live) {
                if (was_connected) latch_stop();
                was_connected = false;
                armed = driving = false;
                break;
            }
            was_connected = true;

            // Observe every STOP generation, including a STOP already resumed by
            // another producer. Cached pre-STOP neutral must not release a latch.
            const uint32_t generation = esp_link_command_generation();
            const bool inhibited = esp_link_commands_inhibited();
            if (generation != observed_generation || (inhibited && !saw_inhibited)) {
                observed_generation = generation;
                neutral_after_sequence = st.notification_sequence;
                neutral_after_ms = now;
                require_post_stop_input = true;
                saw_inhibited = inhibited;
                armed = driving = false;
                break;
            }

            const int32_t throttle = joy_delta(st.joy_ly);
            const int32_t steer = joy_delta(st.joy_lx);
            const bool neutral = throttle == 0 && steer == 0;
            /* 输入新鲜度直接取自 C3 判定结果（PROTOCOL.md §1.4 input_fresh），
               不再用本地时间戳推断。 */
            const bool fresh = st.input_fresh != 0;
            if (inhibited || !armed) {
                const bool post_stop = !require_post_stop_input ||
                    (int32_t)(st.notification_ms - neutral_after_ms) > 0;
                if (fresh && neutral && post_stop && st.notification_sequence != neutral_after_sequence &&
                    (!inhibited || esp_link_resume_commands(generation))) {
                    armed = true;
                    require_post_stop_input = false;
                    saw_inhibited = false;
                }
                break; // A neutral packet can rearm, never start motion.
            }
            if (!fresh) {
                if (driving) latch_stop();
                else {
                    armed = false;
                    neutral_after_sequence = st.notification_sequence;
                }
                break;
            }
            if (neutral && !driving) break; // Idle never owns the control source.

            /* TODO calibrate signs and motor IDs: 0/1 left, 2/3 right. */
            int32_t left = (int32_t)((int64_t)(throttle + steer) * XB_MAX_SPEED_D100 / 32768);
            int32_t right = (int32_t)((int64_t)(throttle - steer) * XB_MAX_SPEED_D100 / 32768);
            if (left > XB_MAX_SPEED_D100) left = XB_MAX_SPEED_D100;
            if (left < -XB_MAX_SPEED_D100) left = -XB_MAX_SPEED_D100;
            if (right > XB_MAX_SPEED_D100) right = XB_MAX_SPEED_D100;
            if (right < -XB_MAX_SPEED_D100) right = -XB_MAX_SPEED_D100;

            // One captured generation for the entire batch. If STOP/resume races
            // between motors, old commands cannot become valid again.
            if (!send_closed(0, left, generation) || !send_closed(1, left, generation) ||
                !send_closed(2, right, generation) || !send_closed(3, right, generation)) {
                latch_stop();
            } else {
                driving = !neutral; // Send release-to-zero once, not every idle cycle.
            }
        } while (false);
        vTaskDelay(pdMS_TO_TICKS(XB_MAP_RATE_MS));
    }
}
