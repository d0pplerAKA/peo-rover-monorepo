#include "led_status.h"
#include "PEO_PINMAP.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <math.h>
#include <atomic>
#include <string.h>
#include "driver/ledc.h"

static const char *TAG = "led_status";
static constexpr uint32_t LED_MAX_DUTY = (1U << 13) - 1;
static constexpr uint32_t LED_BREATH_MS = 1000;
static constexpr uint32_t LED_FLASH_MS = 1000;
static constexpr uint32_t LED_FLASH_ON_MS = 100;

static std::atomic<bool> s_connected{false};

static void led_status_task(void *arg)
{
    enum class State { BREATHING, FLASHING, SOLID };
    State state = State::BREATHING;
    bool prev_connected = false;
    uint32_t state_start = 0;

    while (1) {
        const uint32_t now = millis();
        const bool connected = s_connected.load(std::memory_order_relaxed);

        if (connected && !prev_connected) {
            state = State::FLASHING;
            state_start = now;
        }
        if (!connected)
            state = State::BREATHING;
        prev_connected = connected;

        uint32_t duty = 0;
        switch (state) {
        case State::BREATHING: {
            const float phase = (float)(now % LED_BREATH_MS) / LED_BREATH_MS * 6.283185307179586f;
            const float brightness = (sinf(phase) + 1.0f) / 2.0f;
            duty = (uint32_t)((1.0f - brightness) * LED_MAX_DUTY);
            break;
        }
        case State::FLASHING: {
            const uint32_t elapsed = (uint32_t)(now - state_start);
            if (elapsed < LED_FLASH_MS) {
                duty = ((elapsed / LED_FLASH_ON_MS) % 2 == 0) ? 0 : LED_MAX_DUTY;
            } else {
                state = State::SOLID;
                duty = 0;
            }
            break;
        }
        case State::SOLID:
            duty = 0;
            break;
        }

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void led_status_init(void)
{
    ledc_timer_config_t t;
    memset(&t, 0, sizeof t);
    t.speed_mode = LEDC_LOW_SPEED_MODE;
    t.duty_resolution = LEDC_TIMER_13_BIT;
    t.timer_num = LEDC_TIMER_0;
    t.freq_hz = 5000;
    t.clk_cfg = LEDC_AUTO_CLK;
    /* 状态灯是辅助功能：初始化失败只记日志停用、不 abort，避免拖垮核心心跳任务。 */
    if (ledc_timer_config(&t) != ESP_OK) {
        ESP_LOGW(TAG, "ledc timer config failed; status LED disabled");
        return;
    }

    ledc_channel_config_t c;
    memset(&c, 0, sizeof c);
    c.gpio_num = PEO_PM_LED_GPIO;
    c.speed_mode = LEDC_LOW_SPEED_MODE;
    c.channel = LEDC_CHANNEL_0;
    c.timer_sel = LEDC_TIMER_0;
    c.duty = LED_MAX_DUTY;   /* active-low：初始灭灯；任务创建失败也保持灭灯（不误判为已连接） */
    c.hpoint = 0;
    if (ledc_channel_config(&c) != ESP_OK) {
        ESP_LOGW(TAG, "ledc channel config failed; status LED disabled");
        return;
    }

    if (xTaskCreate(led_status_task, "led_status", 2048, NULL, 1, NULL) != pdPASS)
        ESP_LOGE(TAG, "led status task allocation failed");
}

void led_status_notify_connected(bool connected)
{
    s_connected.store(connected, std::memory_order_relaxed);
}
