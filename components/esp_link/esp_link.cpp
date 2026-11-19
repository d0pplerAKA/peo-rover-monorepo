/**
 * @file    esp_link.cpp
 * @brief   PEO Rover - esp_link 实现：CRC16 + UART 帧收发（FreeRTOS 任务）
 *
 * 任务分工（main 里 xTaskCreate 创建）：
 *   esp_link_rx_task（优先级 6）：初始化 UART（队列已由 app_main 创建）；
 *                                阻塞读字节流 -> 帧状态机 -> CRC 校验
 *                                -> SensorData 快照入 sensor_queue。
 *   esp_link_tx_task（优先级 5）：通知唤醒 + 命令队列消费 -> 打包命令帧 -> 发送
 *                                -> 停顿 >= 1 字节时间（触发 F407 IDLE 组帧）。
 */

#include "esp_link.h"
#include "PEO_PINMAP.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <atomic>
#include <freertos/event_groups.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <string.h>

static const char *TAG = "esp_link";

QueueHandle_t esp_link_cmd_queue    = NULL;
QueueHandle_t esp_link_sensor_queue = NULL;
static EventGroupHandle_t s_uart_event = NULL;
static constexpr EventBits_t UART_READY = 1U;
static portMUX_TYPE s_safety_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_generation = 0;
static bool s_inhibited = false;
static bool s_fatal = false;          /* 启动健康故障永久锁存，由 s_safety_lock 保护。 */
static bool s_stop_pending = false;
static TaskHandle_t s_tx_task = NULL;   /* 注册/读取均由 s_safety_lock 保护；TX 常驻 */
static constexpr uint32_t TX_WAKE = 1U; /* 通知只唤醒；STOP 状态与命令队列才是数据源 */

void esp_link_request_stop(void)
{
    portENTER_CRITICAL(&s_safety_lock);
    ++s_generation;
    s_inhibited = true;
    s_stop_pending = true;
    const TaskHandle_t tx_task = s_tx_task;
    portEXIT_CRITICAL(&s_safety_lock);
    if (tx_task != NULL) xTaskNotify(tx_task, TX_WAKE, eSetBits);
}

void esp_link_fatal_fault(void)
{
    portENTER_CRITICAL(&s_safety_lock);
    s_fatal = true;
    ++s_generation;
    s_inhibited = true;
    s_stop_pending = true;
    const TaskHandle_t tx_task = s_tx_task;
    portEXIT_CRITICAL(&s_safety_lock);
    if (tx_task != NULL) xTaskNotify(tx_task, TX_WAKE, eSetBits);
}

uint32_t esp_link_command_generation(void)
{
    portENTER_CRITICAL(&s_safety_lock);
    const uint32_t value = s_generation;
    portEXIT_CRITICAL(&s_safety_lock);
    return value;
}

bool esp_link_resume_commands(uint32_t observed_generation)
{
    portENTER_CRITICAL(&s_safety_lock);
    const bool allowed = observed_generation == s_generation && !s_stop_pending && !s_fatal;
    if (allowed) s_inhibited = false;
    portEXIT_CRITICAL(&s_safety_lock);
    return allowed;
}

/* ============================================================================
   1. CRC16-CCITT（多项式 0x1021，初始 0xFFFF，查表法）
      与 F407 ESP_LINK_CRC16 完全一致，标准测试向量 crc("123456789") == 0x29B1
   ============================================================================ */
static uint16_t s_crc_table[256];
static bool     s_crc_ready = false;

static void crc_build_table(void)
{
    for (uint16_t i = 0; i < 256; i++)
    {
        uint16_t crc = (uint16_t)(i << 8);
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        s_crc_table[i] = crc;
    }
    s_crc_ready = true;
}

static uint16_t esp_link_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    if (!s_crc_ready)
        crc_build_table();

    while (len--)
        crc = (uint16_t)((crc << 8) ^ s_crc_table[((crc >> 8) ^ *data++) & 0xFFU]);
    return crc;
}

/* ============================================================================
   2. RX：有界滑动候选帧；CRC 失败保留后缀，允许连续流重同步
   ============================================================================ */
static uint8_t s_rx_buf[ESP_LINK_FRAME_OVH + ESP_LINK_RX_PAYLOAD_SIZE];
static size_t s_rx_count = 0;
static uint32_t s_snapshot_seq = 0;    /* 仅 RX 任务更新，不改变 F407 线协议 */

/** 一帧完整（CRC 已过）：解析 SensorData 入快照队列 */
static void handle_frame(uint16_t payload_len)
{
    if (payload_len != sizeof(esp_link_sensor_t))
    {
        ESP_LOGW(TAG, "unexpected payload len %u (expect %u)", payload_len,
                 (unsigned)sizeof(esp_link_sensor_t));
        return;
    }

    esp_link_snapshot_t snapshot;
    memcpy(&snapshot.sensor, &s_rx_buf[3], sizeof(snapshot.sensor));
    /* 与宏/遥测统一使用 FreeRTOS 单调毫秒时间；只记录完整有效帧。 */
    snapshot.received_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    snapshot.seq = ++s_snapshot_seq;
    xQueueOverwrite(esp_link_sensor_queue, &snapshot);
}

void esp_link_queues_init(void)
{
    if (esp_link_sensor_queue == NULL)
        esp_link_sensor_queue = xQueueCreate(1, sizeof(esp_link_snapshot_t));
    if (esp_link_cmd_queue == NULL)
        esp_link_cmd_queue = xQueueCreate(8, sizeof(esp_link_cmd_t));
    if (s_uart_event == NULL) s_uart_event = xEventGroupCreate();
    configASSERT(esp_link_sensor_queue && esp_link_cmd_queue && s_uart_event);
    crc_build_table();
}

void esp_link_rx_task(void *arg)
{
    static_assert(sizeof(esp_link_sensor_t) == 116,
                  "SensorData_t must be 116 bytes (PROTOCOL.md §1.2)");

    ESP_LOGI(TAG, "rx task starting");

    /* UART 初始化（8-N-1，无流控，与 F407 USART2 一致） */
    uart_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.baud_rate  = PEO_PM_ESP_BAUDRATE;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    esp_err_t err = uart_driver_install(PEO_PM_ESP_UART, 2048, 0, 0, NULL, 0);
    if (err == ESP_OK) err = uart_param_config(PEO_PM_ESP_UART, &cfg);
    if (err == ESP_OK)
        err = uart_set_pin(PEO_PM_ESP_UART, PEO_PM_ESP_TX, PEO_PM_ESP_RX,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %s; link remains disabled", esp_err_to_name(err));
        esp_link_request_stop();
        vTaskDelete(NULL);
        return;
    }
    xEventGroupSetBits(s_uart_event, UART_READY);

    ESP_LOGI(TAG, "rx task up (UART%d, %u baud)", PEO_PM_ESP_UART,
             (unsigned)PEO_PM_ESP_BAUDRATE);

    /* 3. 逐字节解析主循环 */
    while (1)
    {
        uint8_t b;
        if (uart_read_bytes(PEO_PM_ESP_UART, &b, 1, pdMS_TO_TICKS(100)) != 1) {
            s_rx_count = 0;  // Incomplete frame timeout / driver error.
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // Bounded sliding candidate. On CRC failure retain the suffix so an
        // embedded AA 55 70 cannot permanently hide a real frame after a lost byte.
        configASSERT(s_rx_count < sizeof(s_rx_buf));
        s_rx_buf[s_rx_count++] = b;
        while (s_rx_count != 0) {
            bool reject = s_rx_buf[0] != ESP_LINK_SOF0;
            if (!reject && s_rx_count >= 2) reject = s_rx_buf[1] != ESP_LINK_SOF1;
            if (!reject && s_rx_count >= 3) reject = s_rx_buf[2] != ESP_LINK_RX_PAYLOAD_SIZE;
            if (!reject && s_rx_count == sizeof(s_rx_buf)) {
                const size_t crc_offset = 3 + ESP_LINK_RX_PAYLOAD_SIZE;
                const uint16_t received = (uint16_t)s_rx_buf[crc_offset] |
                                          (uint16_t)(s_rx_buf[crc_offset + 1] << 8);
                if (esp_link_crc16(s_rx_buf, crc_offset) == received) {
                    handle_frame(ESP_LINK_RX_PAYLOAD_SIZE);
                    s_rx_count = 0;
                    break;
                }
                reject = true;
            }
            if (!reject) break;  // Keep a partial, plausible candidate.
            --s_rx_count;
            memmove(s_rx_buf, s_rx_buf + 1, s_rx_count);
        }
    }
}

/* 最近成功发送的命令名（net stats 仪表盘显示用）：
   atomic<const char*> 跨核安全；指针指向静态字符串字面量，生命周期=程序。
   ID->名字表须与 cmd_parser.cpp 的 k_cmd_table（名字->ID 方向）保持同步。 */
static std::atomic<const char *> s_last_cmd_name{nullptr};

static const char *cmd_name_for(uint8_t id)
{
    switch (id)
    {
    case ESP_CMD_LED_TOGGLE:      return "LED_TOGGLE";
    case ESP_CMD_MOTOR:           return "MOTOR";
    case ESP_CMD_PID:             return "PID";
    case ESP_CMD_MOTOR_CLOSED:    return "MOTOR_CLOSED";
    case ESP_CMD_STOP_ALL:        return "STOP_ALL";
    case ESP_CMD_ODOMETER_RESET:  return "ODOMETER_RESET";
    case ESP_CMD_SYS_RESET:       return "SYS_RESET";
    case ESP_CMD_LED_ON:          return "LED_ON";
    case ESP_CMD_LED_OFF:         return "LED_OFF";
    default:                      return NULL;
    }
}

const char *esp_link_last_command_name(void)
{
    return s_last_cmd_name.load();
}

/* ============================================================================
   3. TX：命令队列 -> 命令帧（每帧一条，帧后停顿触发 F407 IDLE 组帧）
   ============================================================================ */
static bool transmit_command(const esp_link_cmd_t &cmd)
{
    if (cmd.param_len > ESP_CMD_PARAM_MAX) return false;
    uint8_t frame[ESP_LINK_FRAME_OVH + 1 + ESP_CMD_PARAM_MAX];
    const uint8_t payload_len = (uint8_t)(1 + cmd.param_len);
    frame[0] = ESP_LINK_SOF0;
    frame[1] = ESP_LINK_SOF1;
    frame[2] = payload_len;
    frame[3] = cmd.cmd_id;
    memcpy(&frame[4], cmd.param, cmd.param_len);
    const uint16_t crc = esp_link_crc16(frame, (uint16_t)(3 + payload_len));
    frame[3 + payload_len] = (uint8_t)crc;
    frame[4 + payload_len] = (uint8_t)(crc >> 8);
    const size_t size = ESP_LINK_FRAME_OVH + payload_len;
    const int sent = uart_write_bytes(PEO_PM_ESP_UART, frame, size);
    const esp_err_t drained = uart_wait_tx_done(PEO_PM_ESP_UART, pdMS_TO_TICKS(100));
    // FIFO acceptance is NOT wire completion. Two ticks guarantee one full idle tick.
    vTaskDelay(pdMS_TO_TICKS(2));
    const bool ok = sent == (int)size && drained == ESP_OK;
    if (ok) s_last_cmd_name.store(cmd_name_for(cmd.cmd_id));   /* 仪表盘显示到下一命令更新 */
    return ok;
}

/* 失联自停心跳：发一个 LEN=0 空帧（SOF+LEN(0)+CRC）。F407 端 CRC 通过即刷新
   deadman 看门狗（空帧不触发任何命令）；S3 每 200ms 发一次。 */
static bool transmit_heartbeat(void)
{
    uint8_t frame[ESP_LINK_FRAME_OVH];
    frame[0] = ESP_LINK_SOF0;
    frame[1] = ESP_LINK_SOF1;
    frame[2] = 0;                          /* LEN = 0：空帧心跳 */
    const uint16_t crc = esp_link_crc16(frame, 3);
    frame[3] = (uint8_t)crc;
    frame[4] = (uint8_t)(crc >> 8);
    const int sent = uart_write_bytes(PEO_PM_ESP_UART, frame, sizeof(frame));
    const esp_err_t drained = uart_wait_tx_done(PEO_PM_ESP_UART, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(2));          /* 触发 F407 USART2 IDLE 组帧 */
    return sent == (int)sizeof(frame) && drained == ESP_OK;
}

void esp_link_tx_task(void *arg)
{
    portENTER_CRITICAL(&s_safety_lock);
    s_tx_task = xTaskGetCurrentTaskHandle();
    portEXIT_CRITICAL(&s_safety_lock);
    /* 注册前到达的 STOP/命令也不会丢失：就绪后先检查锁存状态和队列。 */
    xEventGroupWaitBits(s_uart_event, UART_READY, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "tx task up");
    while (1)
    {
        portENTER_CRITICAL(&s_safety_lock);
        const bool stop = s_stop_pending;
        const uint32_t generation = s_generation;
        portEXIT_CRITICAL(&s_safety_lock);
        if (stop) {
            esp_link_cmd_t cmd = {};
            cmd.cmd_id = ESP_CMD_STOP_ALL;
            if (transmit_command(cmd)) {
                portENTER_CRITICAL(&s_safety_lock);
                if (s_generation == generation) s_stop_pending = false;
                portEXIT_CRITICAL(&s_safety_lock);
            } else {
                ESP_LOGE(TAG, "STOP transmission failed; retaining safety latch");
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            continue;
        }
        esp_link_cmd_t cmd;
        if (xQueueReceive(esp_link_cmd_queue, &cmd, 0) != pdTRUE) {
            /* 队列优先，通知只作唤醒信号。成功入队与 STOP 都发送通知，
               因而“查空队列到进入等待”之间到达的工作也能唤醒本任务。
               清除通知后回到循环，重新检查 STOP，再非阻塞查队列；
               即使通知已合并/过时，也不会跳过队列或丢失唤醒。
               失联自停：200ms 无命令/通知则发空帧心跳给 F407（deadman 喂狗）。 */
            if (xTaskNotifyWait(0, UINT32_MAX, NULL, pdMS_TO_TICKS(200)) == pdFALSE)
                transmit_heartbeat();
            continue;
        }
        portENTER_CRITICAL(&s_safety_lock);
        /* 唯一发送提交点：通过即已提交，随后 STOP 最多跟在这一帧后面。
           UART 写入/等待始终在锁外，避免跨核自旋期间阻塞。 */
        const bool allowed = !s_inhibited && cmd.generation == s_generation;
        portEXIT_CRITICAL(&s_safety_lock);
        if (allowed) {
            if (!transmit_command(cmd)) {
                ESP_LOGE(TAG, "UART TX failed; requesting STOP");
                esp_link_request_stop();
            }
        }
    }
}

bool esp_link_send_command_for_generation(uint8_t cmd_id, const uint8_t *param,
                                          uint8_t param_len, uint32_t generation)
{
    esp_link_cmd_t cmd = {};
    if (cmd_id == ESP_CMD_STOP_ALL && param_len == 0) {
        esp_link_request_stop();
        return true;
    }
    if (esp_link_cmd_queue == NULL || param_len > ESP_CMD_PARAM_MAX ||
        (param_len != 0 && param == NULL))
        return false;
    portENTER_CRITICAL(&s_safety_lock);
    const bool allowed = !s_inhibited && generation == s_generation;
    cmd.generation = s_generation;
    portEXIT_CRITICAL(&s_safety_lock);
    if (!allowed) return false;
    cmd.cmd_id = cmd_id;
    cmd.param_len = param_len;
    if (param_len != 0) memcpy(cmd.param, param, param_len);
    if (xQueueSend(esp_link_cmd_queue, &cmd, 0) == pdTRUE) {
        /* 发布后再读取句柄：若 TX 在入队期间才注册，仍须通知它。
           若此时尚未注册，TX 入口会检查已发布的队列，因而也不会漏工作。
           TX 始终读取真实队列，通知次数不代表命令数。 */
        portENTER_CRITICAL(&s_safety_lock);
        const TaskHandle_t tx_task = s_tx_task;
        portEXIT_CRITICAL(&s_safety_lock);
        if (tx_task != NULL) xTaskNotify(tx_task, TX_WAKE, eSetBits);
        return true;
    }
    return false;
}
