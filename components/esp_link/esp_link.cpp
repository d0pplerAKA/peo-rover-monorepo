/**
 * @file  esp_link.cpp
 * @brief esp_link implementation: CRC16 + UART frame receive/send (FreeRTOS tasks).
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

/*
 * Task split (created by xTaskCreate in main):
 *   esp_link_rx_task (priority 6): initializes the UART (queues are already
 *       created by app_main); blocks reading the byte stream -> frame state
 *       machine -> CRC check -> pushes the SensorData snapshot into sensor_queue.
 *   esp_link_tx_task (priority 5): woken by notification; consumes the command
 *       queue -> packs a command frame -> sends -> pauses >= 1 byte time
 *       (this triggers the F407 IDLE framing).
 */

#include "esp_link.h"
#include "PEO_PINMAP.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <atomic>
#include <freertos/event_groups.h>
#include <driver/uart.h>
#include <esp_log.h>
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

static const char *TAG = "esp_link";

QueueHandle_t esp_link_cmd_queue    = NULL;
QueueHandle_t esp_link_sensor_queue = NULL;
static EventGroupHandle_t s_uart_event = NULL;
static constexpr EventBits_t UART_READY = 1U;
static portMUX_TYPE s_safety_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_generation = 0;
static bool s_inhibited = false;
static bool s_fatal = false;          /* Startup health fault, latched forever; guarded by s_safety_lock. */
static bool s_stop_pending = false;
static TaskHandle_t s_tx_task = NULL;   /* Registered/read under s_safety_lock; TX is resident. */
static constexpr uint32_t TX_WAKE = 1U; /* Notification only wakes; STOP state and the command queue are the data source. */

/* The PID blob stores the wire protocol's 8 bytes directly: kp (float LE) +
 * ki (float LE). This mutex serializes "manual PID enqueue + save" against
 * "restore read + enqueue" so an older value cannot win the race.
 * NVS operations do not hold s_safety_lock; RX/net only publish the request and
 * never touch flash. */
static constexpr size_t PID_BLOB_SIZE = 8;
static constexpr uint32_t PID_RETRY_MS = 50;
static SemaphoreHandle_t s_pid_mutex = NULL;
static std::atomic<bool> s_pid_push_pending{false};

/** @brief  Read the stored PID blob from NVS into param.
 *  @param param  Output buffer of at least PID_BLOB_SIZE bytes.
 *  @retval true if a valid PID blob was read; false if missing/invalid.
 *  A missing or invalid blob is NOT sent, so the F407 keeps its defaults. */
static bool load_pid(uint8_t *param)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("esp_link", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        if (err != ESP_ERR_NVS_NOT_FOUND)
            ESP_LOGW(TAG, "PID NVS open failed: %s", esp_err_to_name(err));
        return false;
    }
    size_t size = PID_BLOB_SIZE;
    err = nvs_get_blob(handle, "pid", param, &size);
    nvs_close(handle);
    if (err != ESP_OK || size != PID_BLOB_SIZE) {
        if (err != ESP_ERR_NVS_NOT_FOUND)
            ESP_LOGW(TAG, "PID NVS read failed: %s (size=%u)",
                     esp_err_to_name(err), (unsigned)size);
        return false;  /* Missing/invalid blob is not sent; F407 defaults stay. */
    }
    return true;
}

/** @brief  Persist a PID blob into NVS.
 *  @param param  8-byte kp/ki blob.
 *  @retval true on a committed write; false otherwise. */
static bool save_pid(const uint8_t *param)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("esp_link", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PID NVS open for write failed: %s", esp_err_to_name(err));
        return false;
    }
    err = nvs_set_blob(handle, "pid", param, PID_BLOB_SIZE);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PID NVS save failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

/** @brief  Forward a PID to the F407 and save it to NVS (see the header).
 *  @param param      8-byte kp/ki little-endian blob.
 *  @param generation Generation the producer captured.
 *  @retval true if the PID was forwarded/queued (NOT an F407 execution confirmation). */
bool esp_link_send_pid_and_save(const uint8_t *param, uint32_t generation)
{
    if (param == NULL || s_pid_mutex == NULL) return false;
    xSemaphoreTake(s_pid_mutex, portMAX_DELAY);
    const bool sent = esp_link_send_command_for_generation(ESP_CMD_PID, param,
                                                           PID_BLOB_SIZE, generation);
    if (sent && !save_pid(param))
        ESP_LOGW(TAG, "PID forwarded but not persisted; reboot will use previous PID");
    xSemaphoreGive(s_pid_mutex);
    return sent;  /* Preserves the ACK's original meaning: queued for forward, not F407 confirmation. */
}

/** @brief  Request an asynchronous NVS PID restore (see the header).
 *  @retval none */
void esp_link_request_pid_push(void)
{
    s_pid_push_pending.store(true);
    portENTER_CRITICAL(&s_safety_lock);
    const TaskHandle_t tx_task = s_tx_task;
    portEXIT_CRITICAL(&s_safety_lock);
    if (tx_task != NULL) xTaskNotify(tx_task, TX_WAKE, eSetBits);
}

/* TX-only; on failure the request is kept and retried every 50 ms while idle,
 * or together with normal queue consumption when there is work.
 * TX does NOT wait on the PID mutex, so STOP can still be handled while the
 * other core is writing flash. */
static void try_push_pid(void)
{
    static bool retry_logged = false;
    if (!s_pid_push_pending.load() || xSemaphoreTake(s_pid_mutex, 0) != pdTRUE)
        return;
    s_pid_push_pending.exchange(false);  /* A new trigger arriving after this is not cleared by this completion. */
    uint8_t param[PID_BLOB_SIZE];
    bool queued = false;
    if (load_pid(param)) {
        const uint32_t generation = esp_link_command_generation();
        queued = esp_link_resume_commands(generation) &&
                 esp_link_send_command_for_generation(ESP_CMD_PID, param,
                                                       PID_BLOB_SIZE, generation);
        if (!queued) {
            s_pid_push_pending.store(true);
            if (!retry_logged)
                ESP_LOGW(TAG, "PID restore deferred by STOP/fatal/queue; will retry");
            retry_logged = true;
        } else {
            retry_logged = false;
            ESP_LOGI(TAG, "stored PID queued for F407");
        }
    } else {
        retry_logged = false;
    }
    xSemaphoreGive(s_pid_mutex);
}

/** @brief  Latch STOP: bump the generation and set the pending STOP flag (see header). */
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

/** @brief  Fatal startup fault: latch STOP and permanently disable resume. */
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

/** @brief  Return the current safety generation. */
uint32_t esp_link_command_generation(void)
{
    portENTER_CRITICAL(&s_safety_lock);
    const uint32_t value = s_generation;
    portEXIT_CRITICAL(&s_safety_lock);
    return value;
}

/** @brief  Resume commands if the observed generation is still current.
 *  @retval true if resume was allowed. */
bool esp_link_resume_commands(uint32_t observed_generation)
{
    portENTER_CRITICAL(&s_safety_lock);
    const bool allowed = observed_generation == s_generation && !s_stop_pending && !s_fatal;
    if (allowed) s_inhibited = false;
    portEXIT_CRITICAL(&s_safety_lock);
    return allowed;
}

/* ============================================================================
   1. CRC16-CCITT (polynomial 0x1021, init 0xFFFF, table driven)
      Identical to the F407 ESP_LINK_CRC16; standard vector crc("123456789") == 0x29B1
   ============================================================================ */
static uint16_t s_crc_table[256];
static bool     s_crc_ready = false;

/** @brief  Build the 256-entry CRC lookup table once. */
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

/** @brief  Compute the CRC16-CCITT over a byte range.
 *  @param data  Bytes to hash.
 *  @param len   Number of bytes.
 *  @retval the 16-bit CRC. */
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
   2. RX: bounded sliding candidate frame; a CRC failure keeps the suffix so a
      continuous stream can resynchronize
   ============================================================================ */
static uint8_t s_rx_buf[ESP_LINK_FRAME_OVH + ESP_LINK_RX_PAYLOAD_SIZE];
static size_t s_rx_count = 0;
static uint32_t s_snapshot_seq = 0;    /* Updated only by the RX task; does not change the F407 wire protocol. */

/** @brief  One complete, CRC-valid frame arrived: parse the SensorData into the snapshot queue.
 *  @param payload_len  Payload length (must equal sizeof(esp_link_sensor_t)). */
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
    /* Use the same FreeRTOS monotonic ms time base as macro/telemetry; only a
     * complete, valid frame is recorded. */
    snapshot.received_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    snapshot.seq = ++s_snapshot_seq;
    xQueueOverwrite(esp_link_sensor_queue, &snapshot);
    /* Only the first CRC+length-valid sensor frame triggers this; a separate
     * flag avoids re-triggering when seq wraps. */
    static bool first_sensor_received = false;
    if (!first_sensor_received) {
        first_sensor_received = true;
        esp_link_request_pid_push();
    }
}

/** @brief  Create the esp_link queues/mutex/event group (idempotent). */
void esp_link_queues_init(void)
{
    if (esp_link_sensor_queue == NULL)
        esp_link_sensor_queue = xQueueCreate(1, sizeof(esp_link_snapshot_t));
    if (esp_link_cmd_queue == NULL)
        esp_link_cmd_queue = xQueueCreate(8, sizeof(esp_link_cmd_t));
    if (s_uart_event == NULL) s_uart_event = xEventGroupCreate();
    if (s_pid_mutex == NULL) s_pid_mutex = xSemaphoreCreateMutex();
    configASSERT(esp_link_sensor_queue && esp_link_cmd_queue && s_uart_event && s_pid_mutex);
    crc_build_table();
}

/** @brief  RX task: initialize UART, then parse the incoming sensor frame stream. */
void esp_link_rx_task(void *arg)
{
    static_assert(sizeof(esp_link_sensor_t) == 116,
                  "SensorData_t must be 116 bytes (PROTOCOL.md §1.2)");

    ESP_LOGI(TAG, "rx task starting");

    /* UART init (8-N-1, no flow control, matches F407 USART2). */
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

    /* 3. Byte-by-byte parse main loop. */
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

/* Name of the most recently sent command (drives the net stats dashboard):
   atomic<const char*> is cross-core safe; the pointer points at static string
   literals whose lifetime is the program.
   The ID -> name table must stay in sync with cmd_parser.cpp's k_cmd_table
   (the name -> ID direction). */
static std::atomic<const char *> s_last_cmd_name{nullptr};

/** @brief  Map a command ID to its name.
 *  @param id  Command ID (ESP_CMD_*).
 *  @retval static string name, or NULL for an unknown ID. */
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

/** @brief  Return the name of the last command successfully sent to the F407. */
const char *esp_link_last_command_name(void)
{
    return s_last_cmd_name.load();
}

/* ============================================================================
   3. TX: command queue -> command frame (one command per frame, then a pause
      to trigger the F407 IDLE framing)
   ============================================================================ */
/** @brief  Pack and send one command frame over the UART.
 *  @param cmd  Command frame to send.
 *  @retval true if the frame was fully written and the UART drained. */
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
    if (ok) s_last_cmd_name.store(cmd_name_for(cmd.cmd_id));   /* Dashboard shows it until the next command updates. */
    return ok;
}

/* Link-loss deadman heartbeat: send one LEN=0 empty frame (SOF + LEN(0) + CRC).
   On the F407, a passing CRC refreshes its deadman watchdog (an empty frame
   triggers no command); the S3 sends one every 200 ms. */
static bool transmit_heartbeat(void)
{
    uint8_t frame[ESP_LINK_FRAME_OVH];
    frame[0] = ESP_LINK_SOF0;
    frame[1] = ESP_LINK_SOF1;
    frame[2] = 0;                          /* LEN = 0: empty heartbeat frame. */
    const uint16_t crc = esp_link_crc16(frame, 3);
    frame[3] = (uint8_t)crc;
    frame[4] = (uint8_t)(crc >> 8);
    const int sent = uart_write_bytes(PEO_PM_ESP_UART, frame, sizeof(frame));
    const esp_err_t drained = uart_wait_tx_done(PEO_PM_ESP_UART, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(2));          /* Triggers the F407 USART2 IDLE framing. */
    return sent == (int)sizeof(frame) && drained == ESP_OK;
}

/** @brief  TX task: consume the command queue and pace frame transmission. */
void esp_link_tx_task(void *arg)
{
    portENTER_CRITICAL(&s_safety_lock);
    s_tx_task = xTaskGetCurrentTaskHandle();
    portEXIT_CRITICAL(&s_safety_lock);
    /* STOP/commands arriving before registration are not lost: right after
     * becoming ready we check the latch state and the queue. */
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
        /* STOP has priority; every round also checks the restore request, so a
         * continuous command stream cannot starve the PID restore.
         * The restore still travels through the normal queue; ordering with a
         * manual PID is guaranteed by s_pid_mutex. */
        try_push_pid();
        esp_link_cmd_t cmd;
        if (xQueueReceive(esp_link_cmd_queue, &cmd, 0) != pdTRUE) {
            /* Queue first, notification is only a wake signal. A successful
             * enqueue and STOP both send a notification, so work arriving between
             * "saw empty queue" and "entered wait" can still wake this task.
             * After clearing the notification we loop back, re-check STOP, then
             * poll the queue non-blocking; even if notifications were merged or
             * stale, we neither skip the queue nor miss a wake-up.
             * Link-loss deadman: 200 ms without command/notification sends an
             * empty heartbeat frame to the F407 (feeding the deadman); with a
             * PID restore pending this shrinks to 50 ms so a temporarily
             * inhibited resume / busy queue can retry soon. */
            const uint32_t wait_ms = s_pid_push_pending.load() ? PID_RETRY_MS : 200;
            if (xTaskNotifyWait(0, UINT32_MAX, NULL, pdMS_TO_TICKS(wait_ms)) == pdFALSE)
                transmit_heartbeat();
            continue;
        }
        portENTER_CRITICAL(&s_safety_lock);
        /* The single send commit point: passing it means committed, so a later
         * STOP can at most follow this frame. UART write/wait are always outside
         * the lock, avoiding blocking during cross-core spin. */
        const bool allowed = !s_inhibited && cmd.generation == s_generation;
        portEXIT_CRITICAL(&s_safety_lock);
        if (allowed) {
            if (!transmit_command(cmd)) {
                ESP_LOGE(TAG, "UART TX failed; requesting STOP");
                esp_link_request_stop();
                if (cmd.cmd_id == ESP_CMD_PID) esp_link_request_pid_push();
            }
        } else if (cmd.cmd_id == ESP_CMD_PID) {
            /* If a queued PID is invalidated by a newer STOP, re-read the latest
             * NVS value; the restore request must not be lost. */
            ESP_LOGW(TAG, "queued PID invalidated by STOP; scheduling restore");
            esp_link_request_pid_push();
        }
    }
}

/** @brief  Enqueue a command only if the producer's captured epoch is still current (see header). */
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
        /* Read the handle after publishing: if TX only registers during the
         * enqueue, we must still notify it. If it has not registered yet, the TX
         * entry checks the published queue, so work is never lost either way.
         * TX always reads the real queue; the number of notifications is not the
         * number of commands. */
        portENTER_CRITICAL(&s_safety_lock);
        const TaskHandle_t tx_task = s_tx_task;
        portEXIT_CRITICAL(&s_safety_lock);
        if (tx_task != NULL) xTaskNotify(tx_task, TX_WAKE, eSetBits);
        return true;
    }
    return false;
}
