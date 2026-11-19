/**
 * @file  camera_service.cpp
 * @brief OV2640 capture task (JPEG -> TCP 8890, 24 fps application pacing).
 *
 * Copyright (c) 2026 Danny
 * SPDX-License-Identifier: MIT
 */

/*
 * Camera pins come from PEO_PINMAP.h (PEO_PM_CAM_*, user-defined).
 * Resolution: starts at QVGA (320x240) and supports hot-swapping to
 * QQVGA/QVGA/VGA/SVGA via the CAM_RES command (PROTOCOL section 2).
 */

#include "camera_service.h"
#include "esp_net.h"
#include "PEO_PINMAP.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <esp_camera.h>
#include <esp_psram.h>
#include <esp_timer.h>
#include <esp_private/esp_gpio_reserve.h>
#include <string.h>

static const char *TAG = "camera";

static QueueHandle_t s_cam_cmd_queue = NULL;   /* Resolution request queue (1-deep overwrite, keeps only the latest). */

bool camera_queues_init(void)
{
    if (!s_cam_cmd_queue)
        s_cam_cmd_queue = xQueueCreate(1, sizeof(uint8_t));
    return s_cam_cmd_queue != NULL;
}

bool camera_resolution_from_name(const char *name, uint8_t *out)
{
    if (!name || !out) return false;
    if (strcmp(name, "QQVGA") == 0) { *out = FRAMESIZE_QQVGA; return true; }
    if (strcmp(name, "QVGA")  == 0) { *out = FRAMESIZE_QVGA;  return true; }
    if (strcmp(name, "VGA")   == 0) { *out = FRAMESIZE_VGA;   return true; }
    if (strcmp(name, "SVGA")  == 0) { *out = FRAMESIZE_SVGA;  return true; }
    return false;
}

bool camera_request_resolution(uint8_t framesize)
{
    if (!s_cam_cmd_queue) return false;  /* Requests are also accepted while on standby, for the next re-init. */
    return xQueueOverwrite(s_cam_cmd_queue, &framesize) == pdTRUE;
}

/* The module has its own XCLK: pin_xclk=GPIO_NUM_NC means the ESP32 drives no
   clock (esp32-camera convention); xclk_freq_hz must still equal the module
   crystal frequency (used by the OV2640 PLL; standard 20 MHz). */
#define CAM_XCLK_HZ         20000000

/** @brief  Target capture rate for a resolution (frames per second).
 *  @param fs  framesize (unused: all resolutions share one locked rate).
 *  @retval the target fps.
 *  This only limits the application-side capture; the sensor still free-runs and
 *  GRAB_LATEST evicts the surplus frames. The global rate is locked at 24 fps
 *  (user decision 2026-11-18, replacing the old per-size 30/30/24 grading):
 *  - QVGA 24 fps ~0.96 Mbps, SVGA 24 fps ~2.7 Mbps (near the ceiling, judged by EAGAIN);
 *  - multi-window testing was stable in the 15 fps era; 24 fps is the chosen
 *    balance of quality / frame rate / bandwidth.
 *  The period rounds up so integer truncation cannot push the rate higher than intended. */
static int cam_fps_for(framesize_t fs)
{
    (void)fs;
    return 24;   /* All resolutions use the same value: locked at 24 fps. */
}
static int64_t period_us_for_fps(int fps)
{
    return (1000000LL + fps - 1) / fps;
}
/* Runtime frame period. Recalculated after every cfg.frame_size update (boot
   default, switch, retry, rollback) — unconditionally, so the "no switch -> no
   update" gap cannot happen. The initial value is the 15 fps period, replaced on
   the first apply. */
static int64_t s_frame_period_us = period_us_for_fps(15);
static void apply_frame_period(framesize_t fs)
{
    s_frame_period_us = period_us_for_fps(cam_fps_for(fs));
}

/** @brief  Frame-pacing timer callback: only wakes the camera task.
 *  @param arg  The binary semaphore to give.
 *  @retval none
 *  Runs on ESP_TIMER_TASK; it must not capture or send here. A dedicated binary
 *  semaphore avoids clashing with task notifications used by the driver or other
 *  call chains. */
static void cam_frame_timer_cb(void *arg)
{
    xSemaphoreGive((SemaphoreHandle_t)arg);
}

/** @brief  Wait until the absolute deadline of the next frame.
 *  @param timer          One-shot timer used to wake this task.
 *  @param ready          Binary semaphore given by the timer callback.
 *  @param next_frame_us  (in/out) Absolute monotonic deadline; advanced on return.
 *  @retval ESP_OK, or the timer error (never silently degrades to uncapped capture).
 *  esp_timer_get_time() is the monotonic microsecond clock; the one-shot timer
 *  wakes the task through the semaphore without truncating the remaining time to
 *  a FreeRTOS tick and without busy-waiting (which would starve the network task).
 *  Normally the deadline advances by a fixed period so processing/wake latency
 *  never accumulates per frame; if a capture/processing already ran past the
 *  deadline, or a wake crossed into the next period, the cadence is rebuilt from
 *  now instead of chasing missed frames. */
static esp_err_t cam_wait_next_frame(esp_timer_handle_t timer, SemaphoreHandle_t ready,
                                     int64_t *next_frame_us)
{
    int64_t now_us = esp_timer_get_time();
    if (now_us >= *next_frame_us)
    {
        *next_frame_us = now_us + s_frame_period_us;
        return ESP_OK;
    }

    do
    {
        esp_err_t err = esp_timer_start_once(timer, (uint64_t)(*next_frame_us - now_us));
        if (err != ESP_OK)
            return err;   /* A timer failure must not silently degrade into uncapped capture. */
        xSemaphoreTake(ready, portMAX_DELAY);
        now_us = esp_timer_get_time();   /* Re-check the deadline after waking so a spurious wakeup cannot break the cadence. */
    } while (now_us < *next_frame_us);

    *next_frame_us += s_frame_period_us;
    if (*next_frame_us <= now_us)
        *next_frame_us = now_us + s_frame_period_us;
    return ESP_OK;
}

/** @brief  JPEG quality graded by resolution.
 *  @param fs  framesize.
 *  @retval the esp32-camera quality value (0-63; lower = higher quality).
 *  Quality is graded by resolution to control frame size/bandwidth while keeping
 *  airtime headroom for commands and telemetry; real bandwidth varies with scene
 *  content, so the VGA/SVGA values keep their current (measured) parameters. */
static uint8_t cam_quality_for(framesize_t fs)
{
    switch (fs)
    {
    case FRAMESIZE_QQVGA:
    case FRAMESIZE_QVGA:  return 25;   /* q25: quality first; q35 measured only 0.47 Mbps, ample headroom. */
    case FRAMESIZE_VGA:   return 30;   /* Estimated ~1.1 Mbps, scaled down from the SVGA measurement. */
    case FRAMESIZE_SVGA:  return 35;   /* q40 measured 1.55 Mbps, +0 EAGAIN; q35 estimated ~1.9 Mbps — watch EAGAIN after switching. */
    default:              return 30;
    }
}

/** @brief  JPEG frame buffer size adapted to the resolution (user decision 2026-11-17).
 *  @param fs  framesize.
 *  @retval buffer size in bytes.
 *  Each tier = the measured maximum frame size plus headroom. Never leave it to
 *  the driver default (w*h/5 underestimates: QQVGA at 3.8 KB measured a DMA
 *  overflow). A hot-swap sets cfg.frame_size before init, so the buffer tracks it
 *  naturally. The unmeasured tier (SVGA) takes a conservative value. */
static size_t cam_buffer_for(framesize_t fs)
{
    switch (fs)
    {
    case FRAMESIZE_QQVGA: return 16 * 1024;   /* Measured 2-5 KB. */
    case FRAMESIZE_QVGA:  return 32 * 1024;   /* Measured 10-20 KB. */
    case FRAMESIZE_VGA:   return 64 * 1024;   /* Measured 9-60 KB (complex scenes graze the edge). */
    case FRAMESIZE_SVGA:  return 96 * 1024;   /* Not measured yet, conservative; calibrate on first high-texture run. */
    default:              return 64 * 1024;
    }
}

/** @brief  Release the camera control GPIOs after deinit.
 *  @retval none
 *  esp_camera_deinit() does not undo gpio_config's output-pin reservation — IDF
 *  v6.1's gpio_config auto-reserves output pins, so a hot-swap re-init reports
 *  "gpio: conflict found for GPIO[PWDN/RST]". After deinit we actively revoke the
 *  control pins (PWDN/RST; the data/sync lines are inputs and never reserved). */
static void cam_release_control_gpios(void)
{
    uint64_t mask = 0;
    if (PEO_PM_CAM_PWDN >= 0 && PEO_PM_CAM_PWDN < 64) mask |= 1ULL << PEO_PM_CAM_PWDN;
    if (PEO_PM_CAM_RST  >= 0 && PEO_PM_CAM_RST  < 64) mask |= 1ULL << PEO_PM_CAM_RST;
    if (mask) esp_gpio_revoke(mask);
}

/** @brief  Hard-reset the camera sensor.
 *  @retval none
 *  esp_camera_deinit only stops DMA; it does NOT stop the OV2640 from streaming
 *  (the module's own XCLK keeps it outputting). In streaming mode the OV2640
 *  NAKs writes to its timing registers, so set_framesize on re-init fails — the
 *  log signature is: probe reads the PID fine, but "W [3804]=0a fail". Toggling
 *  RST+PWDN forces the sensor back to its default state so the next init is clean. */
static void cam_sensor_hard_reset(void)
{
    if (PEO_PM_CAM_RST >= 0 && PEO_PM_CAM_RST < 64)
    {
        gpio_set_direction((gpio_num_t)PEO_PM_CAM_RST, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)PEO_PM_CAM_RST, 0);    /* RESETB active low: assert reset. */
    }
    if (PEO_PM_CAM_PWDN >= 0 && PEO_PM_CAM_PWDN < 64)
    {
        gpio_set_direction((gpio_num_t)PEO_PM_CAM_PWDN, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)PEO_PM_CAM_PWDN, 1);   /* PWDN active high: power down. */
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (PEO_PM_CAM_PWDN >= 0 && PEO_PM_CAM_PWDN < 64)
        gpio_set_level((gpio_num_t)PEO_PM_CAM_PWDN, 0);   /* Power up. */
    if (PEO_PM_CAM_RST >= 0 && PEO_PM_CAM_RST < 64)
        gpio_set_level((gpio_num_t)PEO_PM_CAM_RST, 1);    /* Release reset. */
    vTaskDelay(pdMS_TO_TICKS(20));   /* The OV2640 needs several ms after reset before it answers SCCB. */
    cam_release_control_gpios();     /* Clear any output reserve a previous gpio_config left; gpio_set_direction does not reserve. */
}

void camera_task(void *arg)
{
    bool active = false;  /* Task-local: false = driver deinitialized, waiting for a new request to self-heal. */
    camera_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.pin_pwdn       = PEO_PM_CAM_PWDN;
    cfg.pin_reset      = PEO_PM_CAM_RST;
    cfg.pin_xclk       = PEO_PM_CAM_XCLK;
    cfg.pin_sccb_sda   = PEO_PM_CAM_SDA;
    cfg.pin_sccb_scl   = PEO_PM_CAM_SCL;
    cfg.pin_d0         = PEO_PM_CAM_D0;
    cfg.pin_d1         = PEO_PM_CAM_D1;
    cfg.pin_d2         = PEO_PM_CAM_D2;
    cfg.pin_d3         = PEO_PM_CAM_D3;
    cfg.pin_d4         = PEO_PM_CAM_D4;
    cfg.pin_d5         = PEO_PM_CAM_D5;
    cfg.pin_d6         = PEO_PM_CAM_D6;
    cfg.pin_d7         = PEO_PM_CAM_D7;
    cfg.pin_vsync      = PEO_PM_CAM_VS;
    cfg.pin_href       = PEO_PM_CAM_HS;
    cfg.pin_pclk       = PEO_PM_CAM_PCLK;
    cfg.xclk_freq_hz   = CAM_XCLK_HZ;
    cfg.ledc_timer     = LEDC_TIMER_0;
    cfg.ledc_channel   = LEDC_CHANNEL_0;
    cfg.pixel_format   = PIXFORMAT_JPEG;
    cfg.frame_size     = FRAMESIZE_QVGA;
    cfg.jpeg_buffer_size = cam_buffer_for(FRAMESIZE_QVGA);   /* Sized per resolution (see cam_buffer_for). */
    cfg.jpeg_quality   = cam_quality_for(FRAMESIZE_QVGA);   /* Graded per resolution (see cam_quality_for). */
    cfg.fb_count       = 3;    /* 3 buffers + GRAB_LATEST (user decision 2026-11-17):
                                 2 queue slots + 1 driver-dedicated DMA/encode buffer;
                                 when the queue is full the oldest is evicted for the newest —
                                 the app always gets the latest frame (low latency) at the cost
                                 of the driver capturing at full rate and old frames being dropped. */
    cfg.fb_location    = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode      = CAMERA_GRAB_LATEST;   /* Always latest: evict-and-return fb, never stop capturing. */

    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (!s_cam_cmd_queue)
        ESP_LOGE(TAG, "camera command queue unavailable; camera inactive; awaiting CAM_RES retry");
    else if (!esp_psram_is_initialized())
        ESP_LOGE(TAG, "N8R8 PSRAM unavailable; camera inactive; awaiting CAM_RES retry");
    else
    {
        err = esp_camera_init(&cfg);
        active = (err == ESP_OK);
        if (!active)
            ESP_LOGE(TAG, "camera init failed (0x%x); camera inactive; awaiting CAM_RES retry", err);
    }
    if (!active)
    {
        esp_net_video_flush();
        esp_camera_deinit();
        cam_release_control_gpios();
    }
    else
    {
        ESP_LOGI(TAG, "camera up: OV2640 QVGA JPEG");
        /* Return immediately after the boot diagnostic: in the idle state (no client / video off) the camera is always deinitialized. */
        esp_net_video_flush();
        esp_camera_deinit();
        cam_release_control_gpios();
        active = false;
    }


    EventGroupHandle_t eg = esp_net_conn_event();
    if (!eg)
    {
        ESP_LOGE(TAG, "connection event group missing; task exits");
        vTaskDelete(NULL);
        return;
    }

    /* Created once per task lifetime; while on standby the one-shot timer has
       expired, so no periodic callback idles. The camera is already deinitialized
       here, so a resource-allocation failure can clean up and exit directly. */
    SemaphoreHandle_t frame_ready = xSemaphoreCreateBinary();
    esp_timer_handle_t frame_timer = NULL;
    esp_timer_create_args_t timer_args = {};
    timer_args.callback = cam_frame_timer_cb;
    timer_args.arg = frame_ready;
    timer_args.dispatch_method = ESP_TIMER_TASK;
    timer_args.name = "cam_frame";
    esp_err_t pace_err = frame_ready ? esp_timer_create(&timer_args, &frame_timer) : ESP_ERR_NO_MEM;
    if (pace_err != ESP_OK)
    {
        ESP_LOGE(TAG, "camera pacing init failed (0x%x); task exits", (int)pace_err);
        if (frame_ready) vSemaphoreDelete(frame_ready);
        vTaskDelete(NULL);
        return;
    }
    int64_t next_frame_us = 0;   /* 0 = start a fresh capture round: the first frame is taken and sent immediately. */

    for (;;)
    {
        /* Gate: no client or video disabled -> block waiting (the idle camera is
           always deinitialized). CAM_RES stays in the overwrite queue, and the
           latest request is applied before restoring init. */
        xEventGroupWaitBits(eg, PEO_NET_EVT_CLIENT | PEO_NET_EVT_VIDEO,
                            pdFALSE, pdTRUE, portMAX_DELAY);
        if (!active)
        {
            uint8_t req = (uint8_t)cfg.frame_size;
            uint8_t newer;
            while (s_cam_cmd_queue && xQueueReceive(s_cam_cmd_queue, &newer, 0) == pdTRUE)
                req = newer;   /* Apply the latest request even before retrying a failed init. */
            if (req != cfg.frame_size)
            {
                cfg.frame_size = (framesize_t)req;
                cfg.jpeg_quality = cam_quality_for(cfg.frame_size);
                cfg.jpeg_buffer_size = cam_buffer_for(cfg.frame_size);   /* Re-size for the new resolution. */
            }
            apply_frame_period(cfg.frame_size);   /* Unconditional sync: the boot default must apply its graded frame rate even without a switch. */
            /* Recover from idle: hard reset (the sensor is still streaming after deinit), then re-init. */
            cam_sensor_hard_reset();
            esp_err_t e = esp_psram_is_initialized()
                        ? esp_camera_init(&cfg) : ESP_ERR_INVALID_STATE;
            if (e != ESP_OK)
            {
                ESP_LOGE(TAG, "cam resume init failed (0x%x); retry in 100ms", (int)e);
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            active = true;
            next_frame_us = 0;   /* After video restarts, the first frame must not inherit the previous deadline. */
            ESP_LOGI(TAG, "camera resumed: framesize=%u", (unsigned)cfg.frame_size);
        }

        while ((xEventGroupGetBits(eg) & (PEO_NET_EVT_CLIENT | PEO_NET_EVT_VIDEO)) ==
               (PEO_NET_EVT_CLIENT | PEO_NET_EVT_VIDEO))
        {
        /* Resolution hot-swap: deinit, hard reset, then init reallocates the
           frame buffers. On a first failure retry once with the latest request;
           only then roll back; if the rollback also fails, deinit and wait. */
        uint8_t req;
        if (s_cam_cmd_queue && xQueueReceive(s_cam_cmd_queue, &req, 0) == pdTRUE)
        {
            uint8_t newer;
            while (xQueueReceive(s_cam_cmd_queue, &newer, 0) == pdTRUE)
                req = newer;   /* Apply only the latest request. */
            if (!active || req != cfg.frame_size)
            {
                framesize_t old_size = cfg.frame_size;
                ESP_LOGI(TAG, "cam res switch %u -> %u", (unsigned)old_size, (unsigned)req);
                cfg.frame_size = (framesize_t)req;
                cfg.jpeg_quality = cam_quality_for(cfg.frame_size);   /* Quality grades with resolution, controlling frame size/bandwidth. */
                cfg.jpeg_buffer_size = cam_buffer_for(cfg.frame_size);   /* Buffer adapts to the resolution. */
                apply_frame_period(cfg.frame_size);
                active = false;
                esp_net_video_flush();
                esp_camera_deinit();
                cam_release_control_gpios();
                cam_sensor_hard_reset();
                esp_err_t e = esp_psram_is_initialized() ? esp_camera_init(&cfg) : ESP_ERR_INVALID_STATE;
                if (e == ESP_OK)
                {
                    active = true;
                    ESP_LOGI(TAG, "cam res applied: %u", (unsigned)req);
                }
                else
                {
                    ESP_LOGW(TAG, "cam res %u init failed (0x%x), retry once",
                             (unsigned)req, (int)e);
                    /* Reuse newer to record whether a new request arrived meanwhile;
                       when the queue is empty req keeps the original target. Retry only
                       once, merging "restore service" and "apply latest request" to avoid
                       repeated inits. */
                    newer = 0;
                    while (xQueueReceive(s_cam_cmd_queue, &req, 0) == pdTRUE)
                        newer = 1;
                    ESP_LOGI(TAG, "cam res retry target: %u (%s)", (unsigned)req,
                             newer ? "newer request" : "original request");
                    cfg.frame_size = (framesize_t)req;
                    cfg.jpeg_quality = cam_quality_for(cfg.frame_size);
                    apply_frame_period(cfg.frame_size);
                    esp_net_video_flush();
                    esp_camera_deinit();   /* Clean up the half-configured state of the failed init. */
                    cam_release_control_gpios();
                    cam_sensor_hard_reset();
                    e = esp_psram_is_initialized() ? esp_camera_init(&cfg) : ESP_ERR_INVALID_STATE;
                    if (e == ESP_OK)
                    {
                        active = true;
                        ESP_LOGI(TAG, "cam res retry applied%s: %u",
                                 newer ? " (newer)" : "", (unsigned)req);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "cam res %u retry init failed (0x%x), revert to %u",
                                 (unsigned)req, (int)e, (unsigned)old_size);
                        cfg.frame_size = old_size;
                        cfg.jpeg_quality = cam_quality_for(cfg.frame_size);
                        cfg.jpeg_buffer_size = cam_buffer_for(cfg.frame_size);   /* Revert: sync the buffer. */
                        apply_frame_period(cfg.frame_size);   /* Revert: sync the frame rate. */
                        esp_net_video_flush();
                        esp_camera_deinit();   /* Clean up the half-configured state of the failed retry. */
                        cam_release_control_gpios();
                        cam_sensor_hard_reset();
                        e = esp_psram_is_initialized() ? esp_camera_init(&cfg) : ESP_ERR_INVALID_STATE;
                        if (e != ESP_OK)
                        {
                            ESP_LOGE(TAG, "camera revert init failed (0x%x); camera inactive; awaiting CAM_RES retry", e);
                            esp_net_video_flush();
                            esp_camera_deinit();
                            cam_release_control_gpios();
                        }
                        else
                        {
                            active = true;
                            ESP_LOGI(TAG, "cam res reverted: %u", (unsigned)old_size);
                        }
                    }
                }
                next_frame_us = 0;   /* After a hot-swap/rollback, send the first frame immediately; do not chase the frames missed during the stop. */
            }
        }

        if (!active)
            break;   /* All resolution-switch attempts failed and we deinitialized: leave the capture loop and wait for self-heal at the gate. */

        camera_fb_t *fb = esp_camera_fb_get();
        if (next_frame_us == 0)
            next_frame_us = esp_timer_get_time() + s_frame_period_us;
        if (fb != NULL && !esp_net_video_enabled())
        {
            esp_camera_fb_return(fb);   /* Video off: capture and discard (stats moved to the network layer). */
        }
        else if (fb != NULL)
        {
            /* Zero-copy enqueue: ownership transfers to esp_net, which returns
               the frame. TCP guarantees integrity, so no application-layer CRC. */
            esp_net_video_set_frame_owner(fb);
            if (!esp_net_send_video(fb->buf, fb->len))
                esp_camera_fb_return(fb);   /* Not enqueued: ownership stays with camera. */
        }

        /* One common epilogue for every capture result: successful enqueue,
           failed return, video-off skipped, empty frame. While waiting, this task
           holds no fb; the first frame is sent without waiting, and the next round
           takes from the driver again, letting GRAB_LATEST evict normally between
           frames. After video turns off, the original gate exits and deinitializes. */
        pace_err = cam_wait_next_frame(frame_timer, frame_ready, &next_frame_us);
        if (pace_err != ESP_OK)
        {
            ESP_LOGE(TAG, "camera pacing wait failed (0x%x); task exits", (int)pace_err);
            break;   /* Take the common exit path: return frames held by esp_net, then free the pacing resources. */
        }
        }

        /* Exit (disconnect / video off / switch all failed): full deinit, returning the PSRAM frame buffers and GPIO reserves. */
        esp_net_video_flush();
        esp_camera_deinit();
        cam_release_control_gpios();
        active = false;
        ESP_LOGI(TAG, "cam idle: deinitialized (frames released)");
        if (pace_err != ESP_OK)
            break;
    }

    /* IDF v6.1: stop and wait for a callback that may still run on the other core before deleting the semaphore. */
    esp_timer_stop_blocking(frame_timer, portMAX_DELAY);
    esp_timer_delete(frame_timer);
    vSemaphoreDelete(frame_ready);
    vTaskDelete(NULL);
}
