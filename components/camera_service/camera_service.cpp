/**
 * @file    camera_service.cpp
 * @brief   PEO Rover - OV5640 抓帧任务（默认目标 24fps JPEG -> UDP 8889）
 *
 * 引脚来自 PEO_PINMAP.h（PEO_PM_CAM_*，用户定义）。
 * 帧率/分辨率：QVGA(320x240) 起步（PROTOCOL.md §5），稳定后可提 VGA。
 */

#include "camera_service.h"
#include "esp_net.h"
#include "PEO_PINMAP.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <esp_camera.h>
#include <esp_psram.h>
#include <esp_timer.h>
#include <esp_private/esp_gpio_reserve.h>
#include <string.h>

static const char *TAG = "camera";

static QueueHandle_t s_cam_cmd_queue = NULL;   /* 分辨率请求队列（1 深覆盖，只保留最新） */
static volatile bool s_camera_ready = false;  /* 初始化成功后接收请求，任务退出前撤销就绪。 */

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
    if (!s_cam_cmd_queue || !s_camera_ready) return false;
    return xQueueOverwrite(s_cam_cmd_queue, &framesize) == pdTRUE;
}

/* 模组自带 XCLK：pin_xclk=GPIO_NUM_NC 时 ESP32 不输出时钟（esp32-camera 约定）；
   xclk_freq_hz 仍须等于模组晶振实际频率（OV5640 PLL 计算用）。
   TODO(用户确认)：常见 20MHz / 24MHz，按模组实际晶振填写 */
#define CAM_XCLK_HZ         24000000
#define CAM_TARGET_FPS_DEFAULT 24   /* 默认目标帧率；入队失败时通过额外补差退避。 */
#define CAM_TX_FAIL_BACKOFF_MS 200  /* 连续入队失败时额外补差，给网络恢复时间。 */
#define CAM_TX_FAIL_BACKOFF_FRAMES 3 /* 连续入队失败达到此帧数后启用退避。 */

static uint32_t s_consecutive_tx_fail = 0;  /* 仅 camera_task 更新，成功入队即清零。 */

static int64_t cam_target_interval_us(void)
{
    int64_t interval_us = (int64_t)(1000000u / CAM_TARGET_FPS_DEFAULT);
    if (s_consecutive_tx_fail >= CAM_TX_FAIL_BACKOFF_FRAMES)
        interval_us += (int64_t)CAM_TX_FAIL_BACKOFF_MS * 1000;
    return interval_us;
}

/* JPEG 帧缓冲容量：esp32-camera 默认公式 w*h/5 对小分辨率太紧
   （QQVGA 只算出 3.8KB，q30 JPEG 实际 3-6KB → DMA overflow / FB-SIZE overrun）。
   按分辨率给 max(w*h/4, 32KB)，实测余量充足且不浪费 PSRAM。 */
/* JPEG 质量按分辨率分级：带宽预算固定（24fps 下 ≤ ~4Mbps），
   像素越高压缩越激进。基准 = QVGA@25（并行会话实测定案，链路稳定）。
   VGA/SVGA 在 25 下实测 sendto ENOBUFS 频繁（空口饱和），需让步画质。 */
static uint8_t cam_quality_for(framesize_t fs)
{
    switch (fs)
    {
    case FRAMESIZE_QQVGA:
    case FRAMESIZE_QVGA:  return 25;   /* 基线：帧 ~10-20KB，1.9-3.8Mbps */
    case FRAMESIZE_VGA:   return 40;   /* 4 倍像素，帧压到 ~15-25KB */
    case FRAMESIZE_SVGA:  return 50;   /* 6 倍像素，画质让步换带宽 */
    default:              return 30;
    }
}

static size_t cam_jpeg_buffer_for(framesize_t fs)
{
    uint32_t w = 320, h = 240;
    switch (fs)
    {
    case FRAMESIZE_QQVGA: w = 160; h = 120; break;
    case FRAMESIZE_QVGA:  w = 320; h = 240; break;
    case FRAMESIZE_VGA:   w = 640; h = 480; break;
    case FRAMESIZE_SVGA:  w = 800; h = 600; break;
    default: break;
    }
    size_t buf = (size_t)w * h / 4;
    return buf < 32 * 1024 ? 32 * 1024 : buf;
}

/* esp_camera_deinit() 不会撤销 gpio_config 对输出引脚的保留——IDF v6.1 的
   gpio_config 对输出脚自动 esp_gpio_reserve，热切换再 init 会报
   "gpio: conflict found for GPIO[PWDN/RST]"。deinit 后主动 revoke 相机控制脚
   （PWDN/RST；数据线/同步线是输入配置，不会被保留）。 */
static void cam_release_control_gpios(void)
{
    uint64_t mask = 0;
    if (PEO_PM_CAM_PWDN >= 0 && PEO_PM_CAM_PWDN < 64) mask |= 1ULL << PEO_PM_CAM_PWDN;
    if (PEO_PM_CAM_RST  >= 0 && PEO_PM_CAM_RST  < 64) mask |= 1ULL << PEO_PM_CAM_RST;
    if (mask) esp_gpio_revoke(mask);
}

/* 传感器硬复位：esp_camera_deinit 只停 DMA，不会让 OV5640 停流（XCLK 模组自带，
   传感器持续出流）。流模式下 OV5640 对时序寄存器（0x3804/0x3811 等）写操作 NAK，
   重新 init 时 set_framesize 必然失败——日志铁证：probe 读 PID 正常但
   "W [3804]=0a fail"。复位 RST+PWDN 强制传感器回默认态，保证下次 init 干净。 */
static void cam_sensor_hard_reset(void)
{
    if (PEO_PM_CAM_RST >= 0 && PEO_PM_CAM_RST < 64)
    {
        gpio_set_direction((gpio_num_t)PEO_PM_CAM_RST, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)PEO_PM_CAM_RST, 0);    /* RESETB 低有效：复位 */
    }
    if (PEO_PM_CAM_PWDN >= 0 && PEO_PM_CAM_PWDN < 64)
    {
        gpio_set_direction((gpio_num_t)PEO_PM_CAM_PWDN, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)PEO_PM_CAM_PWDN, 1);   /* PWDN 高有效：掉电 */
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (PEO_PM_CAM_PWDN >= 0 && PEO_PM_CAM_PWDN < 64)
        gpio_set_level((gpio_num_t)PEO_PM_CAM_PWDN, 0);   /* 上电 */
    if (PEO_PM_CAM_RST >= 0 && PEO_PM_CAM_RST < 64)
        gpio_set_level((gpio_num_t)PEO_PM_CAM_RST, 1);    /* 释放复位 */
    vTaskDelay(pdMS_TO_TICKS(20));   /* OV5640 复位后需数 ms 才能应答 SCCB */
    cam_release_control_gpios();     /* 撤销 gpio_set_direction 产生的输出保留，避免 init conflict 警告 */
}

void camera_task(void *arg)
{
    if (s_cam_cmd_queue == NULL) {
        ESP_LOGE(TAG, "camera command queue unavailable; camera disabled");
        esp_net_video_flush();
        vTaskDelete(NULL);
        return;
    }
    if (!esp_psram_is_initialized()) {
        ESP_LOGE(TAG, "N8R8 PSRAM unavailable: camera disabled; control/network remain online");
        esp_net_video_flush();
        vTaskDelete(NULL);
        return;
    }
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
    cfg.jpeg_buffer_size = cam_jpeg_buffer_for(FRAMESIZE_QVGA);   /* 默认公式对小分辨率太紧，见 cam_jpeg_buffer_for */
    cfg.jpeg_quality   = cam_quality_for(FRAMESIZE_QVGA);   /* 按分辨率分级（见 cam_quality_for） */
    cfg.fb_count       = 3;    /* 异步化后 camera 只入队不发送，归还快、周转压力小；
                                 3 块缓冲延迟上限 ~125ms（vs 4 块 ~166ms），换取更低画面延迟。
                                 若实机出现 fb 池空导致的抓帧抖动，再回调 4。 */
    cfg.fb_location    = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode      = CAMERA_GRAB_WHEN_EMPTY;

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "camera init failed (0x%x) — task exits", err);
        esp_net_video_flush();
        vTaskDelete(NULL);
        return;
    }
    s_camera_ready = true;
    ESP_LOGI(TAG, "camera up: OV5640 QVGA JPEG");

    uint32_t ok_frames = 0, bad_frames = 0, skipped_frames = 0;
    uint32_t tx_fail_frames = 0;   /* bad 细分：仅 tx=入队失败（实际发送由 esp_net 完成） */
    int64_t last_log_us = esp_timer_get_time();

    while (1)
    {
        /* 每 ~5s 输出统计：ok=入队成功，bad(tx)=入队失败；实际发送结果由 esp_net 诊断。 */
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_log_us >= 5000000LL)
        {
            float real_fps = (float)(ok_frames + bad_frames + skipped_frames) * 1000000.0f /
                             (float)(now_us - last_log_us);
            ESP_LOGI(TAG, "cam stats: ok=%lu bad=%lu(tx=%lu) skipped=%lu real_fps=%.1f",
                     (unsigned long)ok_frames, (unsigned long)bad_frames,
                     (unsigned long)tx_fail_frames, (unsigned long)skipped_frames, real_fps);
            ok_frames = bad_frames = skipped_frames = tx_fail_frames = 0;
            last_log_us = now_us;
        }
        /* 分辨率热切换：deinit 后硬复位，再 init 重新分配帧缓冲。
           首次失败后按最新请求重试一次，再失败才回退原分辨率；回退失败则退出。 */
        uint8_t req;
        if (xQueueReceive(s_cam_cmd_queue, &req, 0) == pdTRUE)
        {
            uint8_t newer;
            while (xQueueReceive(s_cam_cmd_queue, &newer, 0) == pdTRUE)
                req = newer;   /* 只应用最新一次请求 */
            if (req != cfg.frame_size)
            {
                framesize_t old_size = cfg.frame_size;
                ESP_LOGI(TAG, "cam res switch %u -> %u", (unsigned)old_size, (unsigned)req);
                cfg.frame_size = (framesize_t)req;
                cfg.jpeg_quality = cam_quality_for(cfg.frame_size);   /* 质量随分辨率分级，控制帧大小/带宽 */
                cfg.jpeg_buffer_size = cam_jpeg_buffer_for(cfg.frame_size);
                esp_net_video_flush();
                esp_camera_deinit();
                cam_release_control_gpios();
                cam_sensor_hard_reset();
                esp_err_t e = esp_camera_init(&cfg);
                if (e == ESP_OK)
                {
                    ESP_LOGI(TAG, "cam res applied: %u", (unsigned)req);
                }
                else
                {
                    ESP_LOGW(TAG, "cam res %u init failed (0x%x), retry once",
                             (unsigned)req, (int)e);
                    /* 复用 newer 记录是否合入期间的新请求；队列为空时 req 保持原目标。
                       只重试一次，把恢复服务与应用最新请求合并，避免反复 init。 */
                    newer = 0;
                    while (xQueueReceive(s_cam_cmd_queue, &req, 0) == pdTRUE)
                        newer = 1;
                    ESP_LOGI(TAG, "cam res retry target: %u (%s)", (unsigned)req,
                             newer ? "newer request" : "original request");
                    cfg.frame_size = (framesize_t)req;
                    cfg.jpeg_quality = cam_quality_for(cfg.frame_size);
                    cfg.jpeg_buffer_size = cam_jpeg_buffer_for(cfg.frame_size);
                    esp_net_video_flush();
                    esp_camera_deinit();   /* 清理失败 init 的半配置态 */
                    cam_release_control_gpios();
                    cam_sensor_hard_reset();
                    e = esp_camera_init(&cfg);
                    if (e == ESP_OK)
                    {
                        ESP_LOGI(TAG, "cam res retry applied%s: %u",
                                 newer ? " (newer)" : "", (unsigned)req);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "cam res %u retry init failed (0x%x), revert to %u",
                                 (unsigned)req, (int)e, (unsigned)old_size);
                        cfg.frame_size = old_size;
                        cfg.jpeg_quality = cam_quality_for(cfg.frame_size);
                        cfg.jpeg_buffer_size = cam_jpeg_buffer_for(cfg.frame_size);
                        esp_net_video_flush();
                        esp_camera_deinit();   /* 清理重试失败的半配置态 */
                        cam_release_control_gpios();
                        cam_sensor_hard_reset();
                        if (esp_camera_init(&cfg) != ESP_OK)
                        {
                            ESP_LOGE(TAG, "camera revert init failed; task exits");
                            s_camera_ready = false;
                            esp_net_video_flush();
                            esp_camera_deinit();
                            cam_release_control_gpios();
                            esp_net_video_flush();
                            vTaskDelete(NULL);
                            return;
                        }
                        ESP_LOGI(TAG, "cam res reverted: %u", (unsigned)old_size);
                    }
                }
            }
        }

        int64_t frame_start_us = esp_timer_get_time();   /* 节拍基准：固定 delay 会在 fb_get 阻塞/切分辨率时漂移 */
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb != NULL && !esp_net_video_enabled())
        {
            skipped_frames++;
            esp_camera_fb_return(fb);
            /* OFF 仍抓帧，不入队发送；节拍保留已有入队失败退避。 */
            int64_t remain_us = cam_target_interval_us() -
                                (esp_timer_get_time() - frame_start_us);
            if (remain_us > 0)
                vTaskDelay(pdMS_TO_TICKS(remain_us / 1000));
            continue;
        }
        if (fb != NULL)
        {
            /* 零拷贝入队：esp32-camera 组件已在首帧检查 SOI 位于 pos 0（否则丢弃），
               并已截尾到 EOI，故此处不再做任何 JPEG 校验（深层合法性由上位机
               QImage 解码兜底——权威裁判）。 */
            esp_net_video_set_frame_owner(fb);
            if (esp_net_send_video(fb->buf, fb->len))
            {
                ok_frames++;   /* 所有权已交给 esp_net，此处不能再归还 */
                s_consecutive_tx_fail = 0;
            }
            else
            {
                bad_frames++;
                tx_fail_frames++;   /* 入队失败：无客户端 / 视频关闭 / 队列不可用 */
                if (s_consecutive_tx_fail != UINT32_MAX)
                    ++s_consecutive_tx_fail;  /* 饱和计数，避免长时间失败回绕后解除退避。 */
                esp_camera_fb_return(fb);   /* 未入队，所有权仍属 camera */
            }
        }

        /* 绝对节拍：本帧已耗时 > 目标间隔则不再补延时；否则补齐差值。
           默认目标 24fps；连续入队失败时额外补差，成功入队后自动恢复。
           延时保留 ms/tick 向下取整，存在量化尾差；下一轮重新计算补差。 */
        int64_t remain_us = cam_target_interval_us() -
                            (esp_timer_get_time() - frame_start_us);
        if (remain_us > 0)
            vTaskDelay(pdMS_TO_TICKS(remain_us / 1000));
    }
}
