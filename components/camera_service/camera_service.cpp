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
    if (!s_cam_cmd_queue) return false;  /* 待命时也接收请求，供下一轮重新初始化 */
    return xQueueOverwrite(s_cam_cmd_queue, &framesize) == pdTRUE;
}

/* 模组自带 XCLK：pin_xclk=GPIO_NUM_NC 时 ESP32 不输出时钟（esp32-camera 约定）；
   xclk_freq_hz 仍须等于模组晶振实际频率（OV5640 PLL 计算用）。
   TODO(用户确认)：常见 20MHz / 24MHz，按模组实际晶振填写 */
#define CAM_XCLK_HZ         24000000
#define CAM_TARGET_FPS_DEFAULT 15   /* 默认目标帧率；入队失败时通过额外补差退避。 */
#define CAM_TX_FAIL_BACKOFF_MS 200  /* 连续入队失败时额外补差，给网络恢复时间。 */
#define CAM_TX_FAIL_BACKOFF_FRAMES 3 /* 连续入队失败达到此帧数后启用退避。 */

static uint32_t s_consecutive_tx_fail = 0;  /* 仅 camera_task 更新，成功入队即清零。 */

/* init 后预热丢弃：传感器刚复位/重新 init，DMA 缓存同步与自动曝光未收敛，
   前 N 帧数据不可靠（上位机解码报 missing SOS / bogus marker 的根源）。
   48 帧 ≈ 2s @ 24fps——期间帧直接归还，不校验、不入队、不发送。 */
#define CAM_WARMUP_FRAMES 48
static uint32_t s_warmup_skip = 0;   /* 仅 camera_task 更新 */

static int64_t cam_target_interval_us(void)
{
    int64_t interval_us = (int64_t)(1000000u / CAM_TARGET_FPS_DEFAULT);
    if (s_consecutive_tx_fail >= CAM_TX_FAIL_BACKOFF_FRAMES)
        interval_us += (int64_t)CAM_TX_FAIL_BACKOFF_MS * 1000;
    return interval_us;
}

/* JPEG 帧缓冲容量：esp32-camera 默认公式 w*h/5 对小分辨率太紧
   （QQVGA 只算出 3.8KB，q30 JPEG 实际 3-6KB → DMA overflow / FB-SIZE overrun）。
   按分辨率给 max(w*h/4, 64KB)，实测余量充足且不浪费 PSRAM。 */
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
    /* 下限提到 64KB：q25 画质下 QVGA/QQVGA 偶发超 32KB（高频纹理场景），
       jpeg_buffer 写满 → 编码器截断输出 → 上位机 premature end 报错。
       PSRAM 充足（8MB 级），3 块 fb 共多占 ~96KB 完全可接受。 */
    return buf < 64 * 1024 ? 64 * 1024 : buf;
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
    bool active = false;  /* 仅本任务使用：false 表示驱动已 deinit，等待新请求自愈 */
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
        ESP_LOGI(TAG, "camera up: OV5640 QVGA JPEG");
        /* 启动诊断完成后立即归还：空闲态（无客户端/视频关）相机恒为 deinit */
        esp_net_video_flush();
        esp_camera_deinit();
        cam_release_control_gpios();
        active = false;
    }

    uint32_t ok_frames = 0, tx_fail_frames = 0, skipped_frames = 0;
    /* ok=入队成功（发送由 esp_net 异步完成）；tx_fail=入队失败（队满/无客户端）；
       skipped=视频关闭未尝试。三项之和 = 本窗口实际抓帧总数。 */
    int64_t last_log_us = esp_timer_get_time();

    EventGroupHandle_t eg = esp_net_conn_event();
    if (!eg)
    {
        ESP_LOGE(TAG, "connection event group missing; task exits");
        vTaskDelete(NULL);
        return;
    }

    for (;;)
    {
        /* 门控：无客户端或视频未启用 → 阻塞等待（camera 空闲态恒为 deinit）。
           CAM_RES 只可能来自在线客户端，阻塞期间无需理会分辨率队列。 */
        xEventGroupWaitBits(eg, PEO_NET_EVT_CLIENT | PEO_NET_EVT_VIDEO,
                            pdFALSE, pdTRUE, portMAX_DELAY);
        if (!active)
        {
            /* 从空闲恢复：硬复位（deinit 后传感器仍处流模式）后重新 init */
            cam_sensor_hard_reset();
            esp_err_t e = esp_psram_is_initialized()
                        ? esp_camera_init(&cfg) : ESP_ERR_INVALID_STATE;
            if (e != ESP_OK)
            {
                ESP_LOGE(TAG, "cam resume init failed (0x%x); retry in 1s", (int)e);
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            active = true;
            ok_frames = tx_fail_frames = skipped_frames = 0;
            s_consecutive_tx_fail = 0;
            s_warmup_skip = CAM_WARMUP_FRAMES;   /* init 后预热：丢弃前 N 帧 */
            last_log_us = esp_timer_get_time();
            ESP_LOGI(TAG, "camera resumed");
        }

        while ((xEventGroupGetBits(eg) & (PEO_NET_EVT_CLIENT | PEO_NET_EVT_VIDEO)) ==
               (PEO_NET_EVT_CLIENT | PEO_NET_EVT_VIDEO))
        {
        /* 活动时输出抓帧统计；待命仅输出等待日志，不能把停机时间计入正常帧率。 */
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_log_us >= 5000000LL)
        {
            if (active)
            {
                float real_fps = (float)(ok_frames + tx_fail_frames + skipped_frames) * 1000000.0f /
                                 (float)(now_us - last_log_us);
                ESP_LOGI(TAG, "cam stats: ok=%lu tx_fail=%lu skipped=%lu real_fps=%.1f",
                         (unsigned long)ok_frames, (unsigned long)tx_fail_frames,
                         (unsigned long)skipped_frames, real_fps);
            }
            else
                ESP_LOGI(TAG, "cam waiting: camera deinitialized");
            ok_frames = tx_fail_frames = skipped_frames = 0;
            last_log_us = now_us;
        }
        /* 分辨率热切换：deinit 后硬复位，再 init 重新分配帧缓冲。
           首次失败后按最新请求重试一次，再失败才回退；回退失败后 deinit 待命。 */
        uint8_t req;
        if (s_cam_cmd_queue && xQueueReceive(s_cam_cmd_queue, &req, 0) == pdTRUE)
        {
            uint8_t newer;
            while (xQueueReceive(s_cam_cmd_queue, &newer, 0) == pdTRUE)
                req = newer;   /* 只应用最新一次请求 */
            if (!active || req != cfg.frame_size)
            {
                framesize_t old_size = cfg.frame_size;
                ESP_LOGI(TAG, "cam res switch %u -> %u", (unsigned)old_size, (unsigned)req);
                cfg.frame_size = (framesize_t)req;
                cfg.jpeg_quality = cam_quality_for(cfg.frame_size);   /* 质量随分辨率分级，控制帧大小/带宽 */
                cfg.jpeg_buffer_size = cam_jpeg_buffer_for(cfg.frame_size);
                active = false;
                esp_net_video_flush();
                esp_camera_deinit();
                cam_release_control_gpios();
                cam_sensor_hard_reset();
                esp_err_t e = esp_psram_is_initialized() ? esp_camera_init(&cfg) : ESP_ERR_INVALID_STATE;
                if (e == ESP_OK)
                {
                    active = true;
                    s_warmup_skip = CAM_WARMUP_FRAMES;   /* 切换后同样预热 */
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
                    e = esp_psram_is_initialized() ? esp_camera_init(&cfg) : ESP_ERR_INVALID_STATE;
                    if (e == ESP_OK)
                    {
                        active = true;
                        s_warmup_skip = CAM_WARMUP_FRAMES;   /* 切换后同样预热 */
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
                            s_warmup_skip = CAM_WARMUP_FRAMES;   /* 回退后同样预热 */
                            ESP_LOGI(TAG, "cam res reverted: %u", (unsigned)old_size);
                        }
                    }
                }
                /* 生命周期转换后重新统计，避免待命/初始化耗时及旧退避影响恢复后的抓帧。 */
                ok_frames = tx_fail_frames = skipped_frames = 0;
                s_consecutive_tx_fail = 0;
                last_log_us = esp_timer_get_time();
            }
        }

        if (!active)
            break;   /* 分辨率切换全部失败已 deinit：退出抓帧循环，回门控等待自愈 */

        int64_t frame_start_us = esp_timer_get_time();   /* 节拍基准：固定 delay 会在 fb_get 阻塞/切分辨率时漂移 */
        camera_fb_t *fb = esp_camera_fb_get();
        if (s_warmup_skip > 0)
        {
            /* init 后预热：前 N 帧数据不可靠（缓存同步/曝光未收敛），直接丢弃 */
            if (fb != NULL)
                esp_camera_fb_return(fb);
            s_warmup_skip--;
            int64_t remain_us = cam_target_interval_us() -
                                (esp_timer_get_time() - frame_start_us);
            if (remain_us > 0)
                vTaskDelay(pdMS_TO_TICKS(remain_us / 1000));
            continue;
        }
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
            /* 零拷贝入队：驱动负责 SOI 检查、缓存同步和 EOI 截尾。
               QQVGA/低复杂度画面的合法 JPEG 可以很小，不能按压缩长度判损坏；
               完整 JPEG 解码校验由上位机完成，解码失败时丢弃该帧。 */
            esp_net_video_set_frame_owner(fb);
            if (esp_net_send_video(fb->buf, fb->len))
            {
                ok_frames++;   /* 所有权已交给 esp_net，此处不能再归还 */
                s_consecutive_tx_fail = 0;
            }
            else
            {
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

        /* 退场（断连/视频关/切换全败）：完整 deinit，归还 PSRAM 帧缓冲与 GPIO 保留 */
        esp_net_video_flush();
        esp_camera_deinit();
        cam_release_control_gpios();
        active = false;
        ESP_LOGI(TAG, "cam idle: deinitialized (frames released)");
    }
}
