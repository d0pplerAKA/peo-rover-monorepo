/**
 * @file    camera_service.cpp
 * @brief   PEO Rover - OV2640 抓帧任务（JPEG -> TCP 8890，应用层 15fps 节拍）
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
#include <freertos/semphr.h>
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
   xclk_freq_hz 仍须等于模组晶振实际频率（OV2640 PLL 计算用，标准 20MHz）。 */
#define CAM_XCLK_HZ         20000000

/* 只限制应用层取帧；传感器仍自由运行，多余帧由 GRAB_LATEST 弹旧回收。
   全局统一 24fps（2026-11-18 用户拍板锁 24，弃分级 30/30/24）：
   - QVGA 24fps ~0.96Mbps、SVGA 24fps ~2.7Mbps（贴墙档，EAGAIN 判据）；
   - 15fps 时代实测多窗口稳定，24fps 为画质/帧率/带宽平衡点。
   周期向上取整，避免整数截断使目标帧率偏高。 */
static int cam_fps_for(framesize_t fs)
{
    (void)fs;
    return 24;   /* 全档统一：锁 24fps */
}
static int64_t period_us_for_fps(int fps)
{
    return (1000000LL + fps - 1) / fps;
}
/* 运行时周期：cfg.frame_size 每次更新后调用（含启动默认档、切换、重试、回退），
   无条件同步，杜绝「未切换不更新」漏洞。初始默认 15fps 周期。 */
static int64_t s_frame_period_us = period_us_for_fps(15);
static void apply_frame_period(framesize_t fs)
{
    s_frame_period_us = period_us_for_fps(cam_fps_for(fs));
}

/* 回调运行于 ESP_TIMER_TASK，只唤醒相机任务，不在定时器任务中取帧或发送。
   使用专用二值信号量，避免与驱动或其他调用链使用的任务通知相互干扰。 */
static void cam_frame_timer_cb(void *arg)
{
    xSemaphoreGive((SemaphoreHandle_t)arg);
}

/* 本帧已入队或归还后，等待下一次取帧的绝对期限；调用者在首帧取出后建立期限。
   esp_timer_get_time() 为单调微秒时钟，单次定时器通过信号量直接唤醒任务，
   不把剩余时间截断为 FreeRTOS tick，也不忙等占用 CPU/影响网络任务。
   正常情况下按固定期限递增，避免将处理耗时、唤醒延迟逐帧累加；
   若取帧/处理已超时，或唤醒跨过下一周期，则从当前时刻重建节拍，不追赶欠帧。 */
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
            return err;   /* 不允许定时器故障后静默退化为不限速取帧 */
        xSemaphoreTake(ready, portMAX_DELAY);
        now_us = esp_timer_get_time();   /* 唤醒后复核期限，不因提前唤醒突破节拍 */
    } while (now_us < *next_frame_us);

    *next_frame_us += s_frame_period_us;
    if (*next_frame_us <= now_us)
        *next_frame_us = now_us + s_frame_period_us;
    return ESP_OK;
}

/* 15fps 带宽预算：QQVGA/QVGA 使用 q40，配合降帧率减少 JPEG 流量，
   为命令/遥测留出空口余量；实际带宽随场景变化，VGA/SVGA 保持现有质量参数。 */
static uint8_t cam_quality_for(framesize_t fs)
{
    switch (fs)
    {
    case FRAMESIZE_QQVGA:
    case FRAMESIZE_QVGA:  return 25;   /* 15fps + q25：画质优先；q35 实测仅 0.47Mbps，余量充裕 */
    case FRAMESIZE_VGA:   return 30;   /* 按 SVGA 实测比例下修预估 ~1.1Mbps */
    case FRAMESIZE_SVGA:  return 35;   /* q40 实测 1.55Mbps +0；q35 预估 ~1.9Mbps，切档后看 EAGAIN */
    default:              return 30;
    }
}

/* JPEG 帧缓冲按分辨率适配（2026-11-17 用户定案）：每档 = 实测帧上限 + 富余。
   不可留空（留空=w*h/5 低估：QQVGA 3.8KB 实测 DMA overflow）。
   热切换先改 cfg.frame_size 再 init，天然配套。未实测档（SVGA）取保守值。 */
static size_t cam_buffer_for(framesize_t fs)
{
    switch (fs)
    {
    case FRAMESIZE_QQVGA: return 16 * 1024;   /* 实测 2-5KB */
    case FRAMESIZE_QVGA:  return 32 * 1024;   /* 实测 10-20KB */
    case FRAMESIZE_VGA:   return 64 * 1024;   /* 实测 9-60KB（复杂场景擦边） */
    case FRAMESIZE_SVGA:  return 96 * 1024;   /* 未实测，保守；首跑高纹理需校准 */
    default:              return 64 * 1024;
    }
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

/* 传感器硬复位：esp_camera_deinit 只停 DMA，不会让 OV2640 停流（XCLK 模组自带，
   传感器持续出流）。流模式下 OV2640 对时序寄存器写操作 NAK，
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
    vTaskDelay(pdMS_TO_TICKS(20));   /* OV2640 复位后需数 ms 才能应答 SCCB */
    cam_release_control_gpios();     /* 清除此前 gpio_config 可能留下的输出保留；gpio_set_direction 不产生保留 */
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
    cfg.jpeg_buffer_size = cam_buffer_for(FRAMESIZE_QVGA);   /* 按分辨率适配（见 cam_buffer_for） */
    cfg.jpeg_quality   = cam_quality_for(FRAMESIZE_QVGA);   /* 按分辨率分级（见 cam_quality_for） */
    cfg.fb_count       = 3;    /* 3 块 + GRAB_LATEST（2026-11-17 用户拍板）：
                                 队列 2 槽 + 1 块 driver 专用 DMA/编码；
                                 队列满弹旧入新——应用永远拿到最新帧（低延迟），
                                 代价 = driver 恒满速抓帧、旧帧在队列弹旧中丢弃。 */
    cfg.fb_location    = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode      = CAMERA_GRAB_LATEST;   /* 永远最新：弹旧回收 fb，不停抓 */

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
        /* 启动诊断完成后立即归还：空闲态（无客户端/视频关）相机恒为 deinit */
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

    /* 任务生命周期内只创建一次；待命期间单次定时器已到期，无周期回调空转。
       此处相机已 deinit，资源申请失败可以直接清理并退出。 */
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
    int64_t next_frame_us = 0;   /* 0 表示新一轮抓帧，首帧直接取出并发送 */

    for (;;)
    {
        /* 门控：无客户端或视频未启用 → 阻塞等待（camera 空闲态恒为 deinit）。
           CAM_RES 留在覆盖队列中，恢复初始化前先应用最新请求。 */
        xEventGroupWaitBits(eg, PEO_NET_EVT_CLIENT | PEO_NET_EVT_VIDEO,
                            pdFALSE, pdTRUE, portMAX_DELAY);
        if (!active)
        {
            uint8_t req = (uint8_t)cfg.frame_size;
            uint8_t newer;
            while (s_cam_cmd_queue && xQueueReceive(s_cam_cmd_queue, &newer, 0) == pdTRUE)
                req = newer;   /* 初始化失败重试时也先应用最新请求 */
            if (req != cfg.frame_size)
            {
                cfg.frame_size = (framesize_t)req;
                cfg.jpeg_quality = cam_quality_for(cfg.frame_size);
                cfg.jpeg_buffer_size = cam_buffer_for(cfg.frame_size);   /* 适配新分辨率 */
            }
            apply_frame_period(cfg.frame_size);   /* 无条件同步：启动默认档无切换也必须应用分级帧率 */
            /* 从空闲恢复：硬复位（deinit 后传感器仍处流模式）后重新 init */
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
            next_frame_us = 0;   /* 重新开视频后首帧不继承上轮期限 */
            ESP_LOGI(TAG, "camera resumed: framesize=%u", (unsigned)cfg.frame_size);
        }

        while ((xEventGroupGetBits(eg) & (PEO_NET_EVT_CLIENT | PEO_NET_EVT_VIDEO)) ==
               (PEO_NET_EVT_CLIENT | PEO_NET_EVT_VIDEO))
        {
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
                cfg.jpeg_buffer_size = cam_buffer_for(cfg.frame_size);   /* 缓冲随分辨率适配 */
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
                    /* 复用 newer 记录是否合入期间的新请求；队列为空时 req 保持原目标。
                       只重试一次，把恢复服务与应用最新请求合并，避免反复 init。 */
                    newer = 0;
                    while (xQueueReceive(s_cam_cmd_queue, &req, 0) == pdTRUE)
                        newer = 1;
                    ESP_LOGI(TAG, "cam res retry target: %u (%s)", (unsigned)req,
                             newer ? "newer request" : "original request");
                    cfg.frame_size = (framesize_t)req;
                    cfg.jpeg_quality = cam_quality_for(cfg.frame_size);
                    apply_frame_period(cfg.frame_size);
                    esp_net_video_flush();
                    esp_camera_deinit();   /* 清理失败 init 的半配置态 */
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
                        cfg.jpeg_buffer_size = cam_buffer_for(cfg.frame_size);   /* 回退同步缓冲 */
                        apply_frame_period(cfg.frame_size);   /* 回退同步帧率 */
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
                            ESP_LOGI(TAG, "cam res reverted: %u", (unsigned)old_size);
                        }
                    }
                }
                next_frame_us = 0;   /* 热切换/回退后也立即发送首帧，不追赶停机期间的欠帧 */
            }
        }

        if (!active)
            break;   /* 分辨率切换全部失败已 deinit：退出抓帧循环，回门控等待自愈 */

        camera_fb_t *fb = esp_camera_fb_get();
        if (next_frame_us == 0)
            next_frame_us = esp_timer_get_time() + s_frame_period_us;
        if (fb != NULL && !esp_net_video_enabled())
        {
            esp_camera_fb_return(fb);   /* 视频关闭：抓帧即弃（统计已移至网络层） */
        }
        else if (fb != NULL)
        {
            /* 零拷贝入队：所有权转移给 esp_net，由发送任务归还。
               TCP 自带完整性校验，无需应用层 CRC。 */
            esp_net_video_set_frame_owner(fb);
            if (!esp_net_send_video(fb->buf, fb->len))
                esp_camera_fb_return(fb);   /* 未入队，所有权仍属 camera */
        }

        /* 所有取帧结果统一收尾：成功入队、失败归还、视频关闭 skipped、空帧。
           等待期间本任务不持有 fb；首帧发送前不等待，下一轮再向 driver 取帧，
           让 GRAB_LATEST 在间隔内正常弹旧。视频关闭后按原有门控退出并 deinit。 */
        pace_err = cam_wait_next_frame(frame_timer, frame_ready, &next_frame_us);
        if (pace_err != ESP_OK)
        {
            ESP_LOGE(TAG, "camera pacing wait failed (0x%x); task exits", (int)pace_err);
            break;   /* 先走统一退场，归还 esp_net 持有的帧，再释放节拍资源 */
        }
        }

        /* 退场（断连/视频关/切换全败）：完整 deinit，归还 PSRAM 帧缓冲与 GPIO 保留 */
        esp_net_video_flush();
        esp_camera_deinit();
        cam_release_control_gpios();
        active = false;
        ESP_LOGI(TAG, "cam idle: deinitialized (frames released)");
        if (pace_err != ESP_OK)
            break;
    }

    /* IDF v6.1：停止并等待可能正在另一核执行的回调，之后才能删除信号量。 */
    esp_timer_stop_blocking(frame_timer, portMAX_DELAY);
    esp_timer_delete(frame_timer);
    vSemaphoreDelete(frame_ready);
    vTaskDelete(NULL);
}
