#ifndef __CAMERA_SERVICE_H__
#define __CAMERA_SERVICE_H__

#include <stdint.h>

/**
 * @file    camera_service.h
 * @brief   PEO Rover - OV2640 摄像头服务：抓帧 -> JPEG -> esp_net TCP 视频流
 *
 * camera_task（优先级 3）：按 PEO_PINMAP.h 的 PEO_PM_CAM_* 引脚初始化
 * esp32-camera，循环抓 JPEG 帧交 esp_net_send_video() 定界发送（TCP 8890）。
 * 初始化/回退失败：清理驱动后待命，新分辨率请求可触发自愈；任务不退出。
 */

void camera_task(void *arg);

/* ---- 分辨率热切换（配合上位机 {"cmd":"CAM_RES","res":"QVGA"}，PROTOCOL §2）---- */
bool camera_queues_init(void);                                    /* 任务创建前调用；分辨率队列创建失败返回 false */
bool camera_resolution_from_name(const char *name, uint8_t *out); /* "QQVGA"/"QVGA"/"VGA"/"SVGA" -> framesize_t */
bool camera_request_resolution(uint8_t framesize);                /* 队列不存在返回 false；活动/待命均可入队（1深覆盖） */

#endif /* __CAMERA_SERVICE_H__ */
