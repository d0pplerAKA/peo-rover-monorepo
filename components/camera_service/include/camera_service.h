#ifndef __CAMERA_SERVICE_H__
#define __CAMERA_SERVICE_H__

#include <stdint.h>

/**
 * @file    camera_service.h
 * @brief   PEO Rover - OV5640 摄像头服务：抓帧 -> JPEG -> esp_net UDP 视频流
 *
 * camera_task（优先级 3）：按 PEO_PINMAP.h 的 PEO_PM_CAM_* 引脚初始化
 * esp32-camera，循环抓 JPEG 帧交 esp_net_send_video() 分片发送（UDP 8889）。
 * 初始化失败：打日志并删除本任务（不影响其余子系统）。
 */

void camera_task(void *arg);

/* ---- 分辨率热切换（配合上位机 {"cmd":"CAM_RES","res":"QVGA"}，PROTOCOL §2）---- */
bool camera_queues_init(void);                                    /* 任务创建前调用；分辨率队列创建失败返回 false */
bool camera_resolution_from_name(const char *name, uint8_t *out); /* "QQVGA"/"QVGA"/"VGA"/"SVGA" -> framesize_t */
bool camera_request_resolution(uint8_t framesize);                /* 未就绪返回 false；就绪后入队（1深覆盖），下一循环应用 */

#endif /* __CAMERA_SERVICE_H__ */
