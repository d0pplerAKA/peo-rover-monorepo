#pragma once
#include <stdint.h>
#include <stddef.h>
#include <vector>

/* 仅模拟网络侧用到的帧字段，并记录原始描述符的归还，禁止用 buf 冒充描述符。 */
struct camera_fb_t { uint8_t *buf; size_t len; };
inline std::vector<camera_fb_t*> host_returned_frames;
inline void esp_camera_fb_return(camera_fb_t *fb) { host_returned_frames.push_back(fb); }
