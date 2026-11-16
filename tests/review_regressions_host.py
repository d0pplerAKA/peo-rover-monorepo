"""Exercise production command parsing, LED setup and frame handoff on the host.

Hardware/RTOS adapters do not simulate camera DMA or actual RMT waveforms.
Requires GCC C++17 or MSVC C++20 (CXX), and the project's managed cJSON source.
"""
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CXX = os.environ.get('CXX') or shutil.which('g++')
if not CXX:
    raise SystemExit('A host C++ compiler is required (CXX, or g++ on PATH; MSVC cl is supported).')


def block(source, start):
    """Extract a production scope; fail loudly if its anchor changes."""
    begin = source.index(start)
    brace = source.index('{', begin)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[begin:end]


parser = (ROOT / 'components/cmd_parser/cmd_parser.cpp').read_text(encoding='utf-8')
parser = '\n'.join(line for line in parser.splitlines() if not line.startswith('#include'))
led = (ROOT / 'components/ws2812_led/ws2812_led.cpp').read_text(encoding='utf-8')
camera = (ROOT / 'components/camera_service/camera_service.cpp').read_text(encoding='utf-8')
link = (ROOT / 'components/esp_link/include/esp_link.h').read_text(encoding='utf-8')
ids = '\n'.join(re.findall(r'^#define ESP_CMD_\w+\s+(?:0x[0-9A-Fa-f]+|\d+)', link, re.M))

prefix = r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include "cJSON.h"
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define pdTRUE 1
#define portMAX_DELAY 0xffffffffu
static int esp_net_ack_queue=1, esp_net_rx_queue=2;
struct esp_net_rx_message_t {
    char *line; uint32_t generation; uint8_t slot; uint32_t received_ms, line_bytes;
};
static int ack_status=-1, color_calls=0, activity_calls=0, uart_calls=0;
static uint8_t color[4];
static bool color_available=true;
static uint32_t generation=7;
static int xQueueSend(int, void *item, unsigned) {
    char *json=*(char**)item;
    cJSON *ack=cJSON_Parse(json);
    if (!ack) abort();
    ack_status=cJSON_GetObjectItem(ack,"status")->valueint;
    cJSON_Delete(ack); free(json); return pdTRUE;
}
static int xQueueReceive(int,void*,unsigned) { return 0; }
static void esp_net_rx_release(uint32_t) {}
static void esp_link_request_stop() { ++generation; }
static uint32_t esp_link_command_generation() { return generation; }
static bool esp_link_resume_commands(uint32_t g) { return g==generation; }
static bool esp_link_send_command_for_generation(uint8_t,const uint8_t*,uint8_t,uint32_t) {
    ++uart_calls; return true;
}
static void esp_net_note_client_activity(uint8_t,uint32_t,bool) { ++activity_calls; }
static bool camera_resolution_from_name(const char*,uint8_t*) { return false; }
static bool camera_request_resolution(uint8_t) { return false; }
static void esp_net_video_set_enabled(bool) {}
static bool ws2812_set_color(uint8_t r,uint8_t g,uint8_t b,uint8_t brightness) {
    ++color_calls; color[0]=r; color[1]=g; color[2]=b; color[3]=brightness;
    return color_available;
}
'''

led_adapter = r'''
#define ESP_OK 0
#define PEO_PM_WS2812_GPIO 48
#define RMT_CLK_SRC_DEFAULT 0
struct rmt_symbol_word_t { unsigned duration0,level0,duration1,level1; };
struct rmt_bytes_encoder_config_t {
    rmt_symbol_word_t bit0,bit1;
    struct { unsigned msb_first:1; } flags;
};
struct rmt_tx_channel_config_t {
    int gpio_num,clk_src; unsigned resolution_hz,mem_block_symbols,trans_queue_depth;
};
static int s_tx_chan=0,s_encoder=0;
static rmt_bytes_encoder_config_t encoded_config;
static int rmt_new_tx_channel(const rmt_tx_channel_config_t*,int*) { return ESP_OK; }
static int rmt_new_bytes_encoder(const rmt_bytes_encoder_config_t *c,int*) {
    encoded_config=*c; return ESP_OK;
}
static int rmt_del_channel(int) { return ESP_OK; }
static int rmt_enable(int) { return ESP_OK; }
static unsigned received_byte(unsigned input) {
    unsigned output=0;
    for (int bit=0;bit<8;++bit) {
        unsigned shift=encoded_config.flags.msb_first ? 7-bit : bit;
        output=(output<<1)|((input>>shift)&1);
    }
    return output;
}
struct camera_fb_t { uint8_t *buf; size_t len; };
static unsigned returned=0,offered=0;
static bool accept_frame=true;
static camera_fb_t *owner=nullptr;
static uint32_t s_consecutive_tx_fail=0;
static void esp_camera_fb_return(camera_fb_t*) { ++returned; }
static void esp_net_video_set_frame_owner(camera_fb_t *fb) { owner=fb; }
static bool esp_net_send_video(const uint8_t *data,uint32_t len) {
    ++offered;
    if (!owner || owner->buf!=data || owner->len!=len) abort();
    return accept_frame;
}
// Timing adapters allow the old small-frame drop path to compile and fail the test.
static int64_t esp_timer_get_time() { return 0; }
static int64_t cam_target_interval_us() { return 1; }
#define pdMS_TO_TICKS(x) (x)
static void vTaskDelay(int64_t) {}
'''

frame_body = block(camera, 'if (fb != NULL)\n        {')
frame_test = r'''
static void offer_frame(camera_fb_t *fb) {
    uint32_t ok_frames=0,tx_fail_frames=0;
    int64_t frame_start_us=0;
    for (unsigned once=0;once<1;++once) {
''' + frame_body + '\n    }\n}\n'

main = r'''
static void require(bool ok,const char *label) {
    if (!ok) { fprintf(stderr,"FAIL %s\n",label); exit(1); }
}
static void command(const char *json,int expected,bool offered_color) {
    cJSON *root=cJSON_Parse(json);
    require(root!=nullptr,"JSON fixture parses");
    ack_status=-1; color_calls=activity_calls=uart_calls=0;
    handle_command(root,generation,0,123);
    cJSON_Delete(root);
    require(ack_status==expected,json);
    require(color_calls==(offered_color?1:0),"color service call count");
    require(activity_calls==0 && uart_calls==0,"LED command must stay local");
}
int main() {
    const char *invalid[]={"-256","-1","101","256","356","0.5","1e100","null","true","\"50\""};
    char json[256];
    for (const char *value:invalid) {
        snprintf(json,sizeof(json),"{\"cmd\":\"WS2812\",\"r\":128,\"g\":0,\"b\":255,\"brightness\":%s}",value);
        command(json,1,false);
    }
    for (const char *key:{"r","g","b"}) {
        for (const char *value:{"-1","256","1.5","null","\"1\""}) {
            cJSON *root=cJSON_Parse("{\"cmd\":\"WS2812\",\"r\":0,\"g\":0,\"b\":0}");
            cJSON_ReplaceItemInObject(root,key,cJSON_Parse(value));
            char *text=cJSON_PrintUnformatted(root); cJSON_Delete(root);
            command(text,1,false); free(text);
        }
    }
    command("{\"cmd\":\"WS2812\",\"r\":128,\"g\":0,\"b\":255}",0,true);
    require(color[0]==128 && color[1]==0 && color[2]==255 && color[3]==100,"default and RGB values");
    for (unsigned brightness:{0u,1u,50u,100u}) {
        snprintf(json,sizeof(json),"{\"cmd\":\"WS2812\",\"r\":128,\"g\":0,\"b\":255,\"brightness\":%u}",brightness);
        command(json,0,true); require(color[3]==brightness,"valid brightness preserved");
    }
    command("{\"cmd\":\"WS2812\",\"r\":1,\"g\":0}",1,false);
    command("{\"cmd\":\"WS2812\",\"r\":1,\"g\":0,\"b\":0,\"gen\":6}",2,false);
    color_available=false;
    command("{\"cmd\":\"WS2812\",\"r\":1,\"g\":0,\"b\":0}",1,true);
    puts("PASS actual parser: invalid/valid RGB and brightness, default, generation, service unavailable");
    require(rmt_init(),"RMT init");
    for (unsigned value:{0u,1u,2u,128u,255u})
        require(received_byte(value)==value,"LED wire MSB order");
    unsigned previous=0;
    for (unsigned brightness=0;brightness<=100;++brightness) {
        unsigned value=received_byte(scale(255,brightness));
        require(value==255*brightness/100 && value>=previous,"brightness monotonic after encoding");
        previous=value;
    }
    puts("PASS actual RMT setup/scale: asymmetric bytes and 0..100 percent brightness");
    uint8_t dummy=0;
    for (size_t length:{945u,1825u,4999u,5000u,65536u}) {
        camera_fb_t fb={&dummy,length};
        accept_frame=true; returned=offered=0; owner=nullptr;
        offer_frame(&fb);
        require(offered==1 && returned==0,"small/large accepted frame transferred exactly once");
        accept_frame=false; returned=offered=0; owner=nullptr;
        offer_frame(&fb);
        require(offered==1 && returned==1,"rejected enqueue returns frame exactly once");
    }
    puts("PASS actual frame handoff: small/large frames, successful transfer and rejected enqueue ownership");
    return 0;
}
'''

cjson = ROOT / 'managed_components/espressif__cjson/cJSON'
with tempfile.TemporaryDirectory(prefix='peo-review-', dir=os.environ.get('TMPDIR')) as temp:
    source = Path(temp) / 'review.cpp'
    exe = Path(temp) / ('review.exe' if os.name == 'nt' else 'review')
    source.write_text('#include <initializer_list>\n' + ids + '\n' + prefix + parser +
                      led_adapter + block(led, 'static inline uint8_t scale(') + '\n' +
                      block(led, 'static bool rmt_init(') + '\n' + frame_test + main,
                      encoding='utf-8')
    if Path(CXX).name.lower() in ('cl', 'cl.exe'):
        # C++20 provides designated initializers for the production RMT config.
        cjson_obj = Path(temp) / 'cjson.obj'
        subprocess.run([CXX, '/nologo', '/utf-8', '/std:c11', '/c', str(cjson / 'cJSON.c'),
                        '/Fo:' + str(cjson_obj)], check=True, cwd=temp)
        cmd = [CXX, '/nologo', '/utf-8', '/std:c++20', '/W3', '/wd4267', '/I', str(cjson),
               str(source), str(cjson_obj), '/Fe:' + str(exe)]
    else:
        cmd = [CXX, '-std=c++17', '-Wall', '-Wextra', '-Werror',
               '-Wno-unused-function', '-Wno-unused-parameter', '-Wno-unused-variable',
               *shlex.split(os.environ.get('CXXFLAGS', '')), '-I', str(cjson),
               str(source), str(cjson / 'cJSON.c'), '-o', str(exe)]
        if os.name == 'nt':
            cmd.insert(1, '-static')
    subprocess.run(cmd, check=True, cwd=temp)
    subprocess.run([str(exe)], check=True)
