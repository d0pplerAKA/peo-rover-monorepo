"""Compile the actual camera allocation/cache functions with host adapters.
This checks ownership of cache-line ranges, NOT physical ESP32 DMA operation.
"""
import os
import shlex
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
CC = os.environ.get('CC') or shutil.which('gcc')
if not CC:
    raise SystemExit('A host C11 compiler is required (CC, or gcc on PATH; MSVC cl is supported).')

def contract_checks():
    """Deterministic pre/post-fix arithmetic from the crash log."""
    line = 64
    old_frame = 0x3C1170B0          # logged Frame[1] with the old 16-byte offset
    old_alloc = old_frame - 16      # start of the heap-owned block
    soi_start = old_frame & ~(line - 1)
    assert soi_start < old_alloc, "fixture must reproduce a pre-fix overlap"
    assert soi_start <= 0x3C117084 < old_alloc, "crash-adjacent heap bytes were shared"
    new_frame = 0x3C1170C0          # nearest 64-byte boundary
    assert (new_frame & (line - 1)) == 0
    print("PASS pre-fix 64-byte cache invalidation crossed into the previous heap block")
    print("PASS crash-adjacent address 0x3C117084 lies inside that collateral range")
    print("PASS cache-aligned frame starts on an owned cache line")

def source_contract_checks():
    """Guard the call-order contract that host allocation tests cannot execute."""
    ll = (ROOT/'components/esp32-camera/target/esp32s3/ll_cam.c').read_text(encoding='utf-8')
    start = ll[ll.index('bool ll_cam_start'):]
    start = start[:start.index('\n}\n')]
    prepare = start.index('ESP_CACHE_MSYNC_FLAG_DIR_M2C')
    link_start = start.index('.link.start')
    assert prepare < link_start, "M2C invalidation must precede DMA link start"
    assert 'sync_err != ESP_OK && sync_err != ESP_ERR_NOT_SUPPORTED' in start, \
        "only unsupported/non-cacheable memory may be tolerated"
    assert 'fb_dma_sync_size' in start, "sync size must come from the owned allocation"

    hal = (ROOT/'components/esp32-camera/driver/cam_hal.c').read_text(encoding='utf-8')
    gray = hal[hal.index('currently used only for YUV to GRAYSCALE'):]
    gray = gray[:gray.index('else if (cam_obj->psram_mode)')]
    assert gray.index('cam_drop_psram_cache') < gray.index('ll_cam_memcpy'), \
        "converted pixels must be invalidated before CPU read"
    assert gray.index('cam_flush_psram_cache') > gray.index('ll_cam_memcpy'), \
        "CPU-authored conversion must be written back"
    print("PASS DMA cache preparation is checked and precedes link start")
    print("PASS YUV conversion invalidates DMA data before read and flushes CPU output")

def function(source, signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 1
    i = brace + 1
    while depth:
        depth += (source[i] == '{') - (source[i] == '}')
        i += 1
    return source[start:i] + '\n'

hal = (ROOT/'components/esp32-camera/driver/cam_hal.c').read_text(encoding='utf-8')
ll = (ROOT/'components/esp32-camera/target/esp32s3/ll_cam.c').read_text(encoding='utf-8')
prefix = r'''
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdalign.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define ESP_IDF_VERSION_VAL(a,b,c) (((a)<<16)|((b)<<8)|(c))
#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(6,1,0)
#define ESP_IDF_VERSION_MAJOR 6
#define CACHE_LL_LEVEL_EXT_MEM 0
#define CACHE_TYPE_DATA 0
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG -2
#define ESP_CACHE_MSYNC_FLAG_DIR_C2M 4
#define ESP_CACHE_MSYNC_FLAG_DIR_M2C 8
#define ESP_CACHE_MSYNC_FLAG_INVALIDATE 1
#define MALLOC_CAP_8BIT 1
#define MALLOC_CAP_INTERNAL 2
#define MALLOC_CAP_SPIRAM 4
#define MALLOC_CAP_DMA 8
#define MALLOC_CAP_DEFAULT 16
#define CAMERA_FB_IN_DRAM 0
#define CAMERA_FB_IN_PSRAM 1
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
#define CAM_CHECK(c,m,r) do { if(!(c)) return (r); } while(0)
typedef int esp_err_t;
typedef struct { uint32_t size,length,sosf,eof,owner,empty; uint8_t *buf; } lldesc_t;
typedef struct { struct { uint8_t *buf; } fb; lldesc_t *dma; size_t fb_offset; int en; } cam_frame_t;
typedef struct {
    bool psram_mode, jpeg_mode;
    uint32_t dma_buffer_size, dma_half_buffer_size, dma_half_buffer_cnt;
    uint32_t dma_node_buffer_size, dma_node_cnt, frame_copy_cnt;
    uint32_t fb_size, recv_size, frame_cnt, dma_bytes_per_item;
    size_t fb_dma_sync_size;
    cam_frame_t *frames; uint8_t *dma_buffer; lldesc_t *dma;
} cam_obj_t;
typedef struct { int fb_location; } camera_config_t;
static cam_obj_t object, *cam_obj=&object;
static size_t mock_line=64;
static uint8_t mock_dma_align=16;
static int failures=0, sync_violations=0, prepares=0;
static esp_err_t sync_result=ESP_OK;
static int last_sync_flags=0;
/* Set to emulate the pre-fix allocator: 16-byte alignment + manual offset. */
static int legacy_layout=0;
static int legacy_offset=0;
struct allocation { void *backing; uintptr_t begin; size_t size; };
static struct allocation slots[8];
static unsigned slot_count=0;
static void *other[32];
static unsigned other_count=0;
static size_t cache_hal_get_cache_line_size(int a, int b) { (void)a;(void)b;return mock_line; }
static uint8_t ll_cam_get_dma_align(cam_obj_t *cam) { (void)cam;return mock_dma_align; }
static bool ll_cam_calc_rgb_dma(cam_obj_t *cam) { (void)cam;return false; }
static size_t heap_caps_get_largest_free_block(int caps) { (void)caps;return 1000000; }
static void *heap_caps_malloc(size_t size, int caps) {
    (void)caps; void *p=calloc(1,size?size:1);other[other_count++]=p;return p;
}
static void *heap_caps_aligned_calloc(size_t alignment,size_t n,size_t size,int caps) {
    (void)alignment;return heap_caps_malloc(n*size,caps);
}
static void *heap_caps_aligned_alloc(size_t alignment,size_t size,int caps) {
    (void)caps;
    void *raw=malloc(size+512);
    uintptr_t base=((uintptr_t)raw+127)&~(uintptr_t)127;
    uintptr_t addr,owned_size;
    if(legacy_layout || legacy_offset) {
        /* Drive the block into the cache-misaligned case deterministically. */
        /* Force a 16-byte-aligned allocation that is NOT cache-line aligned.
         * Rounding base+7 down to 16 used to return base and missed this case. */
        addr=base+16+(uintptr_t)legacy_offset;
        owned_size=size;
    } else {
        addr=(base+alignment-1)&~(uintptr_t)(alignment-1);
        owned_size=size-(addr-base);
    }
    slots[slot_count++]=(struct allocation){raw,addr,owned_size};
    return (void*)addr;
}
static esp_err_t esp_cache_msync(void *addr,size_t size,int flags) {
    last_sync_flags=flags;
    uintptr_t begin=(uintptr_t)addr,end=begin+size;
    bool owned=false;
    for(unsigned i=0;i<slot_count;i++)
        if(begin>=slots[i].begin && end<=slots[i].begin+slots[i].size) owned=true;
    if(!owned || begin%mock_line || size%mock_line) {
        sync_violations++;
        fprintf(stderr,"FAIL cache operation touches bytes outside its framebuffer (line=%u)\n",(unsigned)mock_line);
        return ESP_ERR_INVALID_ARG;
    }
    if(flags & ESP_CACHE_MSYNC_FLAG_DIR_C2M) prepares++;
    return sync_result;
}
static void cleanup(void) {
    for(unsigned i=0;i<slot_count;i++) free(slots[i].backing);
    for(unsigned i=0;i<other_count;i++) free(other[i]);
    slot_count=other_count=0;
}
'''
body = '\n'.join([
    function(hal, 'static inline size_t dcache_line_size(void)'),
    function(hal, 'static inline esp_err_t cam_drop_psram_cache('),
    function(hal, 'static inline esp_err_t cam_flush_psram_cache('),
    function(ll, 'bool ll_cam_dma_sizes('),
    function(hal, 'static lldesc_t * allocate_dma_descriptors('),
    function(hal, 'static esp_err_t cam_dma_config('),
])
suffix = r'''
int main(int argc,char **argv) {
    const size_t lines[]={32,64};
    const uint8_t dma_alignments[]={16,32,64};
    const uint32_t sizes[]={15360,15361,10000};
    legacy_layout = (argc > 1 && !strcmp(argv[1], "legacy"));
    legacy_offset = (argc > 1 && !strcmp(argv[1], "legacy-offset")) ? 16 : 0;
    sync_violations = 0; prepares = 0;
    unsigned cases=0;
    for(unsigned a=0;a<2;a++) for(unsigned b=0;b<3;b++) for(unsigned c=0;c<3;c++) {
        mock_line=lines[a];mock_dma_align=dma_alignments[b];
        sync_violations=prepares=0;
        memset(&object,0,sizeof(object));
        object.psram_mode=true;object.jpeg_mode=true;object.frame_cnt=2;
        object.fb_size=object.recv_size=sizes[c];
        camera_config_t cfg={CAMERA_FB_IN_PSRAM};
        if(cam_dma_config(&cfg)!=ESP_OK) { fprintf(stderr,"FAIL allocation setup\n");failures++;cleanup();continue; }
        for(unsigned f=0;f<2;f++) {
            uint8_t *buf=object.frames[f].fb.buf;
            size_t usable=object.recv_size+object.dma_half_buffer_size;
            if (cam_drop_psram_cache(buf,32) != ESP_OK ||
                cam_drop_psram_cache(buf,usable) != ESP_OK ||
                cam_drop_psram_cache(buf+usable-1025,1025) != ESP_OK)
                failures++; // SOI, max frame/tail range, EOI straddling boundary.
            if (last_sync_flags != (ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_INVALIDATE))
                failures++;
            if (cam_flush_psram_cache(buf,usable) != ESP_OK ||
                last_sync_flags != ESP_CACHE_MSYNC_FLAG_DIR_C2M) failures++;
            // Synchronization errors must reach callers; never mask a failed cache operation.
            sync_result=ESP_FAIL;
            if (cam_drop_psram_cache(buf,32) != ESP_FAIL ||
                cam_flush_psram_cache(buf,32) != ESP_FAIL) failures++;
            sync_result=ESP_OK;
            for(unsigned n=0;n<object.dma_node_cnt;n++) {
                lldesc_t *d=&object.frames[f].dma[n];
                if(d->buf<buf || d->buf+d->size>buf+usable) {fprintf(stderr,"FAIL descriptor out of range\n");failures++;}
            }
        }
        if(sync_violations) failures++;
        size_t required = mock_line > mock_dma_align ? mock_line : mock_dma_align;
        if(object.fb_dma_sync_size % required != 0 ||
           (uintptr_t)object.frames[0].fb.buf % required ||
           (uintptr_t)object.frames[1].fb.buf % required ||
           object.frames[0].fb.buf == object.frames[1].fb.buf) {
            fprintf(stderr,"FAIL framebuffers are not independently cache/DMA aligned\n");failures++;
        }
        cases++;cleanup();
    }
    if(failures) {fprintf(stderr,"FAIL %d checks across %u allocation cases\n",failures,cases);return 1;}
    printf("PASS %u real allocation/cache cases (32/64-byte cache, 16/32/64-byte DMA, non-aligned lengths)\n",cases);
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='peo-camera-', dir=os.environ.get('TMPDIR')) as tmp:
    source = Path(tmp)/'camera_dma_test.c'
    exe = Path(tmp)/('camera_dma_test.exe' if os.name=='nt' else 'camera_dma_test')
    source.write_text(prefix+body+suffix,encoding='utf-8')
    if Path(CC).name.lower() in ('cl', 'cl.exe'):
        cmd=[CC,'/nologo','/utf-8','/std:c11','/W3','/wd4311','/wd4267','/wd4018',str(source),'/Fe:'+str(exe)]
    else:
        cmd=[CC,'-std=c11','-Wall','-Wextra','-Werror','-Wno-pointer-to-int-cast','-Wno-sign-compare','-Wno-unused-function',*shlex.split(os.environ.get('CFLAGS', '')),str(source),'-o',str(exe)]
        if os.name=='nt':cmd.insert(1,'-static')
    subprocess.run(cmd,check=True,cwd=tmp)
    contract_checks()
    source_contract_checks()
    subprocess.run([str(exe)],check=True)
    for layout in ('legacy', 'legacy-offset'):
        result = subprocess.run([str(exe), layout], capture_output=True, text=True)
        if result.returncode != 1 or 'FAIL allocation setup' not in result.stderr:
            raise AssertionError(f'{layout}: expected controlled rejection, got {result.returncode}\n{result.stderr}')
        print(f'PASS rejected {layout} misaligned allocation')
