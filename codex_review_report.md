# Codex 代码审查报告

## 一、结论摘要

当前工程已经具备较好的有界缓冲、快照队列和 STOP 代次隔离设计，但仍存在有效帧超时检查被异常字节流绕过、宏消费过期反馈等影响运动控制的问题，建议修复后再进行持续运动验收。本次记录 10 项发现：严重 2 项、中 7 项、低 1 项；其中多控制源与 TCP 接收前旧数据问题属于历史报告已披露的设计限制，JPEG 尾块问题属于需要实测确认的静态风险。结论以本次实际读取的磁盘代码为准，历史 BLE 架构、测试通过记录和旧固件产物不能替代当前 C3 架构的验证。

审查日期：2026-11-15。先阅读了 `BOOT_LOOP_FIX_REPORT.md`、`README.md`，随后读取 `main/main.cpp`、全部自写业务组件及接口、相关 CMake/sdkconfig、ESP32-S3 相机 HAL/底层/SCCB 路径和部分测试源码。以下行号均来自 PowerShell 按实际文件逐行编号的输出，代码片段保持源码原样；多个片段分别标明位置。

## 二、问题清单

### 1. [严重] C3 的 500ms 有效帧超时检查依赖 UART 无字节，持续坏流可绕过

**位置**：`components/c3_link/c3_link_rx.cpp:184`、`:188`、`:119`；`components/c3_link/include/c3_link.h:31`、`:37`；`components/c3_link/c3_map.cpp:90`、`:93`。

**证据**：`c3_link_rx.cpp:184–201`：

```cpp
        if (uart_read_bytes(PEO_PM_C3_UART, &b, 1, pdMS_TO_TICKS(100)) != 1) {
            /* 读超时/驱动错误：清候选帧 + C3 失联检查 */
            s_rx_count = 0;
            const uint32_t now = millis();
            if (!s_c3_lost && s_last_frame_ms != 0 &&
                (uint32_t)(now - s_last_frame_ms) > C3_LINK_TIMEOUT_MS) {
                s_c3_lost = true;
                ESP_LOGW(TAG, "C3 link lost (%lu ms no frame); latching STOP",
                         (unsigned long)(now - s_last_frame_ms));
                esp_link_request_stop();
                /* 快照标记离线（保留 seq/disc_gen 供重连后比较） */
                xbox_state_t st;
                if (xQueuePeek(xbox_state_queue, &st, 0) != pdTRUE)
                    memset(&st, 0, sizeof(st));
                st.connected = 0;
                xQueueOverwrite(xbox_state_queue, &st);
            }
            continue;
```

`c3_map.cpp:90–96`：

```cpp
            const bool live = st.connected &&
                (uint32_t)(now - st.snapshot_ms) <= XB_SNAPSHOT_MAX_AGE_MS;
            if (!live) {
                if (was_connected) latch_stop();
                was_connected = false;
                armed = driving = false;
                break;
```

**根因分析**：`s_last_frame_ms` 只在有效 GAMEPAD_STATE 的 `apply_gamepad()` 中刷新（`:119`），但检查它的代码只在一次 100ms 读操作失败后执行。只要错误波特率、持续噪声、坏 CRC 或未知类型数据让读操作持续取得字节，即使长期没有有效状态帧，也不会执行 500ms 停机检查。手柄此前在线时，映射任务通常还能在 1000ms 快照过期后加上自身调度延迟停机，但这已经超过所述 500ms 目标；若此前快照就是 `connected=0`、由网络/宏驱动，则映射任务的 `was_connected` 条件也不会提供该兜底。即使完全静默，100ms 读等待与严格 `>` 判断也不提供精确的 500ms 上界。

**修复建议**：让基于“最后有效状态帧”的截止时间检查独立于收字节成功与否，每轮或每个有界批次均执行；读等待取距截止时间的剩余量并明确 `>=` 边界。保留有效帧后的恢复流程，明确尚未收到首帧时是否启用监督。后续验证应覆盖持续坏 CRC、持续未知类型、短间隔噪声、静默，以及手柄离线但宏正在运行的情形。

### 2. [严重] 宏 WAIT 使用无限期保留的传感器快照，可依据过期反馈继续运动

**位置**：`components/esp_link/esp_link.cpp:128`、`:180`；`components/macro_engine/macro_engine.cpp:289`；`components/telemetry/telemetry.cpp:141`。

**证据**：`esp_link.cpp:128–130`：

```cpp
    esp_link_sensor_t sensor;
    memcpy(&sensor, &s_rx_buf[3], sizeof(sensor));
    xQueueOverwrite(esp_link_sensor_queue, &sensor);
```

`esp_link.cpp:180–184`：

```cpp
        if (uart_read_bytes(PEO_PM_ESP_UART, &b, 1, pdMS_TO_TICKS(100)) != 1) {
            s_rx_count = 0;  // Incomplete frame timeout / driver error.
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
```

`macro_engine.cpp:289–299`：

```cpp
                esp_link_sensor_t sn;
                if (xQueuePeek(esp_link_sensor_queue, &sn, 0) == pdTRUE)
                {
                    double v = field_value(&sn, st->u.wait.field);
                    if (field_compare(v, st->u.wait.value, st->u.wait.op))
                        step_done = true;    /* 条件满足 */
                }
                s_elapsed_ms = s_elapsed_ms > UINT32_MAX - MACRO_TICK_MS ?
                               UINT32_MAX : s_elapsed_ms + MACRO_TICK_MS;
                if (s_elapsed_ms >= st->u.wait.timeout_ms)
                    step_done = true;        /* 超时也下一步 */
```

**根因分析**：快照没有附带 S3 本地接收时间/有效代次；UART 静默不会使其失效，WAIT 只判断队列中存在数据。可出现如下路径：旧帧满足 `dist_front_mm < 200`，之后 F407→S3 接收线故障或上报停止，新启动宏仍立即满足该 WAIT 并发送下一步运动命令。相反，条件不满足时，等待超时也会继续下一步，通信故障与正常条件超时没有区分。遥测同样会反复广播最后一帧，缺少显式链路过期状态；远端 `sys_tick_ms` 本身不等于 S3 接收新鲜度校验。

**修复建议**：在内部快照信封中增加本地接收时间、接收序号和链路健康状态，保持 112B 线协议不变。启动依赖反馈的宏以及执行 WAIT 时检查数据年龄；反馈失联应进入明确的中止/锁 STOP 策略，不能隐式退化为“超时继续运动”。区分普通条件超时和通信故障，并向遥测发布过期状态。F407 自身命令超时保护仍需独立验证，本工程的静态代码无法证明其存在或实际电机停机效果。

### 3. [中] 接收前已积压的旧 TCP 命令会获得新代次，可能解除一次 STOP

**位置**：`components/esp_net/esp_net.cpp:408`、`:415`、`:433`；`components/cmd_parser/cmd_parser.cpp:192`、`:199`。历史说明：`BOOT_LOOP_FIX_REPORT.md:167`。

**证据**：`esp_net.cpp:408–415`：

```cpp
            uint32_t received_generation = esp_link_command_generation();
            int r = recv(c.fd, tmp, sizeof(tmp), 0);
            if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) continue;
            if (r <= 0) { drop_client(c, i, r == 0 ? "peer closed" : "recv error"); continue; }
            for (int k = 0; k < r; k++)
            {
                char ch = tmp[k];
                if (c.len == 0) c.generation = received_generation;
```

`cmd_parser.cpp:199–201`：

```cpp
    if (esp_link_resume_commands(generation) &&
        generation == esp_link_command_generation() &&
        esp_link_send_command_for_generation(k_cmd_table[idx].id, param, param_len, generation))
```

**根因分析**：这是已披露的协议边界，不是已入队消息代次校验失效。代次标记发生在本地 `recv()` 前，无法判断 TCP/lwIP 缓冲中的字节是在 STOP 前还是 STOP 后由操作者产生。若 C3 触发 STOP 后，网络任务才读到此前已经发送的完整 MOTOR 命令，这条旧命令可被标成当前代次，并在 STOP 已发送后经解析器恢复命令通道。仅检查行开始时的本地代次不能证明“停止后有新的明确输入”。

**修复建议**：增加客户端会话标识和命令序号，并在 STOP 后使用新的恢复令牌/握手拒绝旧消息；令牌必须参与后续运动命令校验。无法立即改两端时，可将 STOP 与关闭/重新建立控制会话绑定，重新连接后仍要求显式恢复，不要把任意后续 `recv()` 当作新输入证明。

### 4. [中] 多控制源共享同一命令代次，四轮命令不具备所有权和批次隔离

**位置**：`components/c3_link/c3_map.cpp:149`；`components/cmd_parser/cmd_parser.cpp:199`；`components/macro_engine/macro_engine.cpp:272`；`components/esp_link/esp_link.cpp:299`、`:309`。历史说明：`BOOT_LOOP_FIX_REPORT.md:166`。

**证据**：`c3_map.cpp:149–153`：

```cpp
            // One captured generation for the entire batch. If STOP/resume races
            // between motors, old commands cannot become valid again.
            if (!send_closed(0, left, generation) || !send_closed(1, left, generation) ||
                !send_closed(2, right, generation) || !send_closed(3, right, generation)) {
                latch_stop();
```

`esp_link.cpp:299–309`：

```cpp
    portENTER_CRITICAL(&s_safety_lock);
    const bool allowed = !s_inhibited && generation == s_generation;
    cmd.generation = s_generation;
    // Count before publishing so TX cannot complete before accounting starts.
    if (allowed) ++s_pending_commands;
    portEXIT_CRITICAL(&s_safety_lock);
    if (!allowed) return false;
    cmd.cmd_id = cmd_id;
    cmd.param_len = param_len;
    if (param_len != 0) memcpy(cmd.param, param, param_len);
    if (xQueueSend(esp_link_cmd_queue, &cmd, 0) == pdTRUE) return true;
```

**根因分析**：代次只区分 STOP 前后，不区分手柄、宏和不同 TCP 客户端。四轮分别入队，其他生产者可在中间插入同代命令，导致各轮目标来自不同来源；队列满时只能事后 STOP。手柄再次偏杆、宏 ACT 和直接网络命令都可能覆盖对方。现有“只使用一个主动驾驶来源”的文档约束没有被固件强制执行，普通命令也不会自动取消仍在同代运行的宏。

**修复建议**：由单一控制仲裁器管理所有者、租约和切换条件；将四轮目标作为一个逻辑批次提交，由 TX 按明确策略展开。保留 STOP 的全局抢占能力，控制源切换时清理旧所有者的工作。若单来源是产品硬约束，则在协议层拒绝其他来源的运动命令并返回明确状态。

### 5. [中] TX 最终代次检查与 UART 写入之间存在检查—执行窗口，STOP 契约需收紧或修正

**位置**：`components/esp_link/esp_link.cpp:262`、`:229`、`:36`；接口契约：`components/esp_link/include/esp_link.h:109`。

**证据**：`esp_link.cpp:262–268`：

```cpp
        portENTER_CRITICAL(&s_safety_lock);
        const bool allowed = !s_inhibited && cmd.generation == s_generation;
        portEXIT_CRITICAL(&s_safety_lock);
        if (allowed) {
            if (!transmit_command(cmd)) {
                ESP_LOGE(TAG, "UART TX failed; requesting STOP");
                esp_link_request_stop();
```

`esp_link.cpp:229–230`：

```cpp
    const int sent = uart_write_bytes(PEO_PM_ESP_UART, frame, size);
    const esp_err_t drained = uart_wait_tx_done(PEO_PM_ESP_UART, pdMS_TO_TICKS(100));
```

`esp_link.h:109–112`：

```cpp
/** Safety barrier: STOP is latched independently of the ordinary command queue.
 * Old generations are discarded by TX. At most an already-in-flight UART frame
 * can precede STOP. Resume requires the observed generation and a transmitted STOP.
 * Caller must require deliberate fresh input, never automatically resume old work.
```

**根因分析**：core0 的 TX 得到 `allowed=true` 后，core1 可锁存 STOP、推进代次；TX 随后仍用之前的布尔值开始 UART 写入。因此，可能先于 STOP 发出一条“STOP 请求发生时尚未进入 UART”的旧命令，边界比 README 所述“已物理在途的一帧”更宽。此处没有证据表明旧命令会在本轮 STOP 已发出之后继续成批发送：下一轮 TX 会优先处理 STOP；问题在请求至实际发射的线性化语义与时延边界。

**修复建议**：明确“已提交发送”的线性化点，让 STOP 和发送提交在同一同步协议下排序，并记录唯一已提交帧；文档应准确说明该帧可能尚未物理开始发送。若业务要求 STOP 请求返回后不得再开始旧帧，需设计更严格的 UART 提交/取消机制及对应时延验收。不要在自旋锁临界区中直接包住可能阻塞的 UART 写入/等待，也不能认为简单再检查一次就消除了窗口。

### 6. [中] 宏以循环次数模拟毫秒，短时长可提前结束，负载下持续时间与超时又会漂移

**位置**：`components/macro_engine/macro_engine.cpp:264`、`:281`、`:296`、`:346`；当前 tick：`sdkconfig:3054`。

**证据**：`macro_engine.cpp:278–284`：

```cpp
                    if (st->u.act.duration_ms == 0)
                        step_done = true;    /* 立即下一步（命令持续生效） */
                }
                s_elapsed_ms = s_elapsed_ms > UINT32_MAX - MACRO_TICK_MS ?
                               UINT32_MAX : s_elapsed_ms + MACRO_TICK_MS;
                if (s_elapsed_ms >= st->u.act.duration_ms && st->u.act.duration_ms > 0)
                    step_done = true;
```

`macro_engine.cpp:296–299` 与 `:346`：

```cpp
                s_elapsed_ms = s_elapsed_ms > UINT32_MAX - MACRO_TICK_MS ?
                               UINT32_MAX : s_elapsed_ms + MACRO_TICK_MS;
                if (s_elapsed_ms >= st->u.wait.timeout_ms)
                    step_done = true;        /* 超时也下一步 */
```

```cpp
        vTaskDelay(pdMS_TO_TICKS(MACRO_TICK_MS));
```

**根因分析**：进入步骤、刚刚入队 ACT 的同一轮就计入 10ms，尚未真正等待。单步 `duration_ms=10` 可在首次循环直接标记完成，尾步甚至只等待 UART 排空就 STOP，不能保证动作持续 10ms。另一方面，每轮工作耗时、同优先级竞争和高优先级相机驱动抢占均不会计入 `s_elapsed_ms`，导致较长动作/等待在实际时间上延后结束。当前配置是 1000Hz，不能把此问题归因于当前工程使用 10ms tick；它是计时方法本身的问题。另外，持续时间当前从入队而非 UART 完成开始，存在队列延迟语义差异。

**修复建议**：在步骤进入时记录单调时钟，使用实际经过时间/绝对截止时间判断；选择并说明 ACT 时长是从接受、发射还是设备执行开始。节拍可用绝对周期接口减少工作耗时累积，但节拍改造本身不能替代实际时间比较。验证 1/10/11ms、长时 WAIT、调度延迟和积压 TX 下的行为。

### 7. [中] 相机分辨率请求队列分配失败未处理，后续直接接收空队列

**位置**：`components/camera_service/camera_service.cpp:27`、`:163`；启动调用：`main/main.cpp:67`。

**证据**：`camera_service.cpp:27–31`：

```cpp
void camera_queues_init(void)
{
    if (!s_cam_cmd_queue)
        s_cam_cmd_queue = xQueueCreate(1, sizeof(uint8_t));
}
```

`camera_service.cpp:162–167`：

```cpp
        uint8_t req;
        if (xQueueReceive(s_cam_cmd_queue, &req, 0) == pdTRUE)
        {
            uint8_t newer;
            while (xQueueReceive(s_cam_cmd_queue, &newer, 0) == pdTRUE)
                req = newer;   /* 只应用最新一次请求 */
```

**根因分析**：相机队列与其他组件不同，没有检查创建结果，也没有向装配器返回错误。若该次分配失败、而 PSRAM 检查和相机驱动初始化随后成功，任务将以 NULL 句柄调用 `xQueueReceive()`，触发 FreeRTOS 断言或无效访问，破坏“相机失败不影响控制/网络”的降级目标。`camera_request_resolution()` 的空句柄检查不能保护任务自身的接收路径。

**修复建议**：队列初始化返回状态，决定在 app_main 禁用相机任务；相机任务入口也应检查必要资源并安全退出。对于可选相机服务，优先明确降级并发布不可用状态，而非依赖系统断言恢复。

### 8. [中] 相机驱动内部高优先级任务未检查创建结果，失败仍返回初始化成功

**位置**：`components/esp32-camera/driver/cam_hal.c:693`、`:700`、`:705`；`sdkconfig:4413`、`:4415`。

**证据**：`cam_hal.c:697–706`：

```c
#if CONFIG_CAMERA_CORE0
    xTaskCreatePinnedToCore(cam_task, "cam_task", CAM_TASK_STACK, NULL, configMAX_PRIORITIES - 2, &cam_obj->task_handle, 0);
#elif CONFIG_CAMERA_CORE1
    xTaskCreatePinnedToCore(cam_task, "cam_task", CAM_TASK_STACK, NULL, configMAX_PRIORITIES - 2, &cam_obj->task_handle, 1);
#else
    xTaskCreate(cam_task, "cam_task", CAM_TASK_STACK, NULL, configMAX_PRIORITIES - 2, &cam_obj->task_handle);
#endif

    ESP_LOGI(TAG, "cam config ok");
    return ESP_OK;
```

**根因分析**：app_main 的 9 个业务任务都检查了返回值，但驱动另建了一个优先级为 `configMAX_PRIORITIES - 2` 的 `cam_task`，当前固定到 core1、栈配置为 8192B。若分配失败，ISR 和队列已创建，但事件无人消费，初始化仍报告成功，上层反复取帧超时，且已分配的资源没有按创建失败路径及时释放。业务 camera 任务优先级 3 并不代表整个相机路径的优先级；评估实时性和栈/内部 RAM 预算时必须计入这个任务。

**修复建议**：检查所有分支的任务创建结果，失败则关闭中断/DMA并按资源获取的逆序统一清理，返回真实错误。为驱动任务维护可观察的运行状态；后续以故障注入验证任务分配失败不会被当成相机就绪。

### 9. [中] JPEG 长度按整块估算，EOI 搜索可能把未覆盖尾部的旧标记当成本帧结束（待实测确认）

**位置**：`components/esp32-camera/driver/cam_hal.c:399`、`:406`、`:821`、`:847`；`components/esp32-camera/target/esp32s3/ll_cam.c:560`、`:171`。

**证据**：`cam_hal.c:399–407`：

```c
                            cnt++;
                        }

                        cam_obj->frames[frame_pos].en = 0;

                        if (cam_obj->psram_mode) {
                            if (cam_obj->jpeg_mode) {
                                frame_buffer_event->len = cnt * cam_obj->dma_half_buffer_size;
                                if (frame_buffer_event->len > cam_obj->recv_size) {
```

`cam_hal.c:821–830`：

```c
                size_t probe_len = eoi_probe_window(cam_obj->dma_node_buffer_size,
                                                   dma_buffer->len);
                if (probe_len < JPEG_EOI_MARKER_LEN) {
                    goto skip_eoi_check;
                }
                uint8_t *probe_start = dma_buffer->buf + dma_buffer->len - probe_len;
                cam_drop_psram_cache(probe_start, probe_len);
                int off = cam_verify_jpeg_eoi(probe_start, probe_len, true);
                if (off >= 0) {
                    offset_e = dma_buffer->len - probe_len + off;
```

`cam_hal.c:847–853`：

```c
            if (offset_e >= 0) {
                dma_buffer->len = offset_e + JPEG_EOI_MARKER_LEN;
                if (cam_obj->psram_mode) {
                    /* DMA may bypass cache, ensure full frame is visible */
                    cam_drop_psram_cache(dma_buffer->buf, dma_buffer->len);
                }
                return dma_buffer;
```

**根因分析**：PSRAM JPEG 的块大小是 1024B；VSYNC 路径在已完成块数上额外计一块，以此估计长度。`len <= recv_size` 只证明逻辑上限没有超过帧容量，不能证明最后一块每个字节都由本帧 DMA 写入。帧槽会复用，启动前的 M2C cache invalidate 不会清除 PSRAM 中旧内容。如果本帧在最后块中途截断且没有有效 EOI，而旧帧 EOI 恰好位于该块尚未覆盖部分，当前扫描可能接受它并返回混合数据；上层再次检查 SOI/EOI也无法区分标记来自哪一帧。这是帧有效范围风险，不是已确认的堆越界；原先缓存行独占分配修复仍然有价值。

**不确定项**：本次未验证 S3 在 VSYNC/停止 DMA 时最后描述符 `length` 的精确更新语义、总线尾写入行为，以及上述截断序列是否能在当前 OV5640 时序下出现。因此将其标为静态风险，不能据此断言现有坏帧或历史崩溃由它引起。

**修复建议**：基于可验证的本帧 DMA 实际写入范围限制 EOI 搜索，而不是仅使用整块估计；若硬件不能提供可靠的尾部有效字节计数，设计可识别未覆盖尾部的方案。后续用同一槽“较长旧帧＋缺 EOI 的较短新帧”以及 VSYNC/EOF 边界注入验证，确认不会接受旧 EOI。

### 10. [低] C3 迁移后测试入口与 README 未同步，历史验证不能覆盖当前工程

**位置**：`tests/xbox_map_host.cpp:5`；`tests/xbox_run_host.py:23`、`:26`；`tests/net_run_host_tests.py:19`；`tests/net_host_test.cpp:9`；`tests/verify_firmware.py:22`；`README.md:13`、`:46`；当前架构：`main/main.cpp:8`，当前 XCLK：`components/camera_service/camera_service.cpp:51`。

**证据**：`tests/xbox_map_host.cpp:5`：

```cpp
#include "../components/xbox_service/xbox_map.cpp"
```

`tests/net_run_host_tests.py:19–20`：

```python
    for component in ["esp_net","esp_link","macro_engine","cmd_parser","telemetry","xbox_service"]:
        args.append("-I"+str(ROOT/"components"/component/"include"))
```

`tests/net_host_test.cpp:9`：

```cpp
#include "xbox_service.h"
```

`tests/verify_firmware.py:22–23`：

```python
    guard = symbols.get('_setBleLibraryInUse')
    check(guard is not None, 'Arduino BLE memory-retention constructor is linked')
```

`camera_service.cpp:51`：

```cpp
#define CAM_XCLK_HZ         24000000
```

README 的 `:13` 仍称配置为 **20 MHz**，`:46` 仍称 main 显式包含 BLE 内存保留头。磁盘 main 的 `:8–10` 已明确 BLE 迁到 C3，其实际 include 区域也没有该头。目录读取确认当前不存在 `components/xbox_service`、`components/xbox_controller`，网络测试 include 路径也没有加入当前 `c3_link/include`。

**根因分析**：验证脚本和接线/验收文档仍依赖迁移前的架构。旧手柄测试引用已移除的源文件；网络/遥测测试入口未适配新头文件及相机命令依赖；固件检查仍强制要求 S3 的 BLE 保留构造函数。历史测试数量和通过结论不能证明当前 C3 UART CRC、失联、回中与代次路径已经测试。20/24MHz 的文档矛盾也会影响相机标定；模组实际晶振频率本次不确定。

**修复建议**：将 S3 测试入口切换到实际 `c3_link_rx.cpp`/`c3_map.cpp`，补齐 UART、队列与相机命令适配，按当前无 BLE 的 S3 架构更新固件检查。将 BLE/HID 专属测试归属到 C3 工程；同步 README 的任务架构、XCLK 和验收口径，并明确历史报告对应版本。本次只阅读这些脚本，没有执行或构建它们。

## 三、优化方向

按优先级排序；本节为后续建议，本次未实施任何代码修改。

| 优先级 | 方向与依据 | 预期收益 |
|---|---|---|
| P0 | 修复问题 1：以有效状态帧的单调时间截止值驱动 C3 失联监督，逐批次检查并规定最坏调度裕量。 | 持续噪声/坏帧与静默具有一致的停机边界，使“500ms”成为可验证指标。 |
| P0 | 修复问题 2：给 F407 快照增加本地接收时间/序号；通信过期时中止依赖反馈的宏并发布故障。 | 防止旧测距/轮速驱动新的动作，区分条件等待与反馈失联。 |
| P0 | 解决问题 3、4 的控制授权边界：会话恢复令牌、来源所有权、四轮批次及明确的来源切换。 | 避免 STOP 后旧 TCP 数据恢复运动，以及手柄/宏/多客户端混合控制。 |
| P1 | 明确并实现问题 5 的 TX 提交语义；STOP 请求可用任务通知唤醒 TX，减少当前最多一次 10ms 队列等待带来的额外检测延迟（`esp_link.cpp:260`）。 | 获得可解释、可测量的停止时延，同时保持锁内操作短小。 |
| P1 | 修复问题 6，使用真实单调时间；宏和 50Hz 遥测考虑绝对周期调度。当前 `sdkconfig:3054` 和 `sdkconfig.defaults:16` 都是 1000Hz；`esp_net.cpp:229` 附近的 10ms tick 注释不能作为实际配置依据。 | 减少动作/等待计时偏差和周期漂移，防止调参依据过时。 |
| P1 | 修复问题 7、8，统一可选服务的初始化状态和逆序清理。审视网络 `ESP_ERROR_CHECK` 启动失败路径（`esp_net.cpp:147`、`:156–158`、`:320–322`），为可恢复失败设计先 STOP、再降级/重试的策略，NVS 修复不应默认等同于擦除用户数据。 | 分配或初始化失败不会伪装成功，也降低相机/网络故障扩大为整机复位的风险。 |
| P1 | 先验证问题 9 的尾部有效范围；再检查 `cam_drop_psram_cache()`/`cam_flush_psram_cache()` 未传播的同步返回值（`cam_hal.c:109`、`:123`）。保留既有缓存行独占分配，记录同步失败的帧丢弃原因。 | 防止把容量范围误当本帧有效范围，改善异常帧诊断。 |
| P1 | 制定 CPU/内存预算：F407 RX 优先级 6 可在连续积压时持续逐字节处理；驱动 `cam_task` 的优先级远高于应用 camera=3。网络单任务中的逐行 `xQueueSend(...,50ms)`（`esp_net.cpp:438`）会推迟其他客户端 RX/TX 服务，宜采用每轮字节/行数/耗时预算和明确背压策略。 | 控制持续输入和突发负载下的调度延迟，保护 STOP、手柄映射和网络服务时效。 |
| P1 | 更新问题 10 的验证入口，再安排当前源码的边界/故障验证：CRC 重同步、STOP 提交窗口、旧 TCP 积压、队列满、分配失败、同槽残帧、计时漂移。 | 使验证真正对应磁盘生产代码，避免旧测试成绩造成错误信心。 |
| P2 | 遥测与控制 ACK 分配不同发送预算；遥测允许保留最新完整帧。当前 TX 上限为每客户端 8192B（`esp_net.cpp:248`），追加超限立即断开并 STOP（`:456–457`、`:278`），与 5000ms 无进展超时是两个独立条件；应按实际 JSON 大小和输出速率估算可容忍抖动，不能认为一定等满 5s。 | 减少短时遥测积压造成的全局停机，保持 ACK 可观察性和内存有界。 |
| P2 | 增加服务健康状态与异步命令结果。`camera_request_resolution()` 只验证队列存在（`camera_service.cpp:42–45`），相机任务退出后队列仍存在，而解析器会对成功入队立即 ACK 0（`cmd_parser.cpp:158–162`）。应区分“已接受”“已应用”“服务不可用”。 | 上位机能够识别实际生效/回退/离线状态，减少排障误判。 |
| P2 | 评估固定缓冲/对象池、JSON 节点预算和栈高水位。32 条 RX 行的字符串载荷上界约 524320B，4 个行缓冲另约 65540B，4 个待发送流另最多 32768B；这些尚未计 cJSON 节点、宏、socket 和分配器开销（依据 `esp_net.cpp:242`、`:423`、`:248` 与 `esp_net.h:35–36`）。 | 量化 PSRAM/内部 RAM 压力与碎片风险，保留控制路径分配余量。 |

任务配置按实际 `main/main.cpp:70–78` 记录如下。栈参数以 ESP-IDF 任务创建接口的字节口径记录；未将通用 FreeRTOS 的 word 口径套用到本工程。

| 业务任务 | Core | 优先级 | 栈参数（B） |
|---|---:|---:|---:|
| esp_rx | 0 | 6 | 4096 |
| esp_tx | 0 | 5 | 3072 |
| c3_rx | 0 | 5 | 4096 |
| esp_net | 0 | 5 | 10240 |
| macro | 1 | 4 | 6144 |
| c3_map | 1 | 4 | 4096 |
| telemetry | 1 | 4 | 6144 |
| cmd_parser | 1 | 4 | 6144 |
| camera | 1 | 3 | 8192 |

上述业务任务栈合计 52224B，另有相机驱动任务的 8192B 及系统/Arduino/WiFi/lwIP 任务、TCB 等开销。本次没有发现足以静态证明这几个业务栈必然溢出的局部大数组：例如 16KiB 网络行在堆上，宏程序在堆上，视频分片 `pkt` 是静态缓冲；但 UART、日志、cJSON 递归和驱动的最深调用链仍需用高水位及异常路径实测确认，尤其是 3072B 的 TX 栈。不能据此宣称栈余量充足。

## 四、好的实践

- **装配前创建业务队列**：`main/main.cpp:63–67` 在创建任务之前初始化，避免跨核消费者抢先访问尚未创建的队列；`start_task()` 在 `:43` 检查任务创建结果。相机请求队列的例外已列入问题 7。
- **STOP 独立锁存并推进代次**：`esp_link.cpp:36–43` 使用跨核自旋锁保护安全状态，不依赖普通命令队列空位；`:249–252` 只在成功发送且代次仍匹配时清除待发 STOP，失败保留请求。
- **生产者携带代次，普通命令发送前再验证**：`esp_link.cpp:288–314` 和 `:262–264` 能淘汰大部分跨 STOP/恢复的旧队列工作；四轮使用同一捕获代次。保留这一层，另行解决问题 3–5 的边界。
- **队列接受与物理发射完成分开计数**：`esp_link.cpp:302–303` 在发布前计数、`:310–312` 在入队失败时回滚、`:270–273` 在 UART 完成后扣减；`macro_engine.cpp:332–343` 正常结束等待本代排空，避免末条 ACT 被收尾 STOP 直接丢弃。它证明 UART 发送，不代表 F407 已执行。
- **有界解析与显式端序**：F407 RX 固定 112B LEN、C3 RX 限制最多 32B 并验证状态帧恰为 26B；CRC 均覆盖 SOF、LEN、payload，采用 0x1021/0xFFFF、CRC 低字节先传。两条 RX 路径 CRC 失败后滑动保留候选后缀，而不是无界增长（`esp_link.cpp:188–207`，`c3_link_rx.cpp:204–230`）。CRC 表在 app_main 的队列初始化阶段完成，不是由运行中的 RX/TX 首次并发构建。
- **手柄快照与故障恢复约束**：`c3_link_rx.cpp:95–125` 用 overwrite 发布完整状态；`c3_map.cpp:83–129` 检查断连代次、STOP 代次及停止后的新鲜回中通知，且中立空闲不持续覆盖网络控制（`:139`）。摇杆先用 int32 去中值，避免无符号下溢。
- **非破坏快照消费**：宏与遥测都使用 `xQueuePeek()`，不会互相取走最新传感器数据；单深队列使读到的结构体一致。应在此基础上增加接收新鲜度，见问题 2。
- **网络与 JSON 边界防护**：16384B 行上限、原始 NUL 拒绝、分配/队列满断开；严格完整 JSON 解析、整数范围/有限浮点校验、64 步宏上限（`esp_net.cpp:416–442`，`cmd_parser.cpp:62–72`、`:226–234`、`:353`）。大量缓冲位于堆/静态区，避免把最大 JSON 行放到任务栈。
- **部分发送与内存所有权清晰**：`esp_net.cpp:281–314` 保留未发送尾部，只在发送有进展后更新超时；客户端断开释放 RX/TX 缓冲，广播后释放出队字符串；宏所有权移交失败和终止路径有对应释放。遥测的 `add_item()` 在 cJSON 添加失败时释放孤立节点（`telemetry.cpp:47–54`）。
- **视频发送的共享数据与边界处理**：`esp_net.cpp:181–184` 在锁内复制 UDP 描述符/目的端快照，网络调用在锁外；单 camera 生产者使用静态包缓冲。8B 大端分片头、1400B JPEG 分片上限、零长度/总分片数溢出检查和同 IP 去重可保留（`:185–222`）。
- **相机 PSRAM 缓存行独占分配**：`cam_hal.c:529–590` 取 DMA/cache 对齐最大值并向上取整长度，保留真实分配指针；`ll_cam.c:171–176` 在 DMA 启动前检查缓存准备结果。当前 `sdkconfig:2455–2457` 是 64B 数据缓存行，历史 16B 偏移问题在这些路径中已得到针对性处理；不能据此推断所有 DMA 时序均已验证。
- **可选相机服务与基本帧释放**：`camera_service.cpp:111–114` 在 PSRAM 不可用时退出；`:174–197` 尝试分辨率回退；`:239` 对取得的帧归还驱动。引脚定义集中在 `PEO_PINMAP.h`，应用未占用 GPIO35/36/37；实际接线和 XCLK 仍需核实。

## 五、审查限制说明

1. **仅静态审查**。本次只通过 PowerShell 读取/检索代码并新增本报告；没有构建、烧录、启动固件、连接串口、运行测试、Python、编译器或其他工程程序，没有 commit，没有修改或删除既有文件。
2. **当前源码优先**。9 个业务任务的栈与优先级取自本次磁盘 main。历史报告描述本地 BLE，当前 main 已说明 BLE 迁到 C3；README 中 BLE、XCLK 和测试口径存在遗留差异。未将旧 ELF/BIN、旧测试成功记录当作当前工作树的验证结果，也未断言当前工程可成功构建。
3. **覆盖范围有限**。重点逐段检查了自写组件和当前 S3 JPEG/PSRAM DMA 的相机 HAL/底层路径，另读取相关 SCCB、配置和测试入口；没有穷尽全部 managed_components、ESP-IDF、Arduino、所有传感器驱动、转换器及第三方库。未检查外部 F407/C3 工程和上位机实现；源码引用的外部 PROTOCOL.md 对端定义未在本次工程中得到独立核验，因此 CRC/端序结论指本地实现一致性，不代表两端兼容性已验证。
4. **确定性与条件性分开**。问题 1–8 及 10 的代码路径/结构可以静态定位，但严重后果仍取决于相应输入、故障或调度发生；问题 3、4 是既有设计限制，保留在问题清单中是因为它们直接影响停止后恢复与多来源控制。问题 9 的 DMA 尾块有效范围、描述符更新及总线停止语义标为“不确定”，需要硬件资料和故障注入确认，不能归因于它解释历史崩溃。
5. **需要后续实测的主要指标**：持续异常 UART 流下的有效帧超时；STOP 请求到 UART 物理完成的最坏时延；两个核心在串口/网络/视频并发负载下的调度和空闲时间；任务栈高水位、内部 RAM 最小余量、PSRAM 碎片；初始化和任务/队列分配失败降级；相机多次分辨率切换、缓存同步与坏帧复用；真实 WiFi 慢客户端、重连与 TCP 积压；C3 实际通知序号/新鲜度行为；F407 独立停机超时、电机实际编号/方向及命令 ACK 含义。本次未执行这些验证。
6. **本地代次不是硬件急停**。串口 `uart_wait_tx_done()` 成功只能说明本机 UART 发射完成，不能证明 F407 收到或电机已经停止。掉电、崩溃、整条 UART 断线时的最终停机能力不能由这份 S3 静态审查确认。