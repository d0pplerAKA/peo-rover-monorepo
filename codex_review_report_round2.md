# PEO Rover ESP32-S3 静态代码审查报告（Round 2）

审查日期：2026-11-15。审查对象为当前工作区实际文件，包含未提交、未跟踪的组件；不以已有审查报告中的结论代替源码证据。

本轮只创建本文件；未修改源码、头文件、配置、CMake 或测试，未构建、编译、运行测试、烧录或执行 git commit。文中的测试评价来自阅读测试实现，不表示本轮测试通过。

## 一、范围、结论与限制

已逐项阅读 `main/main.cpp`，`esp_link`、`c3_link`、`esp_net`、`macro_engine`、`telemetry`、`cmd_parser`、`camera_service`、`peo_pinmap` 的实现、公开头文件及组件装配；阅读 `tests/` 中全部 host 测试、运行脚本和 stubs。相机重点追踪 `driver/cam_hal.c`、`target/esp32s3/ll_cam.c`、`target/private_include/ll_cam.h` 及 `driver/esp_camera.c` 的分配、同步、DMA、取还帧和失败清理链。

`components/esp32-camera/` 当前整体未被 Git 跟踪，无法仅凭本仓库 Git diff 确定它相对上游的完整改动集；“哪些相机代码是本地新增”这一历史归属**不确定**。本报告审查的是当前实现，不声称完成了整个第三方传感器驱动库的逐行审计。

辅助只读核对了 `sdkconfig`、`sdkconfig.defaults`、托管 cJSON 配置以及本机 `D:/Espressif/v6.1/esp-idf` 的缓存同步、UART 和 FreeRTOS 内存实现。现有 build 目录只用于定位 SDK 路径，不用旧固件产物证明当前源码正确。

主要结论：STOP 锁存、TX 代次屏障、统一建队和相机缓存行所有权已有较好的基础；仍有 4 项严重问题，以及若干中、低问题。优先处理上位机失联监督、网络入口丢 STOP、宏无反馈继续执行，再确定多控制源仲裁策略。以下分类只是建议的后续处理方式，本轮没有实施修复。

严重度口径：**严重**指存在持续运动、停车请求丢失或控制权冲突路径；**中**指可靠性、数据语义、资源或验证缺口；**低**指影响较小的边界、诊断或维护问题。标注“不确定”的事项不等于已经在硬件上复现。

## 二、问题清单

### R2-01｜严重｜未实现要求的上位机 2 秒心跳停车

- **位置**：`components/esp_net/esp_net.cpp:296`、`:408`、`:459`、`:510`；`components/cmd_parser/cmd_parser.cpp:194`。
- **问题与证据**：`net_client_t` 只有 `tx_since`，没有有效命令/PING 的接收时间。网络循环处理 `select`、`recv` 和发送积压，PING 分支只是：

  ```cpp
  // cmd_parser.cpp:194-198
  if (strcmp(jc->valuestring, "PING") == 0) {
      const cJSON *jt = cJSON_GetObjectItem(root, "t");
      send_ping_ack(cJSON_IsNumber(jt) ? (double)jt->valuedouble : 0.0);
      return;
  }
  ```

  上位机应用停发命令但保持 TCP 连接、仍读取遥测时，`recv == 0`、发送积压和 5 秒 TX 停滞都不会触发。此前 MOTOR 目标可以持续生效，代码中没有要求的 2 秒停车路径。TCP 连接存在不能证明应用控制循环存活。
- **建议修法**：记录控制会话最后一次有效心跳的单调时间，独立于 TX 进展，在有界周期检查 2000ms 截止时间，超时直接锁 STOP 并使旧工作失效；不要靠遥测发送成功续期。消息信封应携带接收时间/会话标识，避免积压命令在稍后解析时伪装成新心跳。现有 JSON 命令和线协议可以保持不变。
- **分类**：**可直接安全修**（补齐题设的 2 秒监督）；离线宏是否豁免、多个客户端谁负责续期，须结合 R2-05 由用户明确。实际停车总延迟仍需硬件测量。

### R2-02｜严重｜网络层提前丢弃旧代次整行，绕过 STOP_ALL 无条件接受

- **位置**：`components/esp_net/esp_net.cpp:458`、`:465`、`:483`；`components/cmd_parser/cmd_parser.cpp:186`、`:420`。
- **问题与证据**：

  ```cpp
  // esp_net.cpp:465、483-484
  if (c.len == 0) c.generation = received_generation;
  // ... 收到换行后
  if (c.generation != esp_link_command_generation()) continue;
  esp_net_rx_message_t msg = {strdup(c.buf), c.generation};
  ```

  若 STOP_ALL 被拆成多次接收，行开始时为代次 g，换行前其他任务触发 STOP 变为 g+1，则整行在网络层直接丢弃。即使另一生产者已经恢复 g+1，这次 STOP 也无法重新停车。解析器 `:186-191` 的 STOP 豁免完全到不了；其 `gen`、PING 豁免也同样受入口限制。带显式当前 `gen` 的合法跨边界消息还可能得不到原本应该返回的 ACK。
- **建议修法**：网络层负责长度、NUL、所有权和资源边界，保留原始 generation 将完整行交给解析器；由解析器先识别 STOP，再拒绝陈旧普通工作。队列满/OOM 继续使用现有独立 STOP。不可使用字符串子串搜索识别 STOP。
- **分类**：**可直接安全修**。不需要修改任何线协议。

### R2-03｜严重｜宏 WAIT 在“从未收到传感器首帧”时跳过安全新鲜度检查

- **位置**：`components/macro_engine/macro_engine.cpp:298`、`:301`、`:307`、`:318`。
- **问题与证据**：

  ```cpp
  // macro_engine.cpp:300-307、318-319
  esp_link_snapshot_t sn;
  if (xQueuePeek(esp_link_sensor_queue, &sn, 0) == pdTRUE) {
      // ...
      if (age_ms > ESP_LINK_STALE_MS) { /* STOP + abort */ }
  }
  if (elapsed >= st->u.wait.timeout_ms)
      step_done = true;
  ```

  没有快照时不会进入 `age_ms` 检查。可达例子：F407 遥测方向未接通，宏先发 MOTOR，再进入长 WAIT；电机会持续运行至 WAIT 超时，随后还可继续后续 ACT。已有一帧再停更会在约 500ms 被中止，而始终没有首帧反而缺少同等保护。
- **建议修法**：将“无快照”和“快照过期”统一纳入反馈不可用处理；需要反馈的步骤设置不超过安全阈值的首帧等待上限，或进入 WAIT 即安全中止。不要将反馈缺失转为普通条件超时继续执行。为避免运动后才发现无反馈，可在含运动/反馈步骤的宏启动前检查反馈就绪。
- **分类**：**可直接安全修**（采用保守停车）；首帧允许等待多久可另行调参。

### R2-04｜中｜macro_abort 不享有 STOP_ALL 的陈旧消息豁免

- **位置**：`components/cmd_parser/cmd_parser.cpp:395`、`:420`、`:422`、`:424`、`:442`。
- **问题与证据**：`handle_macro_abort()` 内确实直接 `esp_link_request_stop()`，但任务入口只有 `stop_all`、`ping`、`tagged_work` 可绕过旧代次过滤；`tagged_work` 只包括 `cmd` 或 `type == "macro"`。因此旧信封的 `{"type":"macro_abort"}` 在 `:424-428` 被丢弃，即使附了当前 `gen` 也不会按该字段处理。R2-02 的网络预过滤还会更早丢弃部分 abort。
- **建议修法**：先确定 abort 语义。若它是用户的全局紧急中止入口，应像 STOP_ALL 一样无条件停车；若它只中止指定旧宏，应明确宏身份/会话归属，避免无声丢弃被误当成已经停车。至少补上真实网络入口的回归场景。
- **分类**：**需用户拍板**。当前能确认的是两个停车入口行为不一致；“旧 abort 应否影响新宏”的产品约定**不确定**。

### R2-05｜严重｜generation 只隔离 STOP，没有控制源/会话仲裁

- **位置**：`components/c3_link/c3_map.cpp:139`、`:149`；`components/cmd_parser/cmd_parser.cpp:257`；`components/macro_engine/macro_engine.cpp:235`、`:285`、`:358`；`components/esp_link/esp_link.cpp:319`。
- **问题与证据**：所有生产者进入同一代次队列；准入条件只有：

  ```cpp
  // esp_link.cpp:319-323
  const bool allowed = !s_inhibited && generation == s_generation;
  cmd.generation = s_generation;
  if (allowed) ++s_pending_commands;
  ```

  宏运行不会抑制手柄或普通 TCP MOTOR；两个 TCP 客户端也没有控制权区分。手柄四轮批次只保证使用同一代次，不保证批次不可交错。因此同一 g 中可以混入宏、手柄、PC 的不同目标，四轮形成混合来源。宏等待的 `s_pending_commands` 又包含全局所有生产者，持续手柄输入可拖延宏正常完成；宏最终 STOP 会使其他源的在途命令失效。generation 对这些同代次冲突不起作用。
- **建议修法**：确定宏、手柄、PC 的优先级/抢占规则和控制租约，将来源与会话 token 放在本地命令信封里，由单一仲裁入口准入。若四轮目标应作为一次控制周期提交，应有本地批次提交/失败处理机制，保持 UART 一帧一命令不变。宏完成应按自己的命令水位判断，并以明确规则停车或移交控制权。
- **分类**：**需用户拍板**。是否允许并行调试类命令、手柄能否立即打断宏、任意旁观客户端断开是否停车，都不能由审查者猜定。

### R2-06｜中｜反馈帧接收新鲜不等于其中各传感器采样新鲜

- **位置**：`components/esp_link/include/esp_link.h:79`、`:82`、`:85`、`:88`；`components/esp_link/esp_link.cpp:136`；`components/macro_engine/macro_engine.cpp:97`、`:305`、`:314`。
- **问题与证据**：SensorData 已有 `dist_timestamp_ms`、`mpu_timestamp_ms`、`ctrl_timestamp_ms`、`ina_timestamp_ms`，但 WAIT 只检查本地 `received_ms`。只要 F407 继续发 112B 帧，某个采样子系统即使卡住，旧距离/IMU/速度仍能被 WAIT 当成当前值；距离条件也不会自动检查对应 `dist_*_status`。
- **建议修法**：保持布局不变，按 WAIT 字段选择相应 F407 时间戳，使用同一帧内 `sys_tick_ms - 子系统时间戳` 判年龄，结合状态字段判有效；处理远端重启与回绕。不要直接拿两个 MCU 的绝对时间戳相减。阈值及哪些字段必须联查状态需明确。
- **分类**：**需用户拍板**。S3 未使用这些有效性信息是确定的；F407 是否在采样停更时仍维持状态为有效、时间戳具体更新约定**不确定**，需联查另一端。

### R2-07｜中｜WAIT 对非有限反馈未拒绝，NaN 可使 != 条件立即成立

- **位置**：`components/macro_engine/macro_engine.cpp:103`、`:131`、`:140`、`:314`；`components/cmd_parser/cmd_parser.cpp:331`。
- **问题与证据**：宏 JSON 的比较目标通过 `isfinite(jv->valuedouble)` 校验，但接收的 IMU float 直接转 double 参与比较，`case MACRO_OP_NE: return v != ref;` 对 NaN 返回 true。CRC 只检查传输完整性，不能证明 float 有限；Infinity 也可意外满足大小条件。
- **建议修法**：比较前校验反馈 `isfinite(v)`，将无效值纳入反馈故障处理，不能把 NaN 解释为条件达成；再结合 R2-06 的状态/采样年龄检查。
- **分类**：**可直接安全修**。F407 实际是否会输出非有限值**不确定**，这里确认的是接收端缺少防御。

### R2-08｜中｜电机/PID 参数仅检查线类型范围，没有硬件工作范围

- **位置**：`components/cmd_parser/cmd_parser.cpp:87`、`:95`、`:115`；`components/c3_link/include/c3_link.h:34`。
- **问题与证据**：普通命令允许 `speed` 取整个 int16 范围、`speed_d100` 取整个 int32 范围，PID 只要求有限且不超过 `FLT_MAX`。手柄有 `XB_MAX_SPEED_D100=4000` 的本地限幅，PC/宏却可提交远超它的速度或极大的、负的 PID 参数。
- **建议修法**：由电机/驱动/闭环规格确定工作范围，在普通命令和宏共用的参数打包入口拒绝超范围值，保留现有字段、ID 与编码。不要未经确认把 4000 直接当作所有场景的最终上限。
- **分类**：**需用户拍板**。F407 是否已有充分限幅和 PID 保护**不确定**，不能仅据此断言会损坏硬件。

### R2-09｜中｜网络队列虽有条数上限，字节内存预算仍较大且热路径分配频繁

- **位置**：`components/esp_net/include/esp_net.h:36`；`components/esp_net/esp_net.cpp:288`、`:339`、`:473`、`:484`；`components/telemetry/telemetry.cpp:106`、`:173`；`components/cmd_parser/cmd_parser.cpp:296`；`sdkconfig:2354`、`:2356`、`:2359`。
- **问题与证据**：RX 队列深 32，单行上限 16384B，每行再 `strdup`；仅满队列字符串就可占 `32 × 16385 = 524320B`，四个客户端行缓冲另有 `4 × 16385 = 65540B`，还未计正在解析/等待入队的行、cJSON 节点及宏程序。它们不是队列句柄本身占这么大。小分配优先内部堆的配置下，50Hz 遥测构建/删除 cJSON 树和 TCP `realloc` 反复消耗内部堆；大行可去 PSRAM，PSRAM 缺失时则回退/失败。
- **建议修法**：建立按字节的总 RX 预算，使用可复用行块/宏对象池；大块普通应用数据明确偏好 PSRAM，小型安全状态、RTOS 对象、DMA 描述符保留内部堆。遥测可用有界预分配输出和复用缓冲减少堆操作。日志同时记录内部 8bit 堆余量、最大连续块和 PSRAM 余量。
- **分类**：**可直接安全修**（资源预算/计数及复用）；阈值和池大小要按负载确定。当前是否实际 OOM、碎片化程度**不确定**，不能由“有 8MB PSRAM”反推内部堆充足。

### R2-10｜中｜ACK 与 50Hz 遥测共用 8 深 TX 队列，ACK 可以静默丢失

- **位置**：`components/esp_net/esp_net.cpp:289`；`components/cmd_parser/cmd_parser.cpp:130`、`:141`、`:156`；`components/telemetry/telemetry.cpp:208`。
- **问题与证据**：`esp_net_tx_queue = xQueueCreate(8, sizeof(char *));`；ACK 和遥测都非阻塞 `xQueueSend`，满后直接 `free(out/json)`。网络任务处理 RX 或暂未调度时，遥测可占满队列，使 STOP、命令或 PING ACK 消失。ACK 又广播到所有客户端，没有本地请求来源信息。
- **建议修法**：本地分离可靠控制响应与可丢弃最新值遥测，优先发送 ACK，遥测用覆盖/合并策略；为响应设置有界预算和丢失计数。保留 ACK JSON 格式，利用本地 session 信息选择回复目标；广播是否属于既定协议需先确认。
- **分类**：**可直接安全修**（优先级和队列预算）；回复目标策略**需用户拍板**。不应对 ACK 改为无限等待，否则会阻塞命令解析及后续停车请求。

### R2-11｜中｜UDP 连续突发发送缺少对 ENOMEM/ENOBUFS 的节流反馈

- **位置**：`components/esp_net/esp_net.cpp:225`、`:242`、`:250`、`:272`、`:275`；`components/camera_service/camera_service.cpp:54`、`:317`、`:351`；`sdkconfig:2617`。
- **问题与证据**：每帧向最多 4 个不同地址连续发送所有分包，第一个 `sendto` 失败后中止本帧，后续仍以固定 24fps 重试；错误仅计数打印，没有退避或动态降载。动态 TX buffer 数量为 64。包被 lwIP 接受不代表已经发到空口，上位机接收缓冲也不能缓解 ESP 内部发送池耗尽。代码 `:275-278` 中“包间延时会打爆队列”的因果表述不足以成立，延时主要改变吞吐/时延，须实测。
- **建议修法**：保留 1400B 分片和大端头，按 ENOMEM/ENOBUFS/EAGAIN 分类统计，采用有界退避、整帧降频/降质、发送预算等措施；不要无限重试当前帧或持有 fb 长时间阻塞。按客户端数、帧大小、发送用时关联失败率，避免只盲增 buffer 数量。
- **分类**：**需硬件实测才能定**。突发与缺少反馈是确定的；该配置下是否饱和、最佳 pacing 和帧率**不确定**。当前 1000Hz tick 下毫秒延时与注释里的旧 10ms tick 也不同。

### R2-12｜中｜JPEG “结构校验”实际是标记子串搜索，不能证明 SOF/SOS 合法

- **位置**：`components/camera_service/camera_service.cpp:261`、`:274`、`:285`、`:289`、`:315`。
- **问题与证据**：SOF0 与 SOS 均在整个字节区间内搜索 `FF C0` / `FF DA`，没有按 JPEG 段长度跳过 APP、COM、DQT 等段；“从 SOF0 后搜”也不能排除后续段载荷中的假标记。SOS 的 `Ls` 只用于 `ent_start = sos_off + 2 + ls`，甚至没有拒绝 `Ls=0/1`，SOF0 长度、图像尺寸、分量数与 SOS 对应关系均未检查。因此含假标记的畸形帧仍可能进入发送分支；某些合法 marker fill 形式又可能被当前熵扫描拒绝。
- **建议修法**：用有界 JPEG marker walker，逐段验证长度与边界，只接受所需 baseline 单扫描子集；检查 SOF0、SOS 的最小长度和分量约束，再扫描熵区与 EOI。无需在 MCU 上完整解码，校验结果也不能声称证明熵编码完全可解码。
- **分类**：**可直接安全修**。现有代码的多数直接读取有前置边界，未据此认定当前存在已证实的越界读；问题是合法性判断不足。

### R2-13｜中｜PSRAM JPEG 长度按 DMA 块向上估算，旧尾部 EOI 风险仍需验证

- **位置**：`components/esp32-camera/driver/cam_hal.c:399`、`:406`、`:407`、`:828`、`:839`、`:847`；`components/esp32-camera/target/esp32s3/ll_cam.c:560`。
- **问题与证据**：VSYNC 分支额外 `cnt++`，再令 `frame_buffer_event->len = cnt * dma_half_buffer_size`。它是按 1024B 块估算的覆盖范围；`len <= recv_size` 只证明未超容量，不能证明最后整块字节都由本帧更新。EOI fallback 会在该范围倒序搜最后一个 EOI，而 `:845-846` 注释断言整个 `[0,len)` 均为本帧写入。
- **风险边界**：**不确定**。最后不足一个 DMA 块时硬件究竟写了哪些字节、描述符 length 是否可直接反映有效字节、OV5640 是否填充尾部，需要板上抓取。若尾部保留上一帧内容，缓存 invalidate 只能读到 PSRAM 的真实旧内容，不能把它变成本帧数据；R2-12 的弱结构检查不能完全排除此类混合帧。
- **建议修法**：测量 VSYNC/EOF、描述符写入长度与实际 JPEG EOI 的关系；优先用可靠的实际 DMA 写入界限，必要时增加尾部 canary/抽样取证，确认后再决定 EOI 搜索策略。不要靠增大 fb 或只检查地址对齐解决内容周转问题。
- **分类**：**需硬件实测才能定**。

### R2-14｜中｜相机任务退出后分辨率请求仍 ACK 成功，且成功只表示入队

- **位置**：`components/camera_service/camera_service.cpp:44`、`:118`、`:153`、`:212`；`components/cmd_parser/cmd_parser.cpp:208`。
- **问题与证据**：

  ```cpp
  // camera_service.cpp:44-47
  bool camera_request_resolution(uint8_t framesize) {
      if (!s_cam_cmd_queue) return false;
      return xQueueOverwrite(s_cam_cmd_queue, &framesize) == pdTRUE;
  }
  ```

  无 PSRAM、初始化失败、回退失败时相机任务删除自己，但命令队列仍存在。之后 CAM_RES 不断覆盖无人消费的队列，解析器仍回 ACK status=0。正常热切换发生回退时，上位机也只收到之前的成功 ACK，无法从当前反馈判断实际分辨率。公开 API 还接受任意 uint8 framesize；当前 JSON 入口有白名单，但其他未来调用者无该保障。
- **建议修法**：维护受同步保护的相机就绪/故障状态，请求接口检查就绪并自行验证枚举；区分“已受理”与“已应用”，至少通过现有日志/状态通路提供实际配置和失败结果。若要新增线上结果字段，另行确认，不能把排队成功称作硬件已生效。
- **分类**：**可直接安全修**（内部状态与范围检查）；线上状态反馈形式**需用户拍板**。

### R2-15｜低｜读帧前缓存同步返回值被忽略

- **位置**：`components/esp32-camera/driver/cam_hal.c:101`、`:109`、`:115`、`:123`、`:834`、`:858`；对照 `components/esp32-camera/target/esp32s3/ll_cam.c:171`。
- **问题与证据**：`cam_drop_psram_cache()` 与 `cam_flush_psram_cache()` 返回 void，直接丢弃 `esp_cache_msync()` 返回值；DMA 开始前却检查它并仅容忍 `ESP_ERR_NOT_SUPPORTED`。若读帧前同步失败，调用者仍会读取、校验、发送缓存内容。
- **建议修法**：同步 helper 返回结果，在 PSRAM 帧同步失败时丢帧、记录分类计数；非缓存 DRAM 的 NOT_SUPPORTED 单独处理。保留现有缓存行对齐和完整分配区所有权，禁止靠放宽对齐检查来掩盖错误。
- **分类**：**可直接安全修**。当前正常 S3 地址、长度对齐条件下通常不会失败，实际发生概率**不确定**，因此不升级为已证实的缓存损坏问题。

### R2-16｜中｜host 回归没有覆盖 C3 实现，也不能证明真实双核与队列阻塞行为

- **位置**：`tests/net_host_test.cpp:13`、`:21`、`:27`、`:46`；`tests/net_run_host_tests.py:22`；`tests/link_regression.cpp:15`；`tests/stubs_net/freertos/FreeRTOS.h:15`；`tests/stubs_net/freertos/queue.h:12`；`tests/stubs_link/freertos/task.h:11`。
- **问题与证据**：网络套件替换了 esp_link 安全 API；critical 宏是空操作，`xQueueSend` 忽略等待时间。link 套件使用真实 esp_link 和宏，但靠人工 hook/异常切出模拟任务；通知 stub 明示没有调度器。两套都未编入 `c3_link_rx.cpp` 或 `c3_map.cpp`。因此 500ms 心跳、回中重武装、跨 STOP 的输入序号、C3 重连/回绕和四电机批次，没有对应真实实现回归；50ms 入队等待与跨核交错也未被证明。
- **建议修法**：后续为真实 C3 RX/map 建 fixture，覆盖首帧/静默/坏流/重连/STOP 后回中/回绕；给安全提交、发布、TX dequeue、drain、resume 设置可控交错点，覆盖真实 generation 实现。明确区分确定性时序测试与真正的并发压力测试，不能用空锁 stub 推导无竞态。
- **分类**：**可直接安全修**（补验证，不是本轮运行测试）。硬件双核时延、UART 和 DMA 仍另测。

### R2-17｜中｜host cJSON 编译未复用固件的 16 层嵌套限制

- **位置**：`tests/net_run_host_tests.py:18`、`:24`；`managed_components/espressif__cjson/cJSON/cJSON.h:136`；`managed_components/espressif__cjson/CMakeLists.txt:5`；`sdkconfig:4399`。
- **问题与证据**：host 直接编译 `cJSON.c`，参数列表没有 `-DCJSON_NESTING_LIMIT=16`；其头文件默认值是 1000。固件则由托管组件 CMake 传入 `CONFIG_CJSON_NESTING_LIMIT=16`。同一 JSON 在两端可能有不同接受结果，host 也没有复现固件的深度保护边界。
- **建议修法**：host 显式复用目标嵌套限制，并加入边界深度输入和最大合法宏 JSON。`tests/verify_firmware.py:37` 的旧构建配置检查不能替代 host 编译参数一致性。
- **分类**：**可直接安全修**。本报告没有把固件误判为采用默认 1000 层。

### R2-18｜低｜C3 可变长重同步成功后清空整个缓冲，会吞掉已收尾缀

- **位置**：`components/c3_link/c3_link_rx.cpp:224`、`:225`、`:229`、`:231`。
- **问题与证据**：

  ```cpp
  if (s_rx_count >= frame_total) {
      // ... CRC 成功
      handle_frame(&s_rx_buf[3], payload_len);
      s_rx_count = 0;
      break;
  }
  ```

  正常逐字节接收时 count 等于帧长；但拒绝较长外层候选并滑到较短内嵌有效帧时，可能已经缓存了有效帧后面的下一帧前缀，此时 count 大于 frame_total。清零会把该尾缀也丢掉，造成额外丢帧。这不等于永久失步或缓冲越界。
- **建议修法**：成功消费 frame_total 后减去该长度，memmove 保留剩余字节，继续处理缓冲；固定 112B 的 esp_link 不存在相同的可变长度组合，不能简单类推。
- **分类**：**可直接安全修**。

### R2-19｜中｜C3 UART 持续立即返回错误时存在忙循环

- **位置**：`components/c3_link/c3_link_rx.cpp:208`；对照 `components/esp_link/esp_link.cpp:188`。
- **问题与证据**：C3 分支对 `uart_read_bytes(...) != 1` 只清候选并 `continue`。正常读超时会阻塞，因此不是正常情况下空转；但驱动错误若立即返回，则 prio5/core0 无退避持续循环。esp_link 对应路径有 `vTaskDelay(pdMS_TO_TICKS(1))`。
- **建议修法**：区分超时与负错误返回，连续错误时有限退避并锁 STOP/禁用链路或做受控重建；记录限流错误日志。避免对正常超时再增加大延时而拖慢 500ms 截止时间。
- **分类**：**可直接安全修**。初始化后是否会触发持续驱动错误**不确定**，这是确定存在的异常处理缺口。

### R2-20｜中｜遥测有值/新鲜/一致三个概念未完全区分

- **位置**：`components/telemetry/telemetry.cpp:111`、`:144`、`:160`、`:187`、`:199`、`:202`。
- **问题与证据**：没有传感器快照时置 `received_ms=0`，随后仅用 `now-0 > 500` 判 stale；启动前 500ms 可能输出未带 `link_stale` 的零值数据，注释“received_ms=0 会让 link_stale=true”并不总成立。手柄只按队列里有没有值决定输出，不看 snapshot 年龄/input_fresh；RX 任务停止更新时可能长期显示 connected=true。sensor、gamepad、宏进度和 gen 又是分次读取，JSON 是各自快照的拼接，不是一个全局事务快照。
- **建议修法**：显式保留 `have_sensor`，使无数据立即 stale；手柄展示按本地年龄和连接/输入有效性派生状态。若 UI 要使用宏状态和 gen 作控制决策，增加本地一致性校验（构建前后读 generation，发生变化则重试/丢帧），或给进度附本地 generation 供合并判断；不要把拼接快照宣传为同一时刻的系统状态。
- **分类**：**可直接安全修**（无数据/年龄与内部一致性检查）；线上新增字段另确认。现有 S3 单队列内部复制是一致的，这里不是结构体撕裂问题。

### R2-21｜中｜任务创建失败后的 STOP 可被既存生产者恢复，非永久故障锁

- **位置**：`main/main.cpp:43`、`:46`、`:48`、`:74`、`:75`、`:77`；`components/c3_link/c3_map.cpp:123`。
- **问题与证据**：`start_task()` 失败后请求一次 STOP，app_main 永久 delay；它没有阻止其他已经运行的任务 resume。例如 telemetry 或 cmd_parser 创建失败时，c3_map 可能已经运行，新的回中输入可调用 `esp_link_resume_commands()`，之后偏杆仍能控制车辆。日志里的 “allocation failed; STOP latched” 不能代表不可恢复的启动故障状态。
- **建议修法**：区分正常可恢复 STOP 与启动健康故障。必需任务缺失时设置独立不可被普通输入清除的健康屏障；非必需任务则明确降级，并让装配继续或报告具体不可用能力。不要仅把 app_main 卡住当作全系统停止。
- **分类**：**需用户拍板**（哪些任务缺失必须禁止行驶）；确定策略后可通过本地状态修复，不必改协议。

### R2-22｜低｜旧组件运行依赖未见残留，但多处注释已与实现不符

- **位置**：`components/esp_link/include/esp_link.h:14`、`:115`；`components/esp_net/include/esp_net.h:8`、`:16`；`components/esp_net/esp_net.cpp:369`；`components/telemetry/include/telemetry.h:8`；`components/cmd_parser/include/cmd_parser.h:12`；`tests/net_README.md:22`、`:52`。
- **问题与证据**：仍写 `xbox_map` 生产者、任务内建队、BLE/WiFi 串行启动、遥测阻塞等 sensor 帧、宏暂未实现；测试文档写 1 秒 TX 超时和历史 44/44，而代码超时已为 5000ms、测试列表已增加。上述内容会误导后续并发审查和故障诊断。
- **建议修法**：同步接口注释与实际装配；测试结果注明对应版本和时间，不能把历史数字当作当前通过证明。`xbox_state_t`、`xbox_state_queue`、XB_* 目前仍被 C3 映射/遥测实际使用，不能据旧名称当死代码删除。
- **分类**：**可直接安全修**。在本轮范围的实际 include/REQUIRES/任务入口中，未发现仍运行旧 `xbox_service` / `xbox_controller` 的依赖；注释提及和兼容状态类型不等于旧 BLE 服务仍在运行。

### R2-23｜中｜接线、轮序/方向和相机外部 XCLK 仍是未完成的硬件约定

- **位置**：`components/peo_pinmap/PEO_PINMAP.h:55`；`components/c3_link/c3_map.cpp:9`、`:11`、`:114`、`:141`；`components/c3_link/include/c3_link.h:29`、`:34`；`components/camera_service/camera_service.cpp:50`、`:53`。
- **问题与证据**：C3 UART17/18 注释仍为待确认占位；手柄假设 0/1 左轮、2/3 右轮，直接使用 `joy_ly - 32768` 作油门，死区/满速仍 TODO；相机 `GPIO_NUM_NC` 不输出 XCLK，PLL 却固定按 `24000000` 计算。S3 与 F407 的 UART 外设名称不同本身不是错误：S3 UART1 接对端 USART2 是合理的。
- **建议修法**：架空车轮逐个确认 motor ID、前后转向符号与零速释放，标定死区/速度；对照 C3 接线确认 TX/RX；确认模组是否确有外部时钟及其频率。加入 pin 唯一性/禁止占用 PSRAM 引脚的静态约束可减少后续回归。
- **分类**：**需硬件实测才能定**。当前引脚清单未见重复，也没有占用注释保留的 GPIO35/36/37；不能据 TODO 断言现有实物接错。

### R2-24｜低｜Xbox 位图 bit15 已定义但遥测按钮表不输出

- **位置**：`components/c3_link/include/c3_link.h:58`；`components/c3_link/c3_link_rx.cpp:108`；`components/telemetry/telemetry.cpp:32`、`:48`、`:167`。
- **问题与证据**：`XB_BTN_XBOX = 1U << 15` 被状态帧完整读入 buttons，但 `k_btn_names` 只列 0~14 位；Xbox 键单独按下时 `buttons` JSON 不体现该位。
- **建议修法**：确认上位机已有按钮名字约定，再决定是否补表；如果刻意不展示系统键，应写明过滤规则。
- **分类**：**需用户拍板**。这是确定的本地映射遗漏/过滤行为，但是否属于协议违约**不确定**，不能擅自发明新按钮字符串。

## 三、重点路径核对：已具备的保护与未能证明的边界

### 1. 并发、STOP 与 generation

- `main/main.cpp:63-68` 在创建任务前统一建队并锁启动 STOP，解决了跨核消费者先 peek NULL 的装配时序问题。`esp_link.cpp:149` 与 `c3_link_rx.cpp:150` 在启动任务前建 CRC 表，当前装配下不能把 CRC 的 lazy-init 分支直接判为双核竞态。
- `components/esp_link/esp_link.cpp:38-47` 在安全锁内更新 generation、inhibited、stop_pending；STOP 不占普通队列容量。TX 在 `:254-269` 优先处理 STOP，成功 drain 后仅对匹配代次清 pending。新的 STOP 或失败重试不会被旧发送结果清除。
- 普通命令 check→enqueue 确实跨越锁外窗口（`:319-329`），但命令携带旧 generation，TX 在 `:280-284` 再检查，因此旧命令排进队列不等于能穿过 STOP。当前实现的主要风险不是“少一个入队前检查”，而是 R2-02 的上游丢弃和 R2-05 的同代次控制源混用。
- `esp_link.h:118-124` 明确约定 TX 通过检查即提交：STOP 到来前已提交的一帧可以尚未物理写 UART。单一 TX 任务保证该帧在 STOP 前，不能要求已经提交的字节撤回。UART 调用在锁外，避免跨核自旋锁内阻塞。若要求连这一个旧帧也不允许，需要重定义提交契约，不能只加重复检查声称窗口消失。
- TX 通知只唤醒，真实数据在锁存状态与队列（`esp_link.cpp:272-278`、`:329-336`）；查空与等待间到达的工作保留通知，未发现确定的丢唤醒路径。公开队列句柄若未来被绕过封装直接发送，将破坏 generation、pending 和通知约定；当前审查的生产者使用封装，未发现该绕行。
- `s_pending_commands` 在发布前增加，失败时仅回滚同代次计数（`:322-323`、`:339-341`）；STOP 重置旧计数，成功物理 TX 后减当前计数（`:290-293`）。没有据静态代码发现当前路径的必然计数泄漏。它不是来源级完成水位，详见 R2-05。

### 2. 失联与宏中止/完成

- C3 RX 每轮先检查有效状态帧年龄，持续坏字节流也执行 500ms 监督（`c3_link_rx.cpp:183-205`）；未知 type/错误长度不刷新心跳。静默读等待按剩余截止时间缩短。该路径不是单纯依赖 read timeout 才检查，不能误报为坏流可无限续命。
- 500ms 是算法阈值，不是已实测的最坏停车时间；还包括任务调度、已有提交帧、UART drain、IDLE 以及 F407 执行延迟。`s_last_frame_ms==0` 表示首帧前不监督（`:74`、`:187`），启动 STOP 与 map 的就绪条件共同限制手柄；PC 是否可在无 C3 情况行驶属于 R2-05 的控制策略。
- `c3_map.cpp:83-87` 使用 disconnect_generation 捕获两次采样间的断连；`:100-129` 要求 STOP 后新的通知、时间和回中，`:149-153` 对整批使用同一 generation，任一电机入队失败就锁 STOP。`input_fresh` 直接信任 C3 的判定（`:117-119`）；本仓库没有 C3 HID 实现，无法证明其 5 秒判定与重连序号语义。
- 宏检测 generation 变化后中止（`macro_engine.cpp:201-203`、`:246-254`），正常结束等待 `esp_link_commands_drained()`，没有用“队列空”冒充物理发送完成（`:354-365`）。末尾 ACT 不会因立刻发结束 STOP 而必然失效；STOP_ALL ACT 则特意按中止终结（`:280-283`）。
- ACT duration 从本地提交计时而非 F407 已执行计时（`:264-266`）；`DONE` 在结束 STOP 提交请求时上报，并未等待其被 F407 确认（`:171-174`）。S3 `uart_wait_tx_done` 只能证明自身 UART 发完；没有事务序号 ACK，也没有本轮 F407 源码/硬件证据，不能保证对端收到/执行。把 DONE 定义为“对端已停稳”目前没有依据。
- 纯 ACT/GOTO 宏、普通 PC/手柄命令没有统一的 F407 RX 停更停车监督。是否允许单向控制链路继续工作、是否要求全局反馈失联停车需用户定义；本轮不把未声明的策略编造成现有保证。无反馈 WAIT 的确定缺口已列 R2-03。

### 3. 协议与缓冲边界

- esp_link 使用 `AA 55 + LEN + payload + CRC16`，CRC 覆盖 SOF/LEN/payload（`esp_link.cpp:203-206`、`:233-235`），CRC 低字节先发；TX 最大参数 8B，帧数组 14B（`:225-236`），当前路径长度匹配。RX 明确要求 LEN=112，不受命令侧 64B 常量错误限制（`:119`、`:201`）。滑动候选帧 CRC 失败后保留后缀，数组满时必消费/拒绝，未发现确定的 RX 写越界。
- `esp_link_sensor_t` 的本地人工偏移核算如下，结构大小还有 `esp_link.cpp:154-155` 的 112B static_assert：

  | 偏移（字节） | 字段 |
  | --- | --- |
  | 0 / 2 / 4 | 三路距离 uint16 |
  | 6 / 7 / 8；9~11 | 三路状态；pad1 |
  | 12 | dist_timestamp_ms |
  | 16~31；32~43；44 | quat；accel；mpu_timestamp_ms |
  | 48~63；64~79；80 | speed_d100；odometer；ctrl_timestamp_ms |
  | 84~91；92~99；100 | current_mA；bus_voltage_mV；ina_timestamp_ms |
  | 104；108 / 109；110~111 | sys_tick_ms；last_cmd_id/status；pad2 |

  字段依据 `components/esp_link/include/esp_link.h:70-93`。建议后续增加 offsetof 断言锁定布局，大小断言本身不能锁定每个字段。当前 ESP32-S3 为小端、float 按本地表示 memcpy；跨端布局/浮点约定只见注释，未提供 F407 `SensorData_t` 和权威 PROTOCOL 文件，因此“两端逐字节完全一致”仍**不确定**。
- C3 当前要求 payload 26B，包含 type(1)+body(25)（`c3_link_rx.cpp:67-68`、`:133-139`）。body 0/4 是 seq/disconnect_generation，8/9/10 是 connected/battery/input_fresh，11/13/15/17 是四轴，19/21 是扳机，23 是 buttons（`:97-108`）；最后一次 rd16 访问到 body[24]，本地边界正确。接收前已校验 CRC。未提供 C3 端源文件，不能声称已验证其实际发帧偏移。
- UDP 8B 头各 uint16 手工大端编码，payload 最大 1400B，静态包数组 1408B，长度和包数先约束（`esp_net.cpp:194-205`、`:222-240`）；不能与 esp_link 的小端要求混淆。frame_id uint16 自然回绕；接收端跨回绕、旧帧超时重组是否正确**不确定**，本仓库没有上位机重组器。
- JSON 行在换行时有额外 1 字节终止空间；超长/NUL 整个连接拒绝并 STOP（`esp_net.cpp:466-491`），不是执行超长行的截断前缀。`cJSON_ParseWithOpts(..., true)` 拒绝尾随垃圾（`cmd_parser.cpp:413`）。数值在转换前检查整数/有限/类型范围，goto 越界映射到结束哨兵（`:317-320`），未见先窄化后校验造成的确定溢出。
- 无符号年龄差适用于通常的 uint32 毫秒回绕；不能无限期跨完整 2^32ms 周期推断年龄。回中时间比较采用 int32 差（`c3_map.cpp:122`），假设两事件差小于半周期。没有据此把普通 49.7 天回绕泛化成必然故障；长期无人更新快照、精确零时间戳等边界应在后续验证中覆盖。

### 4. 相机 DMA、缓存与 fb 生命周期

- 当前 sdkconfig 为 64B 数据缓存行（`sdkconfig:2457-2459`）。分配以 max(16、DMA 对齐、cache line) 对齐，长度也向该对齐取整，保存 `fb_dma_sync_size`，offset 要求为 0（`cam_hal.c:529-554`、`:578-587`）。因此 invalidate 扩展的首尾仍属于自己的分配块，不再依赖“只对齐地址”或“额外挪 16B”。这是应保留的修复。
- DMA 开始前同步整块拥有区域，先 M2C invalidate 再启动 link（`ll_cam.c:162-177`、`:195`），读帧前再次 invalidate（`cam_hal.c:834`、`:847`、`:858`）。YUV→灰度 CPU 转换先读同步再 C2M writeback（`:873-876`），当前项目只用 JPEG。符合 ESP32-S3 CPU cache 与 DMA 非自动一致的模型，参见 [Espressif 内存同步说明](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/system/mm_sync.html)。该文档是 API 背景；本机 v6.1 `components/esp_mm/esp_cache_msync.c:96-159` 也已只读核对。
- `camera_task` 所有正常 fb 分支最终还帧：VIDEO_OFF 分支 `camera_service.cpp:227-236`，发送/校验路径 `:345`。分辨率切换在下次取帧前执行，当前单消费者路径未见持有 fb 时 deinit。UDP 同步复制进包再 send，未保存 fb 指针到异步应用队列。
- 失败清理 `cam_deinit()` 遍历释放帧、描述符与队列（`cam_hal.c:720-756`），对齐分配的真实首地址得以保留。host 对齐测试检查分配区间和源码调用顺序，但没有执行实际 GDMA 或 v6.1 堆元数据实现（`tests/camera_dma_alignment_host.py:29-49`、`:126-153`、`:161-167`）；它不能证明硬件缓存行为与 R2-13 的内容有效长度。
- 相机对象/重配置接口并非通用多调用者安全：当前所有实际 init/deinit/fb_get/return 都归 camera_task 所有，暂不能把未被调用的并发重配置 API 当成当前必现竞态。未来不要从命令任务直接调用 `esp_camera_reconfigure()` 或 `esp_camera_set_psram_mode()`。

## 四、优化分析

### 内存总体评价

帧缓冲明确去 PSRAM、DMA 描述符去内部 DMA 堆，方向正确。RTOS 队列及任务栈不能因为启用了 PSRAM 就假定自动外移：本机 SDK `D:/Espressif/v6.1/esp-idf/components/freertos/heap_idf.c:14-20`、`:43-55` 将默认 FreeRTOS 动态分配指向内部 8bit RAM。

`main/main.cpp:70-79` 的 9 个显式任务栈总计 **52224B（51KiB）**；相机驱动另建一个 `cam_task`，栈 **8192B**、core1、优先级 `configMAX_PRIORITIES-2`（`cam_hal.c:699-703`，`sdkconfig:4380-4382`），所以相机启动后不止 9 个运行任务。仅这 10 个任务栈已是 **60416B（59KiB）**，还未算 TCB、WiFi/lwIP、Arduino、事件循环和系统任务。这里是配置预算，不是实测峰值。

按当前 PSRAM JPEG 分配公式 `recv_size+1024` 再向 64B 对齐、4 个 fb 估算（`camera_service.cpp:59-71`、`:148`，`cam_hal.c:540-552`），QQVGA/QVGA 各约 132KiB，VGA 约 304KiB，SVGA 约 590KiB，不含描述符/队列。8MB PSRAM 对这些 fb 有容量余地，但大量小 cJSON、动态 WiFi TX 池和任务栈仍竞争内部堆。应优先做字节预算、复用和内部最大连续块观测，再决定缩栈/改 buffer 数；没有 high-water 实测时，不建议直接压缩 esp_tx 3072B 或解析器 6144B 栈。

### 并发总体评价

队列快照、独立 STOP 锁存与发送提交点的设计基本合理，未发现必须把 UART 操作移入自旋锁的理由。当前更重要的是**入口豁免闭环与控制权闭环**：网络过滤不能抹掉紧急操作；generation 不能替代来源/会话仲裁。宏状态、telemetry gen 与多来源 drain 需要明确一致性契约。优先修 R2-01/02/03，再落实 R2-05，随后补 R2-04/06/07/20。

### 性能总体评价

STM32 50Hz × 117B ≈ 5850B/s，C3 50Hz × 31B ≈ 1550B/s，正常 UART 链路带宽余量充足；逐字节驱动调用和坏流滑动 memmove 是 CPU 成本，未见需先牺牲可靠性才能优化的证据。可以批量读取小块后保持同一字节解析器，但必须保留有界处理和 C3 有效帧截止检查，避免一批无限处理拖延监督。

TX 每帧 drain 再 delay 2 ticks 是 F407 IDLE 契约的一部分（`esp_link.cpp:237-240`）；当前 tick=1000Hz（`sdkconfig:3048`），不能根据旧 100Hz 注释估算它。网络 select 20ms、队列发送可能等待 50ms，若一个 recv 块包含多行且消费慢，会连续等待并延后其他客户端处理；后续可给每轮 RX 总等待/工作量设预算。WiFi、cam_task 的高优先级工作和 50Hz JSON 分配也会影响应用任务调度。实际最坏执行时间、栈水位、UDP ENOMEM 频度与 500ms/2s 停车总时延都**不确定**，须在联机满负载下测量。

视频优化顺序应是发送失败反馈与降载、帧内容有效性、再考虑调 JPEG 质量/fb_count。VIDEO_OFF 仍持续取还帧（`camera_service.cpp:227-236`），所以关闭网络视频不等于关闭传感器/DMA；若需要省电，需另行定义停采/重启策略及恢复时间，不能只改发送开关。普通运行回归则优先减少 JSON/堆热路径和分离 ACK，避免把单纯提高优先级当作所有回压问题的修法。

### 健壮性总体评价与优先修复顺序

已有长度/CRC/有限数值检查、失败释放、非阻塞 TCP 部分发送、队列满降级和 UART 初始化错误处理，优于只关注正常收发的实现。剩余主要短板是“没有数据”与“已有数据过期”的不对称、部分异常的静默丢弃、异步受理被当作成功，以及测试边界与实际平台不一致。队列初始化失败当前由 configASSERT 终止（`esp_link.cpp:148`、`c3_link_rx.cpp:149`、`esp_net.cpp:290`、`macro_engine.cpp:192`），当前配置启用了断言（`sdkconfig:1463`）；不能声称所有组件都支持 NULL 队列降级。若将来关闭断言或要求 OOM 常驻降级，需要统一返回状态并在创建生产者前决定运行模式，尤其启动 STOP 尚未发到 F407 时不能假定外部电机已经停住。

建议修复顺序：

1. **停车闭环**：R2-01（2 秒监督）、R2-02（完整入口 STOP 豁免）、R2-03（无反馈 WAIT）；明确 R2-04 与 R2-05 的 abort/控制权/离线宏策略。
2. **反馈与工作边界**：R2-06/07/08，以及启动失败健康屏障 R2-21。
3. **资源与反馈可靠性**：R2-09/10/14/20，避免 OOM、丢 ACK 和假成功。
4. **视频与硬件闭环**：R2-11/12/13/15/23，保留现有对齐修复，测 DMA 内容周转和网络负载，不凭 host 对齐测试宣称硬件通过。
5. **验证与维护**：R2-16/17/18/19/22/24。先为严重路径补真实实现的交错场景，再在另一次获准验证中构建/测试和上板；本轮没有执行这些动作。

最终验收应分别记录：STOP 请求到 F407 收到/执行的时间、C3/PC 超时到停车的时间、内部堆最低值和最大连续块、各任务栈高水位、有效/坏 JPEG 比例、不同客户端数量下 UDP 失败率。S3 复位、UART 断线、F407 重启时的最终电机安全还取决于 F407 自身 watchdog/默认输出；本仓库不足以确认，明确标为**不确定**，不作跨端安全保证。
