# Network, parser, macro and telemetry host regressions

Run from any working directory (Windows + MinGW):

```bash
TMPDIR='C:/Users/Danny/AppData/Local/hermes/profiles/danny-v1/cache/scratch' python "C:/Users/Danny/Documents/Espressif Files/peo_rover_esp32s3/tests/net_run_host_tests.py"
```

`CXX` overrides the compiler, `CJSON_DIR` overrides the existing cJSON source directory, and `HERMES_NET_TEST_TMP` overrides the scratch directory. 编译器使用 `CXX` 指定值或 PATH 中的 `g++`，以当前脚本为准。The socket HAL uses Winsock's data types, so this harness currently targets Windows/MinGW.

当前维护说明：Round 2 批次 4b 已静态适配新的 RX 消息元数据、处理函数签名、ACK 专用队列及 WAIT 有效采样夹具，并补充 fatal fault 安全边界替身；本轮未编译或运行测试，后续由监督者验证。测试数量以 `net_host_test.cpp` 的注册表及当前 `net_run_host_tests.py` 的发现结果为准，包含 `gen_stale`、`gen_resume`、`gen_stop_all` 三项。下文红绿结果属于历史版本，不能当作当前源码的通过证明。

直接调用 `handle_command` / `handle_macro` 时固定传入 `slot=0`、`received_ms=0`；手工注入 RX 队列使用完整 `esp_net_rx_message_t`，填入实际 `line_bytes` 并预留字节预算，解析器消费时归还。夹具直接包含生产 `esp_net.cpp`，复用其 ACK 队列、活动记录和预算接口实现，不再重复定义这些符号；esp_link 安全接口仍由本套件的替身提供，不代表真实锁/并发行为已验证。

R2-17：公共编译参数加入 `-DCJSON_NESTING_LIMIT=16`，独立 `-fsyntax-only` 与主程序/cJSON 编译均使用该限制，与固件 `CONFIG_CJSON_NESTING_LIMIT=16` 一致；本轮未执行这些编译命令。

TODO（R2-16，留待后续批次）：为真实 `c3_link_rx.cpp` / `c3_map.cpp` 增加完整 host 回归，覆盖 500ms 心跳、持续坏流/读错误、变长重同步尾缀、重连代次、STOP 后回中与输入序号回绕。本轮未加入或运行 C3 完整回归。

The runner independently syntax-checks all four **production** translation units with `-Wall -Wextra -Werror`, then includes those same source files in the executable. It links the existing managed cJSON implementation, not a JSON imitation. Only the FreeRTOS, ESP initialization, socket, allocation-failure and UART safety boundaries are substituted. No ESP-IDF build, network service, radio, or real UART is started. Each case runs in a fresh process. Executables/compiler intermediates are confined to the active profile's scratch directory and removed on exit.

Individual cases can be passed as arguments, for example:

```bash
python tests/net_run_host_tests.py macro64 partial stop_running command_race json_oom
```

## Coverage

- ACK newline framing through the actual network task; fragmented RX of a 64-step macro through the actual parser; whole-line rejection beyond 16,384 bytes.
- 独立多客户端 TX、保留偏移/顺序的部分发送、非阻塞重试和慢客户端断开。积压上限为每客户端 8,192 字节；`NET_TX_TIMEOUT_MS=5000` 表示连续 5 秒没有发送进展才超时，有发送进展即续期，断开后锁 STOP。旧版 1 秒阈值不再适用。
- UDP fragments sent to active TCP peer IPv4 addresses at the existing port 8889, retaining the 8-byte big-endian header and 1,400-byte payload limit. No additional upper-computer handshake is required.
- Negative/high/wrapping goto targets, terminal macro identity, shared sensor snapshot preservation, wait conditions, STOP inside an ACT, pending physical STOP before START, and stale START rejection.
- Normal completion (including out-of-range goto) stays running while the captured generation's TX drain is false: no closing STOP, duplicate ACT, or elapsed-time fallback. Once drain succeeds, the macro reports DONE with its original identity and requests STOP. Pending completion remains immediately interruptible by external STOP, STOP/resume generation changes, abort, or replacement. A generation change after the drain check is reported as aborted; a final STOP_ALL ACT also retains emergency-abort semantics.
- A missing Xbox queue (service allocation failure) does not prevent sensor telemetry. The queue stub throws on NULL peek like FreeRTOS asserts, so graceful degradation cannot accidentally depend on a permissive stub.
- STOP/disconnect/abort protection even with full RX/control queues; stale received JSON is rejected by its captured generation. Parser commands and macro ACTs use `esp_link_send_command_for_generation`, including a STOP/resume preemption injected immediately before enqueue.
- Strict trailing-JSON rejection, 65-step rejection, integer/range/finite checks before wire-format casts.
- Queue creation failure, RX buffer/copy failure, TX allocation failure, program allocation failure, and exhaustive cJSON allocation-failure sweeps for complete telemetry (including macro/gamepad) and ACK construction. Partial JSON objects are not published and cJSON allocations are balanced.
- TCP listener creation failure exits safely rather than using an invalid descriptor in `FD_SET`.

## Red/green evidence（历史版本记录）

Tests were executed against the original production behavior before the corresponding fixes. Observed failing assertions included:

- `newline`: emitted ACK did not end in `\n`.
- `macro64`: RX produced multiple truncated queue entries.
- `oversized`: an oversized line produced executable prefixes.
- `partial`: clients received different/truncated byte streams.
- `slow`: a permanently unwritable client was never disconnected.
- `udp`: no destination-addressed UDP packets were produced.
- `goto_high`, `goto_negative`: invalid jumps did not terminate immediately.
- `progress`: terminal macro ID was lost after freeing the program.
- `snapshot`: telemetry removed the shared sensor snapshot.
- `json_oom`: allocation failure produced a partial JSON object.
- `stop_running`, `abort_full`, `stale_rx`, `goto_parse`: STOP did not terminate the macro, a full control queue lost abort, stale commands executed, and a large goto wrapped into range.
- `net_queue_oom`, `macro_queue_oom`, `invalid_numbers`, `invalid_macro_numbers`: failed creation or unsafe numeric narrowing was not rejected.

- `missing_xbox_queue`: with the strict NULL-peek stub, the unfixed telemetry failed with `configASSERT` (`0/1`); the null guard changed this to `1/1`.
- `completion_drain`, `goto_high_drain`, `goto_negative_drain`: the unfixed macro failed `p.state==MACRO_STATE_RUNNING` (`0/3`) because it reported DONE and requested STOP before TX drained. All three pass with completion pending; each holds drain false for 100 task cycles before allowing completion.

历史版本曾记录 `44/44 host regressions passed`、exit code 0；该数字不是当前测试数量或本轮运行结果。当前用例列表见 `net_host_test.cpp` 和 `net_run_host_tests.py`，新增/变更后的通过情况需另行运行验证。

## Runtime contracts and remaining limits

- `esp_net_rx_queue` 移交 `esp_net_rx_message_t` 的所有权：包含 `line`、`generation`、`slot`、`received_ms`、`line_bytes`；解析器在公共释放点归还字节预算并且只释放一次字符串。完整行总预算为 `PEO_NET_RX_BUDGET_BYTES`（64KiB），不含换行/字符串终止符。ACK/PING 进入 `esp_net_ack_queue`，遥测进入 `esp_net_tx_queue`，两者均为拥有所有权的 `char *`，网络任务每轮分别最多取 8 行、先 ACK 后遥测，广播后释放；ACK 满时仍非阻塞丢弃。宏 START 继续携带捕获的 generation。
- Abort and disconnect call the independent STOP latch directly. The macro task observes generation/inhibition, so queue saturation cannot hide STOP. UART transmission and its safety implementation are tested by the separate link suite, not this HAL harness.
- Normal completion uses `esp_link_commands_drained(generation)`, whose link-layer contract requires the generation still be current, every accepted command to have completed `uart_wait_tx_done`, and no pending STOP. This harness controls that boundary explicitly; it does not itself prove physical UART delivery.
- `app_main` 在创建任务前统一建队；本机不运行 BLE，手柄来自 C3 UART。`esp_net_task` 独立启动 WiFi AP，并通过 select 多路复用处理客户端，不等待 BLE 初始化事件。
- 遥测每轮延时 20ms 后 peek 最新快照，名义频率 50Hz；UART 停更时可重复旧样本，但缺失/接收过期会标记 `link_stale`，手柄心跳过期则省略 `gamepad`。JSON 构建期间 generation 改变时丢帧；这不代表不同传感器在同一时刻采样。
- A finite line limit is intentional: unusually whitespace-heavy JSON above 16 KiB is disconnected even if it has at most 64 steps. The 64-step protocol itself is unchanged.
- host 测试不能证明 WiFi/C3 UART/相机并发负载下的堆余量、实际 lwIP 时序、UART 帧交付、DMA/cache 行为或硬件停车时延，这些仍需上板验证。当前 S3 不运行 BLE；也不把 host 结果当作完整 IDF 构建通过。
- Epochs protect messages already received/queued and partial lines. Without changing the wire protocol, bytes still buffered in the network stack cannot be reliably distinguished from newly sent user intent. A disconnected socket is closed; existing peers remain supported.
