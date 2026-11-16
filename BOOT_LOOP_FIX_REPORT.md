# ESP32-S3 N8R8 循环重启修复与验证报告

## 0. 第二轮：相机 PSRAM DMA 缓存行越界（2026-11-14 晚）

BLE 修复已生效：新日志显示 BLE 初始化完成、8 MB PSRAM 内存测试通过、AP 启动。随后在相机启动后发生新的 `LoadProhibited`，调用链为 `esp_vfs_select -> free -> tlsf_free`。这属于**第二个独立缺陷**，不是原 BLE 崩溃复发。

### 已确认的缺陷

- 数据缓存行 = **64 B**，但相机帧缓冲只保证 **16 B 对齐**，随后又手工偏移 16 B。
- 日志 `Frame[1]: Addr 0x3C1170B0`，原始分配起点为 `0x3C1170A0`。
- 读取帧头 32 B 时，`cam_drop_psram_cache()` 向两端扩到 64 B 行，失效范围变成 `[0x3C117080, 0x3C117100)`；其中 `[0x3C117080, 0x3C1170A0)` 不属于该帧缓冲。
- 崩溃寄存器 `A3 = 0x3C117084` 正落在这段越界失效范围内。M2C invalidate 会丢弃该行尚未写回的堆元数据，之后的 `free()` 读取旧元数据并崩溃。
- 这是**确定的越界缓存失效与破坏机制**；仍需实机运行确认它是本次崩溃的唯一触发条件。

### 修复

- 帧缓冲对齐取 `max(GDMA 对齐, 64 B 缓存行)`，分配长度按该值向上取整，不再二次偏移，`free()` 直接使用分配指针。
- 每帧新增 `fb_dma_sync_size`（独占、已对齐的完整分配长度）；DMA 启动前在 GDMA 复位之后、`link.start` 之前对其做 `C2M | INVALIDATE`，并检查返回值，失败则不启动该帧。
- 增加帧长度上限检查，避免 VSYNC 紧邻最后一个 EOF 时把未覆盖的描述符块当作 JPEG 尾部。
- 独立复核又确认同一路径的两处缺陷并已修复：YUV→灰度原地转换前先失效 DMA 数据、转换后用 C2M 写回（不能反过来丢弃 CPU 写入）；缓存同步只把 `ESP_ERR_NOT_SUPPORTED`（非缓存内存，如 FB_IN_DRAM）视为可继续，其余错误仍中止该帧。
- DMA 起点同步改为 M2C 失效：DMA 即将覆盖整段，无需把旧缓存写回，减少每帧 16 KB writeback 的中断开销。
- 新增 `tests/camera_dma_alignment_host.py`：编译真实 `cam_dma_config`/缓存函数，覆盖 32/64 B 缓存行、16/32/64 B DMA 对齐、非对齐长度共 18 种组合；并校验上述崩溃地址位于修复前越界范围。

### 第二轮产物

- 最终 ELF SHA256：`fdddc275aafc7b8a3f4110d97f4a6b0e6ed840a24da4e60813d0fc2ab01c7244`
- 最终 BIN：1195504 字节，仍需烧录验证。
- 全部 host 回归：UART/宏 8 组、Xbox 4 套、网络/宏/遥测 44/44、相机缓存 18 组，全部通过。

**识别新固件必须看 ELF SHA 前缀 `fdddc275`**；增量构建的 app 描述符 Compile time 可能仍是旧值。

---

## 1. 结论与验证边界

已在授权工程内修复明确的 Arduino4/NimBLE BLE 内存生命周期兼容缺陷，并修复 UART、手柄映射、网络分帧和宏/停止处理中的可复现问题。已完成真实 ESP-IDF v6.1 编译、生产 C++ host 回归、最终 ELF 与 BIN 校验。

**尚未烧录修复版，也没有完成板上持续运行验证。** 不能据此宣称现场所有重启因素均已排除；无线射频、PSRAM 实物、相机和电机行为需按本文末尾清单验收。

- 工程：`C:\Users\Danny\Documents\Espressif Files\peo_rover_esp32s3`
- 当前硬件口径：用户确认的 ESP32-S3 N8R8。
- 仅修改本 ESP32 工程及当前 danny-v1 的相关记录/技能；未改 STM32、上位机、全局 SDK 或 managed_components 源码。
- 没有提交、推送、切换分支、改写用户 Git 索引、擦除设备或修改 eFuse。

## 2. 崩溃证据与兼容性缺陷

用户日志的 ELF SHA 前缀为 `5aa251367`；磁盘原始 ELF 的完整 SHA256 为：

`5aa251367c6cee3992ef67468877b53bdac0ad9a06df108278fcba0ee77cc87b`

两者匹配，以下分析针对实际崩溃版本，而非重编译后的地址。

日志显示 Flash/DIO 启动正常，进入应用后发生：

```text
Core 0 LoadProhibited
vQueueDelete -> semphr_delete_wrapper
-> btdm_controller_deinit_internal -> btdm_controller_init
-> esp_bt_controller_init -> nimble_port_init
-> NimBLEDevice::init -> Core::begin -> xbox_ble_task
无效句柄 A2 = 0x02047d00；EXCVADDR = 0x02047d44
```

已确认的源码/链接证据：

1. Arduino 4.0.0-rc1 的 `esp32-hal-bt.c` 默认 `_bleLibraryInUse=false`，弱 `bleInUse()` 返回该标志。
2. `initArduino()` 在该标志为 false 时执行 `btMemRelease(BT_MODE_BLE)`。ESP-IDF 不允许释放后再次使用该蓝牙模式。
3. esp-nimble-cpp 2.6.0 只检查旧头文件 `esp32-hal-bt-mem.h`；当前 Arduino 正式保留接口是 `esp32-hal-alloc-ble-mem.h`。
4. 原 ELF 缺少 `_setBleLibraryInUse` 构造函数。仅定义 `ARDUINO_ARCH_ESP32` 或调用 `btStarted()` 不会替代该保留机制。
5. 已在 `main.cpp` 显式包含正式头文件。最终 ELF 验证不仅发现构造函数符号，还确认其地址位于初始化数组中，确实会在 `app_main()` 前运行。

因此，“BLE 内存被归还堆、队列/任务分配后再初始化控制器”是与现场异常高度吻合、且已有直接源码和 ELF 证据的缺陷。**不是通过关看门狗或随意绑核掩盖崩溃。其作为本次现场唯一根因的最终确认仍需新固件上板。**

## 3. N8R8 配置

| 项目 | 最终设置 |
|---|---|
| IDF target | `esp32s3` |
| Flash | 8 MB，DIO，80 MHz |
| PSRAM | 8 MB 型号口径，Octal，80 MHz，启动初始化与内存测试 |
| PSRAM 缺失 | 记录错误/容量异常；相机退出，不强行分配 PSRAM 帧缓冲 |
| Flash Octal/OPI | 不启用；N8R8 的 Flash 与 PSRAM 类型不能混淆 |
| 分区 | 保留原 NVS、PHY、factory 偏移，不做数据迁移 |
| UART | 保留 GPIO11/12、921600 baud |
| 相机接线 | 保留用户原定义；GPIO35/36/37 不用于应用外设 |
| cJSON 深度 | 从 1000 限制到 16，满足当前协议结构 |

源 `sdkconfig`、`sdkconfig.defaults` 和生成的 `build/config/sdkconfig.json` 已核对。官方型号参考：[Espressif ESP32-S3 开发板文档](https://documentation.espressif.com/espressif-esp-dev-kits/en/latest/esp32s3/esp-dev-kits-en-master-esp32s3.pdf)，N8R8 表项为 8 MB QD Flash + 8 MB OT PSRAM。

## 4. 主要修复

| 范围 | 修复内容 |
|---|---|
| 启动 | 正式 BLE 内存保留；启动内存诊断；先建队列、检查创建结果；BLE 成功/失败/超时降级；重启先锁存 STOP |
| UART RX | 分离 112B 遥测与 64B 命令上限；拒绝零长；修正 CRC 帧头；有界滑动重同步，处理丢字节与数据内嵌帧头 |
| UART TX / STOP | 独立锁存 STOP，不受普通队列满影响；旧代命令失效；发送失败保留停机请求；等待物理 TX 完成后留空闲；生产者捕获代次避免跨 STOP/恢复复活 |
| Xbox | uint16 摇杆先在 int32 中减中值，避免中心变满速；短/null HID、电池报告校验；断连清状态；一致快照；输入新鲜度与任务心跳；重新回中后才允许驱动 |
| TCP/UDP | JSON 换行；16 KiB 有界行支持完整64步宏；部分发送与慢客户端处理；超长/NUL/OOM/RX满安全断开；UDP 用 sendto 发往 TCP 客户端 IP 的8889端口 |
| 宏 | goto 范围/窄化检查；终态保留 ID；STOP 中止旧宏；正常完成必须等待本代已接受命令真正发送完成，再收尾 STOP，避免丢末条 ACT |
| 遥测 | 使用非破坏快照读取；空 Xbox 队列降级；JSON 分配失败处理 |

停止后的恢复行为详见英文 `README.md`。这些措施不是硬件急停；线上已在发送的一帧可能先于 STOP 完成。

## 5. 实际验证结果

### 5.1 Host 测试

测试直接编译生产 C++ 源码，使用小型 FreeRTOS/UART/socket/BLE 适配器；网络测试链接真实 cJSON。不是仅写了测试，也不是用虚构串口数据冒充硬件实测。

- `python tests/run_link_tests.py`：8 组输出全部 PASS。包含所有255种非法长度、坏CRC、丢CRC字节＋内嵌帧头、STOP重试/代次、发送完成计数，以及真实宏引擎＋真实UART队列的延迟TX集成测试。
- `python tests/xbox_run_host.py`：parser/controller/service/map 四套全部 PASS，包含50,000次并发通知快照检查。
- `python tests/net_run_host_tests.py`：**44/44 host regressions passed**，四个生产翻译单元独立 `-Wall -Wextra -Werror` 检查通过。
- 两轮独立审查发现的持续失步、空队列降级、宏末条命令丢失均补入失败用例，再完成修复并重跑。

### 5.2 ESP-IDF 构建

- `.\build.ps1`：最终退出码 **0**；实际完成编译、链接、BIN生成与分区容量检查。
- 最终增量构建：**0 compiler errors / 0 compiler warnings**。
- 前一轮完整构建有35条 SDK/第三方源码编译警告，本项目自写组件没有警告；未为追求“零警告”擅改全局 SDK 或第三方下载源码。
- `tests/verify_firmware.py`：BLE保留构造函数及初始化数组、有效 N8R8配置全部 PASS。
- `esptool image-info`：ESP32-S3、8MB Flash、DIO、80MHz；Checksum valid、Validation hash valid。

### 5.3 最终产物

| 文件 | 大小（字节） | 烧录偏移 |
|---|---:|---|
| `build/bootloader/bootloader.bin` | 22624 | `0x0` |
| `build/partition_table/partition-table.bin` | 3072 | `0x8000` |
| `build/peo_rover_esp32s3.bin` | 1194688 | `0x10000` |
| `build/peo_rover_esp32s3.elf` | 17166840 | 不直接烧录；用于解码/调试 |

应用分区余量：`0x53540` 字节，构建器报告约22%。

最终 ELF SHA256：
`4bbda267f2c65c76d599f1be0839409a9daa66fd16aa2f9fc194f3639d1f87a4`

最终应用 BIN 文件 SHA256：
`1e917361a9bcb2b4f16ec55c08c39964af4466a774fa3953507ff2adb38fd726`

注意：增量构建可能保留 app descriptor 的原 Compile time。识别这次固件请核对 ELF SHA 前缀 **`4bbda267`**，不要只看 `f68058c-dirty` 或时间字符串。

## 6. 烧录与板上验收（未执行）

先断开电机动力电源或架空车轮，关闭占用串口的监视器。以下 COM11 应以实际 ESP32 端口为准；此前检测为 CP210x，曾被监视器占用。

```powershell
Set-Location 'C:\Users\Danny\Documents\Espressif Files\peo_rover_esp32s3'
& 'C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1'
idf.py -p COM11 flash monitor
```

**请烧录第二轮固件，启动日志的 ELF SHA 前缀必须是 `fdddc275`。** 仅烧应用 BIN 不够，仍需 bootloader + 分区表 + 应用三件套。不需要全片擦除或 eFuse 修改。

第二轮重点观察：

- [ ] 启动显示 `PSRAM=8388608 bytes`、`SPI SRAM memory test OK`。
- [ ] 相机日志出现 `Frame[0]/Frame[1]: Offset: 0`，两个地址均为 64 B 对齐（低 6 位为 0）。
- [ ] 相机启动后持续运行数分钟，不再在 `tlsf_free`/`esp_vfs_select` 崩溃。
- [ ] 反复冷启动、开关 WiFi 客户端、开关手柄，观察是否仍重启。

若仍崩溃，请把完整日志（含 ELF SHA）发回；下一步应打开 `CONFIG_HEAP_POISONING_COMPREHENSIVE` 与 `CONFIG_CAMERA_PSRAM_DMA=n` 做对照，而不是继续盲目改对齐。

## 7. 剩余风险

1. **ESP32掉电、崩溃或UART断线时无法发送STOP**：STM32必须有独立命令超时停车，且需物理电源截止。另一个工程不在本轮修改范围。
2. **无完整多控制源所有权仲裁**：同时移动手柄和运行上位机/宏仍可能互相覆盖；验收时仅一个主动驾驶来源。
3. 已在网络栈中但尚未读取的旧TCP字节，没有端到端序号可判断代次；已入队和已开始接收的行有本地代次保护。若需要强安全会话协议，需同步修改两端。
4. 控制器HID输入新鲜度默认5s、任务心跳1s，需实测实际通知节奏；不能把此软件阈值当硬件安全认证。
5. AP密码保留原值 `12345678`，便于现有上位机联调；只在受控测试环境使用，实际部署前同步修改两端配置。
6. Arduino4仍为RC版本；本轮未升级依赖，也没有把依赖库的所有潜在问题宣布为已审计清零。

## 8. 回退与清理

- 原源码/未提交内容保存在 Git tree：`refs/repair-baselines/n8r8-before`。
- 崩溃 ELF 保存在 Git blob：`refs/repair-baselines/n8r8-crash-elf`。
- 这些是特意保留的本地恢复点，不是新提交，不改当前分支/用户索引。不要盲目 reset 用户原工作区；按文件检查后恢复。
- 测试运行器会自动清理可执行文件；任务临时构建/串口日志、临时索引及检查文件在收尾清理。正式源码、测试、README、本报告和build固件保留。
