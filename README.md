# PEO Rover 上位机 / Upper Computer

> **一句话定位 / One-line role**：上位机是组员和车之间的「控制台」——能看车（视频）、看数据（遥测）、发命令（GUI 控制台），更重要的是能写 Python 脚本来自动化控制车（peo SDK）。
>
> The upper computer is the "console" between the team and the rover — you can see the rover (video), see its data (telemetry), send commands (GUI console), and above all write Python scripts to drive it automatically (peo SDK).

---

## 工程意义 / Why This Project Exists

**中文**：组员大多没有嵌入式背景，直接写 C 固件不现实。上位机把「控制一台车」这件事抽象成一组 Python 动作函数（`peo.drive()`、`peo.stop()`、`peo.cam.read()` …），组员像写单片机代码一样「动作 → 延时 → 动作」，实时通过 WiFi 下发命令。它同时提供一个桌面 GUI：看视频、看遥测仪表盘、手动急停，方便现场调试。

**English**: Most team members have no embedded background, so writing C firmware directly is unrealistic. The upper computer abstracts "driving a rover" into a set of Python action functions (`peo.drive()`, `peo.stop()`, `peo.cam.read()` …) — team members write scripts in a blocking style ("act → delay → act") and the commands stream out over WiFi in real time. It also provides a desktop GUI: video view, telemetry dashboard, and a manual emergency stop for field debugging.

## 功能与作用 / What It Does

| 功能 Function | 说明 Description |
|---|---|
| 桌面 GUI | 视频显示 + 遥测仪表盘 + 控制台 + 急停（空格键） |
| peo SDK | 组员写 Python 脚本调用 `peo.*` 动作函数实时下发命令 |
| 命令/遥测 | TCP 8888 收发 JSON 命令 + 50 Hz 遥测 |
| 视频接收 | TCP 8890 收 JPEG，FrameAssembler 拼帧解码 |
| PID 调参 | `peo.set_pid()` 下发 + 本地 config.json 缓存 |

## 与其它三端的联系 / Connections to the Other Three Nodes

```
        ┌──────────────────────────────────────┐
        │  上位机 Upper Computer (PC)        ★  │
        │  PySide6 GUI + peo SDK（组员脚本）    │
        └──────────────────┬───────────────────┘
                WiFi STA   │ TCP 8888 命令/遥测
                            │ TCP 8890 视频
        ┌──────────────────▼───────────────────┐
        │     ESP32-S3（WiFi 桥 Bridge）        │
        └──────────────────┬───────────────────┘
                            │ USART2
        ┌──────────────────▼───────────────────┐
        │  STM32F407（主控 Main Controller）    │
        └──────────────────┬───────────────────┘
                            │ USART3
        ┌──────────────────▼───────────────────┐
        │   ESP32-C3（手柄 BLE 桥 Gamepad）     │
        └──────────────────────────────────────┘
```

- **↔ ESP32-S3**：唯一出口——WiFi STA 连 S3 热点，TCP 8888 命令/遥测、TCP 8890 视频。
- **↔ STM32F407 / ESP32-C3**：无直接连接，一切经 S3 中转。

---

## 技术细节 / Technical Details

更新：2026-11-16。PySide6 桌面监控/组员 IDE，以及可独立安装的 `peo` Python SDK。上位机通过单个 TCP 8888 连接收发命令/遥测，通过 TCP 8890 接收 JPEG 视频（2026-11-16 起视频流完全走 TCP）；S3 负责桥接与停车屏障，F407 执行电机和传感器操作。

## 启动

需要 Python 3.10+。在本仓库根目录执行：

```powershell
python -m pip install -r requirements.txt
python -m app.main
```

`run.bat` 保留了本机 Python 3.13 的绝对路径；其他电脑使用上面的通用命令，或修改该路径。无需连接真车即可打开 GUI。连接真车前加入 `PEO_Rover` AP（密码 `12345678`），默认主机 `192.168.4.1`。

## 当前功能

- **Monitor**：连接、分辨率选择、视频开关、传感器/资源监控和 STOP；不含旧版直控、手柄或宏面板。
- **Team IDE**：Python 脚本编辑/运行/停止、持久 shell、最新视频和有界输出控制台。脚本与正在执行/续行中的 shell 互斥。
- 切换到 IDE 会取消正在建立的 Monitor 连接或断开现有连接。Monitor 与 SDK 共用本机控制锁；S3 同时只允许一个客户端。
- IDE Stop 设置协作取消标志并立即排入优先 STOP，不等待脚本主动调用 `delay`。脚本只有在 SDK 动作/等待检查到标志时才退出；纯 CPU 死循环无法被强制终止，但不能继续通过 SDK 下发普通命令。
- 正常结束、异常结束或关闭窗口会清理控制连接。空格急停按当前 tab 选择控制路径，文本编辑框聚焦时不触发。

## 通信与停车语义

- 每 500 ms 发送单向安全心跳 `{"type":"heartbeat"}`，仅续期控制看门狗；无 ACK、无时间戳，已取消 PING/RTT。
- `connect()` 只连接 TCP 8888；`VIDEO_ON` / `peo.cam.on()` 后才连接视频端口，`VIDEO_OFF` 关闭视频连接并停止重试。`video_port=None` 禁用此会话的视频功能。
- 状态栏两个 Tab 仅显示 CPU0/CPU1/RAM；来源为 `sensor.sys`，不再显示 Hz、fps、Loss、RTT。
- 普通命令须先获得遥测中的 `gen`。每次连接拥有独立的 socket、发送队列、停止事件和线程；旧连接不能向新连接发送积压命令。
- STOP 清除本机尚未发送的普通命令，并在有界队列满时仍可入队。一个已提交的 TCP 批次可能先发出（每批最多四条）；TCP 或硬件故障时不保证送达。
- `peo.stop()` 等待调用后收到的成功 S3 STOP ACK，超时重发一次；仍未确认时抛 `TimeoutError`。这表示 S3 接受请求，**不表示 F407 已执行，也不是硬件急停**。
- SDK 的 `peo.fresh` 是动态布尔值：已连接、最近 250 ms 收到遥测、且 S3 没有报告 `link_stale`。S3 遥测约 20 Hz；UART 数据年龄超过 500 ms 时标为 stale。断线清除快照。
- 心跳仅证明通信线程仍在运行，不能证明用户算法正在推进。算法必须自行检查数据/视频新鲜度；F407 仍需独立的命令过期停车和物理断电手段。

## 视频与性能

S3 目标采集节拍为 24 fps；当前支持 QQVGA/QVGA/VGA/SVGA。视频默认关闭，开启/切换包含传感器初始化与预热，首帧延迟不是固定 0.5 s。

视频走 TCP 8890，定界为 8 B 大端头（magic+fid+len）+ JPEG 体，单帧上限 512 KiB；TCP 保证有序可靠，无分包/乱序概念。帧先过 SOI/SOF/SOS/段长/EOI 结构守卫再交解码器。视频连接断开或流错位时接收端自动重连（2 s 退避），命令通道不受影响。

SDK 只保留最新完整帧，超过 1 s 不再作为可读新鲜图像；重复读取缓存解码结果并返回独立 ndarray。GUI 合并 Monitor 帧通知、约 30 Hz 轮询 IDE 最新帧，避免积压解码。输出队列最多 256 个 8192 字符块，每轮最多取 64 块；高频输出会丢弃旧块，显示文档最多保留 500 行。

JPEG 结构检查不等于完整解码成功；Qt/OpenCV 仍可能拒绝或容错解码损坏图像。图像质量需实机验收。

## SDK 安装与使用

安装入口已移到仓库根目录，wheel 同时包含 `peo` 和所依赖的 `rover_comm`：

```powershell
python -m pip install .
# 需要 cam.read() 的 BGR 图像时：
python -m pip install '.[vision]'
```

GUI 使用源码中的 `sdk/peo`。也可以运行 `python examples/blink.py`、`python examples/avoid.py` 或 `python examples/vision_line.py`；示例自行加入正确源码路径。涉及运动的示例需要先校准轮序/方向和速度。

完整 API 见 [sdk/API_REFERENCE.md](sdk/API_REFERENCE.md)，入门说明见 [sdk/README.md](sdk/README.md)，线协议见 [PROTOCOL.md](PROTOCOL.md)。`sdk/API_REFERENCE.pdf` 是旧版静态导出，未随本轮 Markdown 自动重建，不作为当前 API 依据。

## 测试

以下命令分别在独立进程执行，不连接真车：

```powershell
python tests/regression_test.py
python tests/sdk_test.py
python tests/smoke_test.py
```

回归测试使用模拟 socket；GUI 测试使用 offscreen Qt。SDK 测试通过 `PEO_CONFIG_PATH` 指向临时目录，不再覆盖或删除用户的 `~/.peo_rover_config.json`。测试不验证真实无线电、S3/F407 执行、急停时延或相机图像质量。

审查发现、修复范围及最新验证结果见 [REVIEW_AND_REPAIR.md](REVIEW_AND_REPAIR.md)。
