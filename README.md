# PEO Rover 上位机

更新：2026-11-16。PySide6 桌面监控/组员 IDE，以及可独立安装的 `peo` Python SDK。上位机通过单个 TCP 8888 连接收发命令/遥测，通过 UDP 8889 接收 JPEG 视频；S3 负责桥接与停车屏障，F407 执行电机和传感器操作。

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

- 自动每 500 ms 发送 `{"cmd":"PING","t":...}`；S3 当前固件不处理旧的 `{"type":"heartbeat"}`。
- 普通命令须先获得遥测中的 `gen`。每次连接拥有独立的 socket、发送队列、停止事件和线程；旧连接不能向新连接发送积压命令。
- STOP 清除本机尚未发送的普通命令，并在有界队列满时仍可入队。一个已提交的 TCP 批次可能先发出（每批最多四条）；TCP 或硬件故障时不保证送达。
- `peo.stop()` 等待调用后收到的成功 S3 STOP ACK，超时重发一次；仍未确认时抛 `TimeoutError`。这表示 S3 接受请求，**不表示 F407 已执行，也不是硬件急停**。
- SDK 的 `peo.fresh` 是动态布尔值：已连接、最近 250 ms 收到遥测、且 S3 没有报告 `link_stale`。S3 遥测约 25 Hz；UART 数据年龄超过 500 ms 时标为 stale。断线清除快照。
- 心跳仅证明通信线程仍在运行，不能证明用户算法正在推进。算法必须自行检查数据/视频新鲜度；F407 仍需独立的命令过期停车和物理断电手段。

## 视频与性能

S3 目标采集节拍为 24 fps；当前支持 QQVGA/QVGA/VGA/SVGA。视频默认关闭，开启/切换包含传感器初始化与预热，首帧延迟不是固定 0.5 s。

接收端限定来源为已连接 S3 的 IP 和 UDP 8889，校验 8 B 大端头、每包不超过 1400 B、总帧不超过 512 KiB。分片数必须在同帧内一致；迟到旧帧首包、冲突重复包、超时和坏 JPEG 结构均被丢弃。包 0 之前到达的分片不缓存，包 0 之后的分片可乱序。

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
