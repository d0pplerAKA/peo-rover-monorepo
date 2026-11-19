# PEO Rover — 上位机（PySide6 GUI + peo SDK）

> **一句话定位**：上位机是组员与车之间的「控制台」——能看车（视频）、看数据（遥测）、发命令（GUI），更重要的是能写 Python 脚本实时控制车（peo SDK）。

---

## 1. 工程总览

上位机（Python ≥3.10，PySide6）分三层：

- **app/**：PySide6 桌面 GUI——Tab1「Monitor」看视频/仪表盘/急停，Tab2「Team IDE」写脚本 + 控制台；
- **rover_comm/**：通信库（组员可直接 import）——命令编码、RoverClient（TCP 8888 命令/遥测 + TCP 8890 视频）、控制锁；
- **sdk/peo/**：组员 SDK（唯一入口 `peo` 包）——动作函数（drive/stop/set_pid…）+ 只读传感器命名空间 + 磁盘配置缓存。

连接模型：上位机以 WiFi STA 连 S3 热点，TCP 8888 收发命令/遥测、TCP 8890 收视频；心跳每 0.5s 单向发送；所有普通命令自动附当前遥测代次（gen）。

## 2. 工程意义

1. **组员没有嵌入式背景**：把「控制一台车」抽象成一组 Python 动作函数，组员像写单片机代码一样「动作 → 延时 → 动作」，实时通过 WiFi 下发。
2. **视觉导航的落点**：视频帧直接进 `peo.cam` 变成 ndarray（cv2），视觉算法（循线/避障）在 PC 上写，算力不受 MCU 限制。
3. **现场调试台**：GUI 的仪表盘（距离/速度/电流/姿态/CPU）、急停按钮、IDE 控制台让现场排障可视化。
4. **配置管理**：PID/分辨率缓存在 `~/.peo_rover_config.json`，重连、F407 复位后自动重发——调好的参数不会丢。

## 3. 功能与作用

| 功能 | 实现 |
|---|---|
| Monitor 页 | 连接、分辨率选择、视频开关、传感器仪表盘、急停按钮（Tab1 唯一命令按钮） |
| Team IDE 页 | 脚本编辑/运行/停止、持久 shell REPL、最新视频、统一控制台 |
| peo SDK | 动作函数 + 只读传感器命名空间 + `delay()` 阻塞节拍 + 单实例控制锁 |
| 视频接收 | TCP 8890，8B 头（0xCAFE+fid+len）+ JPEG，结构守卫 + 坏帧丢弃 |
| 心跳 | 每 0.5s 单向 `{"type":"heartbeat"}`（无 ACK、无时间戳、只续期看门狗） |
| 急停 | 按钮 + 空格快捷键；SDK `peo.stop()` 等 S3 新 ACK（0.3s×2 重试） |
| 配置缓存 | PID + 分辨率存 `~/.peo_rover_config.json`（PEO_CONFIG_PATH 可覆盖） |
| i18n | GUI 中英切换（默认英文，面向国际组员），SDK 报错消息随 GUI 语言 |

## 4. 与其它三端的联系

```
        ┌──────────────────────────────────┐
        │   上位机（PC，本工程）★           │
        │   PySide6 GUI + peo SDK 脚本      │
        └───────────────┬──────────────────┘
              WiFi STA  │ TCP 8888 命令/遥测
                         │ TCP 8890 视频
        ┌───────────────▼──────────────────┐
        │     ESP32-S3（WiFi 桥）           │
        └───────────────┬──────────────────┘
                         │ USART2
        ┌───────────────▼──────────────────┐
        │   STM32F407（主控执行器）          │
        └───────────────┬──────────────────┘
                         │ USART3
        ┌───────────────▼──────────────────┐
        │   ESP32-C3（手柄 BLE 桥）          │
        └──────────────────────────────────┘
```

- **↔ ESP32-S3**：唯一出口——8888 命令/遥测、8890 视频；心跳喂 S3 的看门狗。
- **↔ STM32F407 / ESP32-C3**：无直接连接，一切经 S3 中转；车上 `lock` 字段（手操/自主）在上位机只读显示（控制锁徽章）。

## 5. 主要实现

### 5.1 模块分层

| 层 | 文件 | 职责 |
|---|---|---|
| GUI | app/gui.py | MainWindow v2：Tab1 监控 + Tab2 IDE + 语言切换 + 急停 |
| GUI | app/video.py | VideoWidget 用 QImage 直解 JPEG（无 OpenCV 依赖）+ 30 帧 fps |
| GUI | app/i18n.py | zh/en 双语字典 + tr() |
| 通信 | rover_comm/protocol.py | 纯数据层：命令 ID/CRC16/帧打包/SensorData |
| 通信 | rover_comm/client.py | RoverClient：三线程（send/tcp/heartbeat）+ 视频线程 + gen 机制 |
| 通信 | rover_comm/control.py | ControlLock：127.0.0.1:8892 本地租约锁 |
| SDK | sdk/peo/__init__.py | 动作函数 + 只读命名空间 |
| SDK | sdk/peo/_bridge.py | 连接桥：RoverClient + 50Hz 快照 + 配置缓存 + 重发监控线程 |
| SDK | sdk/peo/cam.py | 单槽最新帧邮箱 + cv2 懒解码 |

### 5.2 peo SDK 动作清单

| 动作 | 语义 |
|---|---|
| `connect(host="peo_rover.local")` | 连 8888+8890+心跳+租约；自动重发缓存 PID/分辨率 |
| `disconnect()` | 断连并释放租约 |
| `delay(ms)` | 阻塞等待；停标志触发即抛 ScriptStopped |
| `led(on=None)` | True→LED_ON / False→LED_OFF / None→LED_TOGGLE |
| `drive_all(speed_cm_s)` | 四轮同速闭环（**单位 cm/s**，MOTOR_CLOSED） |
| `drive(*pairs)` | 变参 (motor, speed_cm_s)；只发指定电机（数据更新）；可为负（倒车） |
| `motor(*pairs)` | 开环 PWM **千分比 -1000..1000**（MOTOR）；与 drive 互斥切换开/闭环模式 |
| `set_pid(kp=…, ki=…)` | 四轮共享 PI；缺省合并缓存（kp 0.25 / ki 0.005）；发命令 + 写缓存 |
| `stop()` | 优先级 STOP + 等**新的** S3 ACK（0.3s×2），未确认抛 TimeoutError |
| `reset_odometer()` | 四轮里程清零 |
| `ws2812(r,g,b,brightness=100)` | 状态灯颜色（S3 本地命令） |
| `sys_reset()` | F407 软复位（S3 保持运行）；恢复后自动重发缓存配置 |

只读命名空间：`mpu.quat/accel`、`vl53.front_mm/left_mm/right_mm/*_ok`、`wheels.speed_cm_s/odometer/current_mA/bus_voltage_mV`、`sys.cpu0/cpu1/ram`、`cam`、顶层 `fresh/control_mode/fw_version/s3_fw/lock`（无数据返回 None）。

### 5.3 连接与安全机制

- **代次（gen）**：普通命令自动附最新遥测代次；STOP_ALL 单独发送（清队列 + 代次置失效 + ready 清除）；收到遥测仅当代次合法且未被 STOP 屏蔽才更新。
- **心跳**：0.5s 单向，只续期 S3 看门狗（S3 超 2s 自动急停）——**心跳不证明算法在推进**，算法必须自行检查 `fresh`/视频新鲜度。
- **fresh 语义**：已连接 + 有快照 + 无 link_stale + 距最新快照 <250ms（`FRESH_MS=250`）。
- **控制锁**：`ControlLock`（127.0.0.1:8892）本机互斥——GUI 与 SDK 脚本同一时刻只有一方能控制；GUI 切 Tab2 自动断开释放。
- **配置重发监控**：后台线程每 0.1s 查 fresh 恢复沿（连接后/F407 复位后/link_stale 恢复），自动重发缓存 PID + CAM_RES。

### 5.4 视频接收

8B 大端头（magic 0xCAFE + frame_id + JPEG 长度）+ JPEG 体；拼帧在 `RoverClient._video_loop`（精确读头再精确读体）；magic/长度非法（1..512KB 之外）即断流重连（2s 退避）；解码前过 `jpeg_structure_ok`（SOI/SOF/SOS/EOI/宽高 ≤5M 像素）结构守卫。GUI Tab1 邮箱取最新帧、Tab2 经 `peo.cam`（单槽最新帧 + 1s 新鲜窗口 + cv2 懒解码）。

## 6. 功能的影响与制约

- **两种速度单位不能混**：`drive/drive_all` 是 **cm/s**（闭环，MOTOR_CLOSED）；`motor` 是 **千分比 ±1000**（开环，MOTOR）。发 `motor` 会把 F407 切到开环模式，发 `drive` 切回闭环——两者语义互斥。
- **STOP 的 ACK 只是「S3 已接收」**：协议无请求 ID，`peo.stop()` 等到的是 S3 的确认，不是 F407 已执行；GUI 急停按钮/空格不等 ACK（立即发）。真正急停是 S3 锁 STOP + F407 独立 deadman 的合力。
- **心跳不续命算法**：纯 CPU 死循环的脚本无法被强制终止（IDE Stop 是协作式取消标志），只能靠心跳保住 S3 不误判，但车若在跑必须靠 S3 心跳超时或物理断电。
- **单客户端硬约束**：本机 ControlLock + S3 `PEO_NET_MAX_CLIENTS=1` 双重互斥；想多脚本并发会被锁拒绝（项目明确禁止多脚本并发）。
- **视频新鲜窗口 1s**：超过 1s 的旧帧 `cam.read()` 不再返回（`_FRAME_FRESH_MS=1000`）；依赖视觉的算法必须检查 `cam.fresh`。
- **配置缓存写失败静默吞掉**：`_save_config` 失败只记日志不抛异常——组员脚本不会因磁盘问题中断，但参数可能没持久化。
- **遥测节拍**：F407 上行 50Hz，S3 遥测 JSON 自主态 20Hz/手操态 2Hz——`fresh`（250ms 窗口）在 2Hz 下依然成立，但手操态传感器数据有效刷新率只有 2Hz。

## 7. 技术栈与运行

- Python ≥3.10 + PySide6 + numpy + opencv-python（可选组 `[vision]`）。
- 运行：`python -m pip install -r requirements.txt` → `python -m app.main`（`run.bat` 保留了本机绝对路径）。
- 测试：`python tests/regression_test.py`（离线回归 32 项）、`python tests/sdk_test.py`、`python tests/smoke_test.py`。

## 8. 版权

Copyright (c) 2026 Danny — MIT License（见 `LICENSE`）。
