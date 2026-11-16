"""
rover_comm.client — 网络客户端（TCP 8888 命令/传感器流 + UDP 8889 视频）

设计：纯 Python（threading + 回调），不依赖 Qt。
- GUI 层把回调桥接成 Qt signal（跨线程 emit 自动 queued）。
- 组员写算法时也可直接用本类（不用 GUI）。

用法：
    c = RoverClient()
    c.on_sensor = lambda sd: print(sd.dist_front_mm)
    c.on_frame  = lambda fid, jpeg: save(fid, jpeg)
    c.connect("192.168.4.1")
    c.stop_all()
    ...
    c.close()
"""
import json
import queue
import socket
import threading
import time

from .protocol import (
    TCP_CMD_PORT, UDP_VIDEO_PORT, VIDEO_MAGIC,
    SensorData, GamepadState, VideoHeader,
)

# 心跳间隔（s）：上位机保活，ESP32 端超时 2s 自动 STOP_ALL（PROTOCOL.md §2.4）
HEARTBEAT_INTERVAL_S = 0.5


# =============================================================================
# UDP 视频帧组装器（按 frame_id 分组、packet_id 排序；丢包 = 整帧丢弃）
# =============================================================================
class FrameAssembler:
    """按 frame_id 组装 UDP 分包（8B 头 + 1400B 分片）。

    关键规则（防花屏，宁可丢帧不可拼错帧）：
    - packet_id==0 且 fid 变化 → 新帧开始，重置缓冲；
    - fid 与当前帧不同的迟到包（802.11 重传/乱序）→ 直接丢弃，绝不混入当前帧；
    - 所有分包到齐（len==total，无洞）才 emit；
    - 2s 未凑齐 → 残帧丢弃，防陈旧分片堆积；
    - emit 前 JPEG 结构守卫（SOI 开头 + EOI 结尾），坏帧丢弃不上报。
    诊断统计：packets_total/late/bad、frames_ok/aborted/bad_jpeg——暴露给 GUI 状态栏定位丢包源。
    """

    def __init__(self):
        self._frame_id = -1
        self._packets = {}
        self._frame_start = 0.0
        self.on_bad_frame = None      # (jpeg:bytes, frame_id:int) 组装层坏帧取证钩子
        # ---- 诊断统计（只增不减，GUI 定期读取）----
        self.packets_total = 0        # 收到的视频包总数
        self.packets_late = 0         # 迟到包（旧帧，丢弃）
        self.packets_bad = 0          # 非法包头（坏 magic / packet_id 越界）
        self.frames_ok = 0            # 拼齐且合法的帧
        self.frames_aborted = 0       # 超时残帧（丢包导致）
        self.frames_bad_jpeg = 0      # 拼齐但 JPEG 结构非法
        self.last_jpeg_len = 0        # 最近一帧合法 JPEG 字节数

    def push(self, hdr: VideoHeader, data: bytes) -> bytes | None:
        """收到一个 UDP 包，若凑齐整帧则返回完整 JPEG，否则 None。"""
        self.packets_total += 1
        if hdr.magic != VIDEO_MAGIC or hdr.total_pkt <= 0:
            self.packets_bad += 1
            return None
        if hdr.packet_id >= hdr.total_pkt:
            self.packets_bad += 1    # 垃圾包头守卫（packet_id 越界直接丢）
            return None
        now = time.monotonic()
        if hdr.frame_id != self._frame_id:
            if hdr.packet_id != 0:
                self.packets_late += 1
                return None   # 旧帧的迟到包：丢弃，避免污染当前帧
            self._frame_id = hdr.frame_id
            self._packets = {}
            self._frame_start = now
        elif now - self._frame_start > 2.0:
            self._packets = {}      # 残帧超时：放弃（下一帧 packet_id==0 会重开）
            self.frames_aborted += 1
            return None
        self._packets[hdr.packet_id] = data
        if len(self._packets) == hdr.total_pkt:
            jpeg = b''.join(self._packets[i] for i in range(hdr.total_pkt))
            self._packets = {}
            # JPEG 结构守卫：SOI 开头 + EOI 结尾，挡住截断/错位帧，
            # 避免 Qt 解码报错刷屏（"missing SOS"/"premature end" 类）。
            if len(jpeg) < 4 or jpeg[:2] != b'\xff\xd8' or jpeg[-2:] != b'\xff\xd9':
                self.frames_bad_jpeg += 1
                if self.on_bad_frame:
                    self.on_bad_frame(jpeg, hdr.frame_id)   # 取证：拼齐但结构坏
                return None
            self.frames_ok += 1
            self.last_jpeg_len = len(jpeg)
            return jpeg
        return None


def _sensor_from_json(j: dict) -> SensorData:
    """§2.1 传感器 JSON → SensorData（字段缺失取 0）。"""
    dist = j.get("dist", {})
    imu = j.get("imu", {})
    wheels = j.get("wheels", {})
    power = j.get("power", {})
    ack = j.get("cmd_ack", {})
    quat = imu.get("quat") or [0.0, 0.0, 0.0, 0.0]
    accel = imu.get("accel_g") or [0.0, 0.0, 0.0]
    speed = wheels.get("speed_d100") or [0, 0, 0, 0]
    odo = wheels.get("odometer") or [0, 0, 0, 0]
    cur = power.get("current_mA") or [0, 0, 0, 0]
    volt = power.get("bus_voltage_mV") or [0, 0, 0, 0]

    def _d(field: str):
        return dist.get(field, {}).get("mm", 0)

    def _s(field: str):
        return 0 if dist.get(field, {}).get("ok", True) else 1

    return SensorData(
        dist_front_mm=_d("front"), dist_left_mm=_d("left"), dist_right_mm=_d("right"),
        dist_front_status=_s("front"), dist_left_status=_s("left"), dist_right_status=_s("right"),
        quat_w=quat[0], quat_x=quat[1], quat_y=quat[2], quat_z=quat[3],
        accel_x=accel[0], accel_y=accel[1], accel_z=accel[2],
        speed_d100=tuple(speed), odometer=tuple(odo),
        current_mA=tuple(cur), bus_voltage_mV=tuple(volt),
        sys_tick_ms=j.get("tick", 0),
        last_cmd_id=ack.get("id", 0), last_cmd_status=ack.get("status", 0),
    )


# =============================================================================
# RoverClient
# =============================================================================
class RoverClient:
    """TCP 命令/传感器流 + UDP 视频接收。"""

    def __init__(self):
        self._tcp: socket.socket | None = None
        self._udp: socket.socket | None = None
        self._send_queue: queue.Queue | None = queue.Queue(maxsize=64)  # 主线程只入队，发送线程执行阻塞 sendall
        self._sender_thread: threading.Thread | None = None
        self._running = False
        self._tcp_thread: threading.Thread | None = None
        self._udp_thread: threading.Thread | None = None
        self._hb_thread: threading.Thread | None = None
        self._video_asm: FrameAssembler | None = None   # 视频组装器（诊断统计，_udp_loop 填充）
        # v1.5 会话代次（PROTOCOL.md §2.1）：随 50Hz sensor JSON 的 "gen" 更新；
        # 命令/宏发送时自动附带，STOP 前积压的旧命令会因旧代次被 ESP32 拒绝（status=2）。
        self._observed_gen: int | None = None
        self.sys_info: dict | None = None
        self._ping_t_ms: int | None = None
        self.last_rtt_ms: int | None = None

        # 回调（用户赋值；在接收线程里调用，勿做重活）
        self.on_sensor: callable | None = None          # (SensorData)
        self.on_frame: callable | None = None           # (frame_id:int, jpeg:bytes)
        self.on_macro_status: callable | None = None    # (dict)
        self.on_gamepad: callable | None = None         # (GamepadState)
        self.on_ack: callable | None = None             # (dict: {"ack":cmd,"status":n})
        self.on_bad_frame: callable | None = None       # (jpeg:bytes, frame_id:int) 组装层坏帧取证
        self.on_disconnected: callable | None = None    # ()

    # ---------------- 连接管理 ----------------
    @property
    def connected(self) -> bool:
        return self._tcp is not None and self._running

    def connect(self, host: str, tcp_port: int = TCP_CMD_PORT,
                udp_port: int = UDP_VIDEO_PORT) -> bool:
        """连 TCP + 绑 UDP，启动两个接收线程。返回是否成功。"""
        self.close()
        try:
            self._tcp = socket.create_connection((host, tcp_port), timeout=5)
            # create_connection 会把 timeout=5 残留在套接字上：recv 5s 无数据即抛
            # socket.timeout（OSError 子类），被 _tcp_loop 误判为断连而关闭连接——
            # 遥测静默期（F407 未接/无传感器帧）必触发。改回阻塞模式：
            # 断链由 recv 的 EOF/错误 + 心跳 send 失败双重检测。
            self._tcp.settimeout(None)
            self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._udp.bind(("0.0.0.0", udp_port))
            # 加大收包缓冲：24fps 突发下 GUI 主线程短暂繁忙（解码/重绘）时，
            # 内核缓冲区过小会直接丢包 → 帧不完整；256KB ≈ 20 帧余量
            self._udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 256 * 1024)
            self._udp.settimeout(0.5)
        except OSError as e:
            self.close()
            raise ConnectionError(f"连接失败: {e}") from e

        self._running = True
        self._observed_gen = None   # 重连后不沿用旧代次：首条 sensor JSON 会在 20ms 内带来新值
        self._ping_t_ms = None      # 新会话：清空未完成的 PING 登记与旧会话监控状态
        self.last_rtt_ms = None
        self.sys_info = None
        self._tcp_thread = threading.Thread(target=self._tcp_loop, daemon=True,
                                            name="rover-tcp")
        self._udp_thread = threading.Thread(target=self._udp_loop, daemon=True,
                                            name="rover-udp")
        self._hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True,
                                           name="rover-heartbeat")
        self._sender_thread = threading.Thread(target=self._sender_loop, daemon=True,
                                               name="rover-send")
        self._tcp_thread.start()
        self._udp_thread.start()
        self._hb_thread.start()
        self._sender_thread.start()
        return True

    def close(self):
        if self._tcp is None and self._udp is None and not self._running:
            return
        self._running = False
        for s in (self._tcp, self._udp):
            if s is not None:
                try:
                    s.shutdown(socket.SHUT_RDWR)   # 先 shutdown：可靠唤醒其它线程的阻塞 recv/send，防卡死
                except OSError:
                    pass
                try:
                    s.close()
                except OSError:
                    pass
        self._tcp = self._udp = None
        if self.on_disconnected:
            self.on_disconnected()

    # ---------------- 发送 ----------------
    def send_json(self, obj) -> bool:
        """发一个 JSON 对象（自动换行结尾）。

        v1.5 起自动附加会话代次：命令/宏消息在已知当前代次时带上
        `gen`（PROTOCOL.md §2.1）——STOP 前进入 TCP 缓冲的旧命令因此带旧
        代次，会被 ESP32 以 status=2 拒绝，不会在 STOP 后解除急停。
        心跳（type=heartbeat）不附加。
        """
        obj = dict(obj)   # 浅拷贝，不把 gen 写回调用方的对象
        t = obj.get("type")
        if ("cmd" in obj or t == "macro") and "gen" not in obj and self._observed_gen is not None:
            obj["gen"] = self._observed_gen
        line = json.dumps(obj, separators=(",", ":")) + "\n"
        return self._send_raw(line.encode("utf-8"))

    def send_macro(self, macro) -> bool:
        """发宏脚本（Macro 实例或 dict）。"""
        obj = macro.to_dict() if hasattr(macro, "to_dict") else macro
        return self.send_json(obj)

    def stop_all(self) -> bool:
        return self.send_json({"cmd": "STOP_ALL"})

    def ping(self) -> bool:
        t = int(time.monotonic() * 1000)   # 单调时钟：RTT 不受系统调时影响
        self._ping_t_ms = t                # 先登记后发送，ACK 用 t 精确匹配
        if not self.send_json({"cmd": "PING", "t": t}):
            self._ping_t_ms = None         # 入队失败撤销登记
            return False
        return True

    def video_on(self) -> bool:
        return self.send_json({"cmd": "VIDEO_ON"})

    def video_off(self) -> bool:
        return self.send_json({"cmd": "VIDEO_OFF"})

    def reset_odometer(self) -> bool:
        return self.send_json({"cmd": "ODOMETER_RESET"})

    def _send_raw(self, data: bytes) -> bool:
        """非阻塞入队：实际 sendall 在专用发送线程执行，绝不卡调用方（GUI 主线程）。"""
        if not self.connected:
            return False
        try:
            self._send_queue.put_nowait(data)
            return True
        except queue.Full:
            return False

    def _sender_loop(self):
        """单发送线程：这里允许阻塞 sendall（链路拥塞时也只阻塞本线程）。"""
        while self._running:
            try:
                data = self._send_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            tcp = self._tcp
            if tcp is None:
                break
            try:
                tcp.sendall(data)
            except OSError:
                break   # 对端已死：交给 close() 唤醒所有线程并通知断连
        if self._running:
            self.close()

    # ---------------- 接收线程 ----------------
    def _tcp_loop(self):
        """按行收 JSON：sensor → on_sensor；macro 状态 → on_macro_status。"""
        buf = b""
        while self._running:
            try:
                chunk = self._tcp.recv(4096)
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except ValueError:
                    continue
                self._dispatch(obj)
        self.close()

    def _dispatch(self, obj: dict):
        # 命令 ACK（ESP32 回执，无 type 字段）
        if "ack" in obj and "type" not in obj:
            if (obj.get("ack") == "PING" and self._ping_t_ms is not None
                    and obj.get("t") == self._ping_t_ms):   # t 匹配：拒绝迟到/串台的 PING ACK
                self.last_rtt_ms = int(time.monotonic() * 1000) - self._ping_t_ms
                self._ping_t_ms = None
            if self.on_ack:
                self.on_ack(obj)
            return
        t = obj.get("type")
        if t == "sensor":
            if isinstance(obj.get("sys"), dict):
                self.sys_info = obj["sys"]
            gen = obj.get("gen")
            if isinstance(gen, int) and gen >= 0:
                self._observed_gen = gen   # v1.5 会话代次跟踪
            if self.on_sensor:
                self.on_sensor(_sensor_from_json(obj))
            if self.on_gamepad and "gamepad" in obj:
                self.on_gamepad(GamepadState.from_json(obj["gamepad"]))
        elif t == "macro" and self.on_macro_status:
            self.on_macro_status(obj)

    def _heartbeat_loop(self):
        """每 500ms 发心跳保活（PROTOCOL.md §2.4：ESP32 超时 2s 自动急停）。"""
        while self._running:
            time.sleep(HEARTBEAT_INTERVAL_S)
            if self._running and not self.send_json({"type": "heartbeat"}):
                self.close()   # 发送失败（对端已死）→ 主动关连，同时解除 tcp 线程的阻塞 recv
                break

    def _udp_loop(self):
        """收 UDP 视频包，组装 JPEG 帧后回调 on_frame。"""
        asm = FrameAssembler()
        self._video_asm = asm      # 暴露诊断统计（GUI 状态栏读取）
        asm.on_bad_frame = self.on_bad_frame
        while self._running:
            try:
                # 64KB 接收缓冲：不假设 ESP32 端分片大小（PEO_NET_VIDEO_MAX_PKT 可能
                # 被调大，若 recvfrom 缓冲小于包长会被静默截断 → 拼帧错位）
                packet, _ = self._udp.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(packet) < 8:
                continue
            hdr = VideoHeader.parse(packet[:8])
            jpeg = asm.push(hdr, packet[8:])
            if jpeg is not None and self.on_frame:
                self.on_frame(hdr.frame_id, jpeg)
