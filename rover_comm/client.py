"""TCP client with bounded buffers and independent connection lifetimes."""
import json
import logging
import math
import socket
import threading
import time
from collections import deque

from .protocol import TCP_CMD_PORT, VIDEO_TCP_PORT, VIDEO_MAGIC, SensorData, VIDEO_REC

HEARTBEAT_INTERVAL_S = 0.5
MAX_JSON_LINE = 16384
MAX_VIDEO_BYTES = 512 * 1024   # 单帧 JPEG 上限（定界头 4B 长度，超限即视为流错位）
_LOG = logging.getLogger(__name__)


def jpeg_structure_ok(data: bytes) -> bool:
    """Check marker boundaries, SOF before SOS, scans and terminal EOI.

    This is a structural guard, not an entropy decoder or an image validity proof.
    """
    if len(data) < 4 or data[:2] != b'\xff\xd8':
        return False
    i, n = 2, len(data)
    saw_sof = saw_sos = False
    while i < n:
        if data[i] != 0xFF:
            return False
        while i < n and data[i] == 0xFF:
            i += 1
        if i >= n:
            return False
        marker = data[i]
        i += 1
        if marker == 0xD9:
            return saw_sof and saw_sos and i == n
        if marker in (0x00, 0xD8) or 0xD0 <= marker <= 0xD7:
            return False
        if marker == 0x01:
            continue
        if i + 2 > n:
            return False
        size = int.from_bytes(data[i:i + 2], 'big')
        if size < 2 or i + size > n:
            return False
        if marker in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
                      0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF):
            if size < 8:
                return False
            height = int.from_bytes(data[i + 3:i + 5], 'big')
            width = int.from_bytes(data[i + 5:i + 7], 'big')
            if not width or not height or width * height > 5_000_000:
                return False
            saw_sof = True
        i += size
        if marker == 0xDA:
            if not saw_sof or size < 6:
                return False
            saw_sos = True
            while i < n:
                start = data.find(b'\xff', i)
                if start < 0:
                    return False
                i = start
                while i < n and data[i] == 0xFF:
                    i += 1
                if i >= n:
                    return False
                if data[i] == 0 or 0xD0 <= data[i] <= 0xD7:
                    i += 1
                    continue
                i = start  # next scan/EOI is parsed by the outer marker loop
                break
    return False


def _sensor_from_json(j: dict) -> SensorData:
    """Reject malformed telemetry; missing measurements are marked invalid."""
    def obj(parent, key):
        value = parent.get(key, {})
        if not isinstance(value, dict):
            raise ValueError(f'invalid {key}')
        return value

    def number(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('invalid telemetry number')
        return value

    def vector(parent, key, length):
        values = parent.get(key, [0] * length)
        if not isinstance(values, list) or len(values) != length:
            raise ValueError(f'invalid {key}')
        return tuple(number(v) for v in values)

    dist, imu = obj(j, 'dist'), obj(j, 'imu')
    wheels, power, ack = obj(j, 'wheels'), obj(j, 'power'), obj(j, 'cmd_ack')
    distances = [obj(dist, k) for k in ('front', 'left', 'right')]
    quat, accel = vector(imu, 'quat', 4), vector(imu, 'accel_g', 3)
    return SensorData(
        dist_front_mm=number(distances[0].get('mm', 0)),
        dist_left_mm=number(distances[1].get('mm', 0)),
        dist_right_mm=number(distances[2].get('mm', 0)),
        dist_front_status=0 if distances[0].get('ok') is True else 1,
        dist_left_status=0 if distances[1].get('ok') is True else 1,
        dist_right_status=0 if distances[2].get('ok') is True else 1,
        quat_w=quat[0], quat_x=quat[1], quat_y=quat[2], quat_z=quat[3],
        accel_x=accel[0], accel_y=accel[1], accel_z=accel[2],
        speed_d100=vector(wheels, 'speed_d100', 4), odometer=vector(wheels, 'odometer', 4),
        current_mA=vector(power, 'current_mA', 4), bus_voltage_mV=vector(power, 'bus_voltage_mV', 4),
        sys_tick_ms=number(j.get('tick', 0)), last_cmd_id=number(ack.get('id', 0)),
        last_cmd_status=number(ack.get('status', 0)), link_stale=j.get('link_stale', False) is not False)



class _Session:
    def __init__(self, tcp, video_port):
        self.tcp, self.video = tcp, None
        self.video_port = video_port
        self.video_requested = False
        self.video_revision = 0  # OFF/ON 切换使正在连接或读取的旧任务失效。
        self.peer = tcp.getpeername()[0]
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.condition = threading.Condition()
        self.pending = deque()
        self.generation = None
        self.blocked_generation = None
        self.stop_acks = 0
        self.threads = []


class RoverClient:
    """Callbacks run on worker threads; callback errors cannot kill receivers.

    Queue acceptance is not an MCU execution acknowledgment. Normal commands
    require a telemetry generation; STOP/安全心跳不依赖遥测代次。
    """
    def __init__(self):
        self._state_lock = threading.RLock()
        self._connect_lock = threading.Lock()
        self._stop_lock = threading.Lock()
        self._session = None
        self._epoch = 0
        self._tcp = self._video = None
        self._running = False
        self._observed_gen = None
        self.sys_info = None
        self.on_sensor = self.on_frame = self.on_ack = self.on_disconnected = None

    @property
    def connected(self):
        with self._state_lock:
            return self._session is not None and not self._session.stop.is_set()

    @property
    def generation(self):
        return self._observed_gen

    def wait_ready(self, timeout=0.5):
        with self._state_lock:
            session = self._session
        return bool(session and session.ready.wait(timeout) and not session.stop.is_set())

    @staticmethod
    def _callback(callback, *args):
        if callback:
            try:
                callback(*args)
            except Exception:
                _LOG.exception('Rover callback failed')

    @staticmethod
    def _dispose(sock):
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()

    def connect(self, host, tcp_port=TCP_CMD_PORT, video_port=VIDEO_TCP_PORT):
        """只建立命令连接；VIDEO_ON 发出后才连接指定视频端口。

        video_port=None 表示此会话禁用视频（VIDEO_ON 返回 False）。
        视频线程空闲时阻塞等待，不建立 TCP，也不进行超时重连。
        """
        with self._connect_lock:
            self.close()
            with self._state_lock:
                epoch = self._epoch
            tcp = None
            try:
                tcp = socket.create_connection((host, tcp_port), timeout=5)
                tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                tcp.settimeout(0.5)  # bound send stalls; receiver handles idle timeouts
                session = _Session(tcp, video_port)
                with self._state_lock:
                    if epoch != self._epoch:
                        raise ConnectionError('Connection cancelled')
                    self._session = session
                    self._tcp, self._video, self._running = tcp, None, True
                    self._observed_gen = self.sys_info = None
                    workers = [('rover-send', self._sender_loop), ('rover-tcp', self._tcp_loop),
                               ('rover-heartbeat', self._heartbeat_loop)]
                    if video_port is not None:
                        workers.append(('rover-video', self._video_loop))
                    for name, target in workers:
                        thread = threading.Thread(target=target, args=(session,), daemon=True, name=name)
                        session.threads.append(thread)
                        thread.start()
                return True
            except OSError as exc:
                self._dispose(tcp)
                raise ConnectionError(f'Connection failed: {exc}') from exc

    def _close_session(self, session):
        with self._state_lock:
            if self._session is not session:
                return
            self._session = None
            self._running = False
            self._tcp = self._video = None
            self._observed_gen = self.sys_info = None
            session.stop.set()
            session.ready.set()  # wake waiters; they also check stop
            with session.condition:
                session.pending.clear()
                session.condition.notify_all()
            self._dispose(session.tcp)
            self._dispose(session.video)
            # Serialize reset callbacks before a new session can be installed.
            self._callback(self.on_disconnected)

    def close(self):
        with self._state_lock:
            self._epoch += 1  # also cancels an in-flight connect
            session = self._session
            if session:
                self._close_session(session)
        if session:
            current = threading.current_thread()
            for thread in session.threads:
                if thread is not current:
                    thread.join(timeout=0.6)

    def send_json_batch(self, objects, *, expected_session=None):
        if len(objects) > 4:
            raise ValueError("At most four commands per batch")
        with self._state_lock:
            session = self._session
            if session is None or (expected_session is not None and session is not expected_session):
                return False
            lines = []
            priority = False
            for original in objects:
                obj = dict(original)
                command = obj.get('cmd')
                if command == 'STOP_ALL':
                    if len(objects) != 1:
                        raise ValueError('STOP_ALL must be sent separately')
                    priority = True
                elif obj.get('type') != 'heartbeat' or command is not None:
                    if session.generation is None:
                        return False
                    obj.setdefault('gen', session.generation)
                if command == 'VIDEO_ON':
                    if session.video_port is None:
                        return False
                line = json.dumps(obj, separators=(',', ':'), allow_nan=False).encode('utf-8')
                if len(line) > MAX_JSON_LINE:
                    raise ValueError('Command line exceeds 16384 bytes')
                lines.append(line + b'\n')
            if not lines:
                return True
            data = b''.join(lines)
            if len(data) > 65536:
                raise ValueError('Command batch exceeds 65536 bytes')
            with session.condition:
                if priority:
                    session.pending.clear()
                    if session.generation is not None:
                        session.blocked_generation = session.generation
                    session.generation = self._observed_gen = None
                    session.ready.clear()
                elif len(session.pending) >= 64:
                    return False
                session.pending.append(data)
                session.condition.notify_all()
            return True

    def send_json(self, obj):
        return self.send_json_batch([obj])

    def stop_all(self):
        return self.send_json({'cmd': 'STOP_ALL'})

    def stop_and_wait(self, timeout=0.3, attempts=2):
        """Wait for a newly received successful S3 STOP ACK, not old telemetry.

        The wire protocol has no request ID; this confirms S3 acceptance only.
        """
        with self._stop_lock:
            for _ in range(attempts):
                with self._state_lock:
                    session = self._session
                    if not session:
                        return False
                    before = session.stop_acks
                    if not self.stop_all():
                        return False
                with session.condition:
                    session.condition.wait_for(
                        lambda: session.stop_acks > before or session.stop.is_set(), timeout)
                    if session.stop_acks > before:
                        return True
                    if session.stop.is_set():
                        return False
            return False

    def video_on(self):
        return self.send_json({'cmd': 'VIDEO_ON'})

    def video_off(self):
        return self.send_json({'cmd': 'VIDEO_OFF'})

    def reset_odometer(self):
        return self.send_json({'cmd': 'ODOMETER_RESET'})

    def sys_reset(self):
        return self.send_json({'cmd': 'SYS_RESET'})

    def _sender_loop(self, session):
        try:
            while not session.stop.is_set():
                with session.condition:
                    session.condition.wait_for(lambda: session.pending or session.stop.is_set())
                    if session.stop.is_set():
                        return
                    data = session.pending.popleft()
                session.tcp.sendall(data)  # at most one committed batch may precede STOP
                # 跟随真正发出的命令更新视频需求，而非仅跟随入队：STOP 可能清掉
                # 未发送的 VIDEO_ON，不能因此留下没有视频流的永久重连任务。
                # GUI、SDK、原始 JSON 共用此处；不增加任何往返或 ACK 等待。
                with self._state_lock:
                    if self._session is not session:
                        return
                    requested = session.video_requested
                    for line in data.splitlines():
                        command = json.loads(line).get('cmd')
                        if command == 'VIDEO_ON':
                            requested = True
                        elif command == 'VIDEO_OFF':
                            requested = False
                    if requested != session.video_requested:
                        with session.condition:
                            session.video_requested = requested
                            session.video_revision += 1
                            if not requested:
                                self._dispose(session.video)  # 打断阻塞 recv。
                                session.video = self._video = None
                            session.condition.notify_all()
        except OSError:
            pass
        finally:
            self._close_session(session)

    def _tcp_loop(self, session):
        buf = bytearray()
        try:
            while not session.stop.is_set():
                try:
                    chunk = session.tcp.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buf.extend(chunk)
                while b'\n' in buf:
                    end = buf.index(b'\n')
                    if end > MAX_JSON_LINE:
                        return
                    line = bytes(buf[:end])
                    del buf[:end + 1]
                    if not line.strip():
                        continue
                    try:
                        obj = json.loads(line)
                        if isinstance(obj, dict):
                            self._dispatch(obj, session)
                    except (ValueError, TypeError, OverflowError, RecursionError):
                        _LOG.debug('Discarded invalid telemetry/JSON')
                if len(buf) > MAX_JSON_LINE:
                    break
        except OSError:
            pass
        finally:
            self._close_session(session)

    def _dispatch(self, obj, session=None):
        with self._state_lock:
            session = session or self._session
            if session is None or self._session is not session:
                return
            if 'ack' in obj and 'type' not in obj:
                if obj.get('ack') == 'STOP_ALL' and obj.get('status') == 0:
                    with session.condition:
                        session.stop_acks += 1
                        session.condition.notify_all()
                self._callback(self.on_ack, obj)
            elif obj.get('type') == 'sensor':
                sensor = _sensor_from_json(obj)
                gen = obj.get('gen')
                if type(gen) is int and 0 <= gen <= 0xFFFFFFFF and gen != session.blocked_generation:
                    session.generation = self._observed_gen = gen
                    session.ready.set()
                if isinstance(obj.get('sys'), dict):
                    self.sys_info = {key: value for key, value in obj['sys'].items()
                                     if key in ('cpu0', 'cpu1', 'ram_free', 'ram_total')
                                     and type(value) in (int, float) and 0 <= value <= 0xFFFFFFFF
                                     and math.isfinite(value)}
                self._callback(self.on_sensor, sensor)

    def _heartbeat_loop(self, session):
        while not session.stop.wait(HEARTBEAT_INTERVAL_S):
            with self._state_lock:
                if self._session is not session:
                    return
                # 单向安全续期：不回 ACK、不携带时间戳、不测 RTT，也不恢复运动。
                if not self.send_json_batch([{'type': 'heartbeat'}], expected_session=session):
                    self._close_session(session)
                    return

    @staticmethod
    def _recv_exact(sock, length):
        """读满 length 字节；超时/EOF/错误返回 None（TCP 帧边界不可妥协）。"""
        parts = []
        remaining = length
        try:
            while remaining > 0:
                chunk = sock.recv(remaining)
                if not chunk:
                    return None
                parts.append(chunk)
                remaining -= len(chunk)
        except OSError:
            return None
        return b''.join(parts)

    def _video_loop(self, session):
        """按需接收 12B 定界头和 JPEG；只有视频开启时才进行 5s 超时/2s 重连。

        每次 ON/OFF 使用独立 revision。关闭/重新开启期间完成的旧连接或旧帧
        均丢弃，防止旧工作覆盖新 socket；重连始终使用本会话指定的视频端口。
        """
        while not session.stop.is_set():
            with session.condition:
                session.condition.wait_for(lambda: session.stop.is_set() or session.video_requested)
            with self._state_lock:
                if self._session is not session or session.stop.is_set():
                    return
                if not session.video_requested:
                    continue
                revision = session.video_revision
            sock = None
            try:
                sock = socket.create_connection((session.peer, session.video_port), timeout=3)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.settimeout(5.0)
                with self._state_lock:
                    if (self._session is not session or session.stop.is_set()
                            or not session.video_requested or session.video_revision != revision):
                        continue
                    session.video = self._video = sock
                while not session.stop.is_set():
                    hdr = self._recv_exact(sock, VIDEO_REC.size)
                    if hdr is None:
                        break
                    magic, fid, length = VIDEO_REC.unpack(hdr)
                    if magic != VIDEO_MAGIC or not 1 <= length <= MAX_VIDEO_BYTES:
                        break  # 定界错位不能继续复用字节流，关闭后重连。
                    body = self._recv_exact(sock, length)
                    if body is None:
                        break
                    with self._state_lock:
                        if (self._session is not session or not session.video_requested
                                or session.video_revision != revision):
                            break
                        # 结构守卫保留：缺 SOF/SOS/EOI 等坏帧绝不送入解码器。
                        if not jpeg_structure_ok(body):
                            continue
                        self._callback(self.on_frame, fid, body)
            except OSError:
                pass  # 视频失败不影响命令/遥测通道。
            finally:
                with self._state_lock:
                    if session.video is sock:
                        session.video = None
                        if self._session is session:
                            self._video = None
                    self._dispose(sock)
            # OFF、断连和下一次 ON 都可立即打断退避；关闭期间不会再连接。
            with session.condition:
                session.condition.wait_for(
                    lambda: session.stop.is_set() or not session.video_requested
                    or session.video_revision != revision, timeout=2.0)
