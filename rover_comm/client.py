"""TCP/UDP client with bounded buffers and independent connection lifetimes."""
import json
import logging
import math
import socket
import threading
import time
from collections import deque

from .protocol import TCP_CMD_PORT, UDP_VIDEO_PORT, VIDEO_MAGIC, SensorData, VideoHeader

HEARTBEAT_INTERVAL_S = 0.5
MAX_JSON_LINE = 16384
MAX_VIDEO_BYTES = 512 * 1024
MAX_VIDEO_PACKETS = (MAX_VIDEO_BYTES + 1399) // 1400
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


class FrameAssembler:
    """One bounded frame; accept reordered fragments after packet 0.

    A different frame starts only with packet 0. Serial-number ordering rejects
    late old packet-0 datagrams; after a 2s idle gap the sequence can restart.
    """
    def __init__(self):
        self._frame_id = -1
        self._total = 0
        self._packets = {}
        self._bytes = 0
        self._frame_start = 0.0
        self._done = False
        self.packets_total = self.packets_late = self.packets_bad = 0
        self.frames_ok = self.frames_aborted = self.frames_bad_jpeg = 0
        self.last_jpeg_len = 0

    def push(self, hdr: VideoHeader, data: bytes) -> bytes | None:
        self.packets_total += 1
        if (hdr.magic != VIDEO_MAGIC or not 1 <= hdr.total_pkt <= MAX_VIDEO_PACKETS
                or not 0 <= hdr.packet_id < hdr.total_pkt or not 0 <= hdr.frame_id <= 65535
                or not 0 < len(data) <= 1400):
            self.packets_bad += 1
            return None
        now = time.monotonic()
        expired = now - self._frame_start > 2.0
        if hdr.frame_id != self._frame_id or expired:
            delta = (hdr.frame_id - self._frame_id) & 0xFFFF
            if hdr.packet_id != 0 or (self._frame_id >= 0 and not expired and delta >= 0x8000):
                self.packets_late += 1
                return None
            if self._packets:
                self.frames_aborted += 1
            self._frame_id, self._total = hdr.frame_id, hdr.total_pkt
            self._packets, self._bytes = {}, 0
            self._frame_start, self._done = now, False
        if self._done:
            self.packets_late += 1
            return None
        if hdr.total_pkt != self._total:
            self.packets_bad += 1
            return None
        previous = self._packets.get(hdr.packet_id)
        if previous is not None:
            if previous != data:
                self.packets_bad += 1
                self._packets.clear()
                self._done = True
                self.frames_aborted += 1
            return None
        self._bytes += len(data)
        if self._bytes > MAX_VIDEO_BYTES:
            self._packets.clear()
            self._done = True
            self.packets_bad += 1
            return None
        self._packets[hdr.packet_id] = data
        if len(self._packets) != self._total:
            return None
        jpeg = b''.join(self._packets[i] for i in range(self._total))
        self._packets.clear()
        self._done = True
        if not jpeg_structure_ok(jpeg):
            self.frames_bad_jpeg += 1
            return None
        self.frames_ok += 1
        self.last_jpeg_len = len(jpeg)
        return jpeg


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
    def __init__(self, tcp, udp):
        self.tcp, self.udp = tcp, udp
        self.peer = tcp.getpeername()[0]
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.condition = threading.Condition()
        self.pending = deque()
        self.generation = None
        self.blocked_generation = None
        self.stop_acks = 0
        self.ping_t = None
        self.threads = []


class RoverClient:
    """Callbacks run on worker threads; callback errors cannot kill receivers.

    Queue acceptance is not an MCU execution acknowledgment. Normal commands
    require a telemetry generation; STOP/PING remain available before telemetry.
    """
    def __init__(self):
        self._state_lock = threading.RLock()
        self._connect_lock = threading.Lock()
        self._stop_lock = threading.Lock()
        self._session = None
        self._epoch = 0
        self._tcp = self._udp = None
        self._running = False
        self._observed_gen = None
        self._video_asm = None
        self.sys_info = None
        self.last_rtt_ms = None
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

    def connect(self, host, tcp_port=TCP_CMD_PORT, udp_port=UDP_VIDEO_PORT):
        with self._connect_lock:
            self.close()
            with self._state_lock:
                epoch = self._epoch
            tcp = udp = None
            try:
                tcp = socket.create_connection((host, tcp_port), timeout=5)
                tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                tcp.settimeout(0.5)  # bound send stalls; receiver handles idle timeouts
                if udp_port is not None:
                    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 256 * 1024)
                    udp.bind(('0.0.0.0', udp_port))
                    udp.settimeout(0.5)
                session = _Session(tcp, udp)
                with self._state_lock:
                    if epoch != self._epoch:
                        raise ConnectionError('Connection cancelled')
                    self._session = session
                    self._tcp, self._udp, self._running = tcp, udp, True
                    self._observed_gen = self.sys_info = self.last_rtt_ms = None
                    self._video_asm = FrameAssembler() if udp else None
                    workers = [('rover-send', self._sender_loop), ('rover-tcp', self._tcp_loop),
                               ('rover-heartbeat', self._heartbeat_loop)]
                    if udp:
                        workers.append(('rover-udp', self._udp_loop))
                    for name, target in workers:
                        thread = threading.Thread(target=target, args=(session,), daemon=True, name=name)
                        session.threads.append(thread)
                        thread.start()
                return True
            except OSError as exc:
                self._dispose(tcp)
                self._dispose(udp)
                raise ConnectionError(f'Connection failed: {exc}') from exc

    def _close_session(self, session):
        with self._state_lock:
            if self._session is not session:
                return
            self._session = None
            self._running = False
            self._tcp = self._udp = None
            self._observed_gen = self.sys_info = self.last_rtt_ms = self._video_asm = None
            session.stop.set()
            session.ready.set()  # wake waiters; they also check stop
            with session.condition:
                session.pending.clear()
                session.condition.notify_all()
            self._dispose(session.tcp)
            self._dispose(session.udp)
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
                elif command != 'PING':
                    if session.generation is None:
                        return False
                    obj.setdefault('gen', session.generation)
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

    def ping(self):
        with self._state_lock:
            session = self._session
            if not session:
                return False
            session.ping_t = time.monotonic_ns() // 1_000_000
            return self.send_json({'cmd': 'PING', 't': session.ping_t})

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
                if obj.get('ack') == 'PING' and session.ping_t is not None and obj.get('t') == session.ping_t:
                    self.last_rtt_ms = time.monotonic_ns() // 1_000_000 - session.ping_t
                    session.ping_t = None
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
                if not self.ping():
                    self._close_session(session)
                    return

    def _udp_loop(self, session):
        with self._state_lock:
            asm = self._video_asm
        while not session.stop.is_set():
            try:
                packet, source = session.udp.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if source[0] != session.peer or source[1] != UDP_VIDEO_PORT or len(packet) < 8:
                continue
            hdr = VideoHeader.parse(packet[:8])
            jpeg = asm.push(hdr, packet[8:])
            if jpeg is not None:
                with self._state_lock:
                    if self._session is not session:
                        return
                    self._callback(self.on_frame, hdr.frame_id, jpeg)
