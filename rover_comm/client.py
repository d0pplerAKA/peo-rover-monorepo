"""TCP client with bounded buffers and independent connection lifetimes.

Copyright (c) 2026 Danny
SPDX-License-Identifier: MIT
"""
import json
import logging
import math
import socket
import threading
import time
from collections import deque

from .protocol import TCP_CMD_PORT, VIDEO_TCP_PORT, VIDEO_MAGIC, SensorData, VIDEO_REC

HEARTBEAT_INTERVAL_S = 0.5            # one-way safety heartbeat period (seconds)
MAX_JSON_LINE = 16384                 # max bytes of a single JSON command line
MAX_VIDEO_BYTES = 512 * 1024          # max bytes of one JPEG frame (the framing
                                      # header carries a 4-byte length; anything
                                      # larger is treated as stream corruption)
_LOG = logging.getLogger(__name__)


def jpeg_structure_ok(data: bytes) -> bool:
    """Check marker boundaries, SOF before SOS, scans and terminal EOI.

    This is a structural guard, not an entropy decoder or an image validity
    proof.
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
        last_cmd_status=number(ack.get('status', 0)),
        control_mode=number(j.get('control_mode', 0)),
        fw_version=number(j.get('fw_version', 0)),
        s3_fw=number(j.get('s3_fw', 0)),
        lock=number(j.get('lock', 0)),
        link_stale=j.get('link_stale', False) is not False)



class _Session:
    """Per-connection state bundle shared by the client's worker threads.

    Attributes:
        tcp: the command-channel socket.
        video: the video-channel socket (None until video is requested).
        video_port: TCP port the video channel connects to.
        video_requested: True while VIDEO_ON is active.
        video_revision: bumped on every OFF/ON transition so old in-flight
            connects/reads are invalidated.
        peer: the peer IP string (for the video channel to dial).
        stop: set to end every worker thread for this session.
        ready: set once a telemetry generation has been observed.
        condition: guards pending/buffer state.
        pending: outgoing byte chunks waiting to be sent (bounded deque).
        generation: the current telemetry generation (None until first sensor).
        blocked_generation: generation value ignored while a STOP is in flight.
        stop_acks: count of successful S3 STOP ACKs received.
        threads: worker threads owned by this session.
    """

    def __init__(self, tcp, video_port):
        self.tcp, self.video = tcp, None
        self.video_port = video_port
        self.video_requested = False
        self.video_revision = 0  # OFF/ON transitions invalidate old in-flight tasks.
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
    """Asynchronous TCP client for commands, telemetry, and video.

    Callbacks run on worker threads; callback errors cannot kill receivers.
    Queue acceptance is not an MCU execution acknowledgment. Normal commands
    require a telemetry generation; STOP and the safety heartbeat do not depend
    on telemetry generation.

    Attributes:
        connected: True while a session is active and not stopped.
        generation: the latest observed telemetry generation (or None).
        sys_info: parsed S3 system stats {cpu0, cpu1, ram_free, ram_total}.
        on_sensor / on_frame / on_ack / on_disconnected: user callbacks.
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
        """True while a session exists and its stop event is not set."""
        with self._state_lock:
            return self._session is not None and not self._session.stop.is_set()

    @property
    def generation(self):
        """The latest observed telemetry generation (or None)."""
        return self._observed_gen

    def wait_ready(self, timeout=0.5):
        """Block until a telemetry generation is ready (or timeout).

        Args:
            timeout: seconds to wait.

        Returns:
            True if the session became ready within the timeout.
        """
        with self._state_lock:
            session = self._session
        return bool(session and session.ready.wait(timeout) and not session.stop.is_set())

    @staticmethod
    def _callback(callback, *args):
        """Invoke a user callback; any exception is logged, never propagated."""
        if callback:
            try:
                callback(*args)
            except Exception:
                _LOG.exception('Rover callback failed')

    @staticmethod
    def _dispose(sock):
        """Shutdown (to wake blocked reads) and close a socket, if any."""
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()

    def connect(self, host, tcp_port=TCP_CMD_PORT, video_port=VIDEO_TCP_PORT):
        """Establish the command connection; the video port is dialed only after
        VIDEO_ON is sent.

        video_port=None disables video for this session (VIDEO_ON returns
        False). The video thread blocks idle without opening TCP or retrying.
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
        """Tear down a session: clear state, wake waiters, close sockets, then
        emit on_disconnected before a new session can be installed."""
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
        """Close the current session and cancel any in-flight connect, then join
        the worker threads (best-effort, bounded timeout)."""
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
        """Queue one batch of up to four JSON commands atomically.

        STOP_ALL must be sent alone (it clears the queue and invalidates the
        current generation). Non-heartbeat commands require a known generation
        and get `gen` attached automatically.

        Args:
            objects: list of command dicts.
            expected_session: if set, the batch is dropped unless it targets
                this exact session.

        Returns:
            True if the batch was queued (or trivially empty), False if the
            queue is full / no generation / wrong session.
        """
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
        """Queue a single JSON command (convenience over send_json_batch)."""
        return self.send_json_batch([obj])

    def stop_all(self):
        """Queue a priority STOP_ALL."""
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
        """Request video streaming (VIDEO_ON)."""
        return self.send_json({'cmd': 'VIDEO_ON'})

    def video_off(self):
        """Request video streaming to stop (VIDEO_OFF)."""
        return self.send_json({'cmd': 'VIDEO_OFF'})

    def reset_odometer(self):
        """Reset the four wheel odometers (ODOMETER_RESET)."""
        return self.send_json({'cmd': 'ODOMETER_RESET'})

    def sys_reset(self):
        """Soft-reset the F407 (SYS_RESET)."""
        return self.send_json({'cmd': 'SYS_RESET'})

    def led_toggle(self):
        """Toggle the onboard LED (LED_TOGGLE)."""
        return self.send_json({'cmd': 'LED_TOGGLE'})

    def led_on(self):
        """Turn the onboard LED on (LED_ON)."""
        return self.send_json({'cmd': 'LED_ON'})

    def led_off(self):
        """Turn the onboard LED off (LED_OFF)."""
        return self.send_json({'cmd': 'LED_OFF'})

    def motor(self, motor, speed):
        """Open-loop motor command (MOTOR): motor index + per-mille speed."""
        return self.send_json({'cmd': 'MOTOR', 'motor': motor, 'speed': speed})

    def _sender_loop(self, session):
        """Send thread: drains the pending queue with a blocking sendall."""
        try:
            while not session.stop.is_set():
                with session.condition:
                    session.condition.wait_for(lambda: session.pending or session.stop.is_set())
                    if session.stop.is_set():
                        return
                    data = session.pending.popleft()
                session.tcp.sendall(data)  # at most one committed batch may precede STOP
                # Track video demand from what was actually SENT, not merely
                # queued: a STOP may clear an un-sent VIDEO_ON, which must not
                # leave a permanent reconnect task with no video stream.
                # GUI, SDK, and raw JSON share this path; no extra round-trips
                # or ACK waits are added.
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
                                self._dispose(session.video)  # interrupt a blocked recv.
                                session.video = self._video = None
                            session.condition.notify_all()
        except OSError:
            pass
        finally:
            self._close_session(session)

    def _tcp_loop(self, session):
        """Command-channel receive thread: newline-delimited JSON lines."""
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
        """Route one parsed JSON object to ACK handling or sensor handling."""
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
        """One-way safety heartbeat: keeps the S3 watchdog alive. No ACK, no
        timestamp, no RTT measurement, and it never restores motion."""
        while not session.stop.wait(HEARTBEAT_INTERVAL_S):
            with self._state_lock:
                if self._session is not session:
                    return
                if not self.send_json_batch([{'type': 'heartbeat'}], expected_session=session):
                    self._close_session(session)
                    return

    @staticmethod
    def _recv_exact(sock, length):
        """Read exactly length bytes; timeout/EOF/error returns None (TCP frame
        boundaries are non-negotiable)."""
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
        """On-demand video receiver: 8-byte framing header + JPEG body, with a
        5 s read timeout and 2 s reconnect backoff, only while video is on.

        Every ON/OFF uses its own revision; an old connection or old frame that
        finishes during an off/re-on is dropped so it can never overwrite a new
        socket. Reconnects always dial this session's video port.
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
                        break  # framing lost sync: cannot reuse the stream, reconnect.
                    body = self._recv_exact(sock, length)
                    if body is None:
                        break
                    with self._state_lock:
                        if (self._session is not session or not session.video_requested
                                or session.video_revision != revision):
                            break
                        # Keep the structural guard: frames missing SOF/SOS/EOI
                        # must never reach a decoder.
                        if not jpeg_structure_ok(body):
                            continue
                        self._callback(self.on_frame, fid, body)
            except OSError:
                pass  # video failure must not affect the command/telemetry channel.
            finally:
                with self._state_lock:
                    if session.video is sock:
                        session.video = None
                        if self._session is session:
                            self._video = None
                    self._dispose(sock)
            # OFF, disconnect, and the next ON can all interrupt the backoff
            # immediately; nothing reconnects while video is off.
            with session.condition:
                session.condition.wait_for(
                    lambda: session.stop.is_set() or not session.video_requested
                    or session.video_revision != revision, timeout=2.0)
