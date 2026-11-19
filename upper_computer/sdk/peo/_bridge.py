"""SDK connection/snapshot bridge; one monitor per connection lifetime.

Copyright (c) 2026 Danny
SPDX-License-Identifier: MIT
"""
import json
import math
import os
import tempfile
import threading
import time

from rover_comm.client import RoverClient
from rover_comm.control import ControlLock
from ._i18n import tr

FRESH_MS = 250                     # a sensor snapshot is "fresh" within 250 ms
_stop_requested = threading.Event()   # cooperative script-cancellation flag


class ScriptStopped(Exception):
    """Raised when the IDE stop button cancels a running script."""


class NotConnected(Exception):
    """Raised when an action needs a connection but none is active."""


class LinkLost(Exception):
    """Raised when the connection dropped, or a command could not be queued
    safely."""


class CamOff(Exception):
    """Raised when a decoded camera read is requested while video is off."""


class _Bridge:
    """Wires the SDK action functions to a RoverClient plus a 50 Hz sensor
    snapshot, the control lease, and the on-disk PID/resolution cache.

    Attributes:
        client: the underlying RoverClient (TCP command/telemetry/video).
        _snap: the latest SensorData snapshot (None until first packet).
        _snap_t: monotonic time of the latest snapshot (seconds).
        _lock: guards _snap/_snap_t.
        _lease: local ControlLock serializing the GUI vs SDK.
        _lifecycle: serializes connect/disconnect calls.
        _config_lock: guards config-file writes.
        _config: cached {'pid': {...}, 'cam_res': ...} from the config file.
        _recovered: True once config was re-sent after a fresh recovery edge.
        _connected: whether the bridge thinks a connection is active.
        _host: the hostname/IP last connected to.
        _monitor_stop/_monitor_t: the config-reapply monitor thread.
    """

    def __init__(self):
        self.client = RoverClient()
        self._snap = None
        self._snap_t = 0.0
        self._lock = threading.Lock()
        self._lease = ControlLock()
        self._lifecycle = threading.Lock()
        self._config_lock = threading.Lock()
        self._config = {}
        self._recovered = False
        self._connected = False
        self._host = None
        self._monitor_stop = threading.Event()
        self._monitor_t = None
        self.client.on_sensor = self._on_sensor
        self.client.on_disconnected = self._on_disconnected

    def connect(self, host='peo_rover.local'):
        """Connect to the ESP32 (acquires the local lease, loads cached config,
        starts the config monitor). Raises ScriptStopped if a stop was already
        requested."""
        if _stop_requested.is_set():
            raise ScriptStopped(tr('script stopped', '脚本已停止'))
        with self._lifecycle:
            if self._connected and self.client.connected and self._host == host:
                return
            self.disconnect()
            self._acquire_lock()
            try:
                self._load_config()
                self.client.connect(host)
                self._connected = self.client.connected
                if not self._connected:
                    raise ConnectionError('Connection closed during startup')
                self._host = host
                self._recovered = False
                self._start_monitor()
            except Exception:
                self.disconnect()
                raise

    def disconnect(self):
        """Stop the monitor and close the client; releases the lease."""
        self._monitor_stop.set()
        self.client.close()
        self._on_disconnected()

    def _acquire_lock(self):
        self._lease.acquire()

    def _release_lock(self):
        self._lease.release()

    def _on_sensor(self, sensor):
        """Store the newest snapshot; a backwards system-tick jump (uint32
        rollover is forward) marks the vehicle as reset (needs config re-send)."""
        with self._lock:
            if self._snap is not None:
                old_tick, tick = int(self._snap.sys_tick_ms), int(sensor.sys_tick_ms)
                # uint32 rollover is forward progress; a backwards jump is a reset.
                if ((tick - old_tick) & 0xFFFFFFFF) > 0x7FFFFFFF:
                    self._recovered = False
            self._snap = sensor
            self._snap_t = time.monotonic()

    def _on_disconnected(self):
        """Clear connection state, release the lease, and reset the camera."""
        self._connected = False
        self._recovered = False
        self._monitor_stop.set()
        self._release_lock()
        with self._lock:
            self._snap = None
            self._snap_t = 0.0
        from .cam import cam
        cam._reset()

    def _snapshot(self):
        """Return the latest snapshot, or None when not connected."""
        with self._lock:
            return self._snap if self._connected else None

    @property
    def fresh(self):
        """True when connected, a snapshot exists, the S3 link is not stale,
        and the snapshot arrived within FRESH_MS."""
        with self._lock:
            return bool(self._connected and self._snap is not None and not self._snap.link_stale
                        and time.monotonic() - self._snap_t < FRESH_MS / 1000)

    def _config_path(self):
        """Path of the JSON config cache (~/.peo_rover_config.json, or the
        PEO_CONFIG_PATH override)."""
        return os.environ.get('PEO_CONFIG_PATH') or os.path.join(os.path.expanduser('~'), '.peo_rover_config.json')

    def _load_config(self):
        """Load and validate the PID/resolution cache; invalid/missing entries
        are dropped rather than crashing."""
        try:
            with open(self._config_path(), encoding='utf-8') as stream:
                config = json.load(stream)
            if not isinstance(config, dict):
                config = {}
        except (OSError, ValueError):
            config = {}
        clean = {}
        pid = config.get('pid')
        if isinstance(pid, dict):
            values = [pid.get('kp', 0.25), pid.get('ki', 0.005)]
            if all(type(v) in (float, int) and abs(v) <= 3.402823466e38 and math.isfinite(v) for v in values):
                clean['pid'] = dict(zip(('kp', 'ki'), values))
        if config.get('cam_res') in ('QQVGA', 'QVGA', 'VGA', 'SVGA'):
            clean['cam_res'] = config['cam_res']
        self._config = clean

    def _save_config(self):
        """Atomically write the config cache (temp file + os.replace)."""
        temporary = None
        with self._config_lock:
            try:
                path = os.path.abspath(self._config_path())
                with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=os.path.dirname(path), delete=False) as stream:
                    temporary = stream.name
                    json.dump(self._config, stream, ensure_ascii=False, indent=2, allow_nan=False)
                os.replace(temporary, path)
                temporary = None
            except (OSError, ValueError):
                pass
            finally:
                if temporary:
                    try:
                        os.unlink(temporary)
                    except OSError:
                        pass

    def _push_config(self, session=None):
        """Re-send the cached PID and resolution to the vehicle."""
        commands = []
        pid = self._config.get('pid')
        if pid:
            commands.append({'cmd': 'PID', 'kp': pid['kp'], 'ki': pid['ki']})
        commands.append({'cmd': 'CAM_RES', 'res': self._config.get('cam_res', 'QVGA')})
        return self.client.send_json_batch(commands, expected_session=session)

    def _start_monitor(self):
        """Start a background thread that re-applies PID/resolution whenever the
        link recovers from stale (fresh recovery edge)."""
        stop = self._monitor_stop = threading.Event()
        session = self.client._session

        def monitor():
            while not stop.wait(0.1):
                if not self._connected:
                    return
                if not self.fresh:
                    self._recovered = False
                elif not self._recovered and not _stop_requested.is_set():
                    restored = self._push_config(session)
                    if not stop.is_set():
                        self._recovered = restored
        self._monitor_t = threading.Thread(target=monitor, daemon=True, name='peo-monitor')
        self._monitor_t.start()

    def _check_send(self):
        """Pre-flight checks before sending: not stopped, connected, and a
        current telemetry generation is known. Raises the matching exception."""
        if _stop_requested.is_set():
            raise ScriptStopped(tr('script stopped', '脚本已停止'))
        if not self._connected or not self.client.connected:
            raise NotConnected(tr('not connected: call peo.connect(...) first', '未连接：先 peo.connect(...)'))
        if not self.client.wait_ready(0.5):
            raise LinkLost(tr('No current telemetry generation; command not sent', '未收到当前遥测代次，命令未发送'))
        if _stop_requested.is_set():
            raise ScriptStopped(tr('script stopped', '脚本已停止'))

    def send(self, obj):
        """Send one JSON command after pre-flight checks; True on success."""
        self._check_send()
        if not self.client.send_json(obj):
            raise LinkLost(tr('Command not queued: disconnected or busy', '命令未入队：连接中断或队列忙'))
        return True

    def send_batch(self, commands):
        """Send a batch of drive commands; on local overload, request STOP so a
        previous motion target is never left running silently."""
        self._check_send()
        if not self.client.send_json_batch(commands):
            # Avoid leaving a previous motion target running after local overload.
            self.client.stop_all()
            raise LinkLost(tr('Drive batch not queued; STOP requested', '四轮命令未入队，已请求停车'))

    def emergency_stop(self):
        """Send a priority STOP and wait for a fresh S3 ACK; raises TimeoutError
        if unconfirmed. Stays callable after script cancellation."""
        # Must remain callable from a ScriptStopped exception handler.
        if not self.client.connected:
            raise NotConnected(tr('not connected', '未连接'))
        if not self.client.stop_and_wait():
            raise TimeoutError(tr('STOP acceptance was not confirmed by S3', '未收到 S3 的 STOP 接收确认'))


_bridge = _Bridge()
