"""Offline failure-path regressions: python tests/regression_test.py.

Use this file directly to run only regressions. No vehicle, real sockets or user
configuration are used. GUI checks run offscreen when PySide6 is installed.
"""
import importlib.util
import json
import os
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch, Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'sdk')]
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from rover_comm.client import RoverClient, FrameAssembler, jpeg_structure_ok, _sensor_from_json
from rover_comm.protocol import SensorData, VideoHeader, VIDEO_MAGIC
import peo
from peo._bridge import _Bridge, _stop_requested

JPEG = (b'\xff\xd8\xff\xc0\x00\x0b\x08\x00\x01\x00\x01\x01\x01\x11\x00'
        b'\xff\xda\x00\x08\x01\x01\x00\x00\x3f\x00abc\xff\xd9')


def eventually(predicate, timeout=1.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return bool(predicate())


class FakeTCP:
    def __init__(self, stop_ack=False):
        self.rx, self.sent = queue.Queue(), []
        self.closed = False
        self.stop_ack = stop_ack
        self.block = None

    def setsockopt(self, *args): pass
    def settimeout(self, *args): pass
    def getpeername(self): return ('192.168.4.1', 8888)

    def feed(self, obj):
        self.rx.put(json.dumps(obj).encode() + b'\n')

    def recv(self, size):
        import socket
        try:
            return self.rx.get(timeout=0.1)
        except queue.Empty:
            raise socket.timeout()

    def sendall(self, data):
        if self.block:
            self.block.wait(0.5)
        if self.closed:
            raise OSError('closed')
        for line in data.splitlines():
            obj = json.loads(line)
            self.sent.append(obj)
            if obj.get('cmd') == 'PING':
                self.feed({'ack': 'PING', 'status': 0, 't': obj['t']})
            if self.stop_ack and obj.get('cmd') == 'STOP_ALL':
                self.feed({'ack': 'STOP_ALL', 'status': 0})

    def shutdown(self, *_):
        self.closed = True
        self.rx.put(b'')
        if self.block:
            self.block.set()

    def close(self): self.shutdown()


class ClientTests(unittest.TestCase):
    def setUp(self):
        self.client = RoverClient()

    def tearDown(self):
        self.client.close()

    def connect(self, tcp=None, gen=7):
        tcp = tcp or FakeTCP()
        with patch('rover_comm.client.socket.create_connection', return_value=tcp):
            self.client.connect('test', udp_port=None)
        if gen is not None:
            tcp.feed({'type': 'sensor', 'gen': gen})
            self.assertTrue(self.client.wait_ready(1))
        return tcp

    def test_heartbeat_uses_firmware_ping(self):
        tcp = self.connect()
        self.assertTrue(eventually(lambda: any(o.get('cmd') == 'PING' for o in tcp.sent)))
        self.assertFalse(any(o.get('type') == 'heartbeat' for o in tcp.sent))
        self.assertTrue(eventually(lambda: self.client.last_rtt_ms is not None))

    def test_stop_bypasses_full_queue_and_invalidates_old_work(self):
        tcp = self.connect()
        session = self.client._session
        with session.condition:
            for _ in range(64):
                session.pending.append(b'{"cmd":"MOTOR"}\n')
            self.assertFalse(self.client.send_json({'cmd': 'PID'}))
            self.assertTrue(self.client.stop_all())
            self.assertEqual(len(session.pending), 1)
            self.assertEqual(json.loads(session.pending[0])['cmd'], 'STOP_ALL')
        self.assertFalse(self.client.send_json({'cmd': 'MOTOR'}))
        tcp.feed({'type': 'sensor', 'gen': 7})
        time.sleep(0.02)
        self.assertIsNone(self.client.generation)
        tcp.feed({'type': 'sensor', 'gen': 8})
        self.assertTrue(self.client.wait_ready(1))
        self.assertTrue(self.client.send_json({'cmd': 'MOTOR'}))

    def test_stop_requires_new_matching_ack_not_old_success(self):
        tcp = self.connect()
        tcp.feed({'type': 'sensor', 'gen': 7, 'cmd_ack': {'id': 3, 'status': 0}})
        tcp.feed({'ack': 'STOP_ALL', 'status': 0})
        self.assertTrue(eventually(lambda: self.client._session.stop_acks == 1))
        self.assertFalse(self.client.stop_and_wait(timeout=0.02, attempts=2))
        tcp.stop_ack = True
        self.assertTrue(self.client.stop_and_wait(timeout=0.5))

    def test_reconnect_has_no_old_workers_queue_or_generation(self):
        old_tcp = self.connect()
        old = self.client._session
        new_tcp = self.connect(gen=None)
        self.assertIsNone(self.client.generation)
        self.assertFalse(self.client.send_json({'cmd': 'MOTOR_CLOSED'}))
        self.client._close_session(old)
        self.assertTrue(self.client.connected)
        self.assertTrue(all(not t.is_alive() for t in old.threads))
        self.assertFalse(any(o.get('cmd') == 'MOTOR_CLOSED' for o in new_tcp.sent))
        self.assertTrue(old_tcp.closed)

    def test_malformed_json_and_telemetry_do_not_kill_rx(self):
        tcp = self.connect()
        received = []
        self.client.on_sensor = received.append
        tcp.rx.put(b'[]\nnull\n{"type":"sensor","imu":{"quat":[1]}}\n')
        tcp.feed({'type': 'sensor', 'gen': 9, 'link_stale': True})
        self.assertTrue(eventually(lambda: len(received) == 1))
        self.assertTrue(received[0].link_stale)
        self.assertTrue(self.client.connected)

    def test_unterminated_oversized_json_closes_connection(self):
        tcp = self.connect()
        tcp.rx.put(b'x' * 16385)
        self.assertTrue(eventually(lambda: not self.client.connected))

    def test_callback_exception_is_contained(self):
        tcp = self.connect()
        self.client.on_sensor = Mock(side_effect=RuntimeError('callback failure'))
        with self.assertLogs('rover_comm.client', level='ERROR'):
            tcp.feed({'type': 'sensor', 'gen': 8})
            self.assertTrue(eventually(lambda: self.client.on_sensor.called))
        self.assertTrue(self.client.connected)

    def test_batch_is_all_or_none_and_disallows_nan(self):
        self.connect()
        self.assertFalse(self.client.send_json_batch([{'cmd': 'PID'}], expected_session=object()))
        with self.assertRaises(ValueError):
            self.client.send_json_batch([{'cmd': 'PID', 'kp': float('nan')}])
        with self.assertRaises(ValueError):
            self.client.send_json_batch([{'cmd': 'PID'}] * 5)

    def test_bind_failure_closes_both_sockets(self):
        tcp, udp = FakeTCP(), Mock()
        udp.bind.side_effect = OSError('port busy')
        with patch('rover_comm.client.socket.create_connection', return_value=tcp), \
                patch('rover_comm.client.socket.socket', return_value=udp):
            with self.assertRaises(ConnectionError):
                self.client.connect('test')
        self.assertTrue(tcp.closed)
        udp.close.assert_called_once()

    def test_close_cancels_inflight_connect(self):
        entered, release = threading.Event(), threading.Event()
        tcp = FakeTCP()
        errors = []
        def create(*a, **kw):
            entered.set()
            release.wait(1)
            return tcp
        def connect():
            try:
                self.client.connect('test', udp_port=None)
            except ConnectionError as exc:
                errors.append(exc)
        with patch('rover_comm.client.socket.create_connection', side_effect=create):
            thread = threading.Thread(target=connect)
            thread.start()
            self.assertTrue(entered.wait(1))
            self.client.close()
            release.set()
            thread.join(1)
        self.assertFalse(self.client.connected)
        self.assertTrue(errors)
        self.assertTrue(tcp.closed)


class VideoTests(unittest.TestCase):
    @staticmethod
    def header(fid, packet, total):
        return VideoHeader(VIDEO_MAGIC, fid, packet, total)

    def test_changed_total_cannot_crash_or_mix_frame(self):
        asm = FrameAssembler()
        asm.push(self.header(1, 0, 3), JPEG[:10])
        asm.push(self.header(1, 2, 3), JPEG[20:])
        self.assertIsNone(asm.push(self.header(1, 0, 2), b'bad'))
        self.assertEqual(asm.push(self.header(1, 1, 3), JPEG[10:20]), JPEG)
        self.assertEqual(asm.packets_bad, 1)

    def test_late_first_packet_does_not_replace_new_frame(self):
        asm = FrameAssembler()
        asm.push(self.header(10, 0, 2), JPEG[:10])
        asm.push(self.header(9, 0, 2), b'old')
        self.assertEqual(asm.push(self.header(10, 1, 2), JPEG[10:]), JPEG)
        self.assertEqual(asm.frames_ok, 1)

    def test_wrap_duplicates_bounds_and_aborted_count(self):
        asm = FrameAssembler()
        self.assertEqual(asm.push(self.header(65535, 0, 1), JPEG), JPEG)
        self.assertIsNone(asm.push(self.header(65535, 0, 1), JPEG))
        self.assertEqual(asm.push(self.header(0, 0, 1), JPEG), JPEG)
        self.assertIsNone(asm.push(self.header(1, 0, 65535), b'x'))
        asm.push(self.header(1, 0, 2), b'x')
        asm.push(self.header(2, 0, 2), b'y')
        self.assertEqual(asm.frames_aborted, 1)

    def test_marker_guard_requires_sof_and_checks_multiscan(self):
        self.assertTrue(jpeg_structure_ok(JPEG))
        self.assertFalse(jpeg_structure_ok(b'\xff\xd8' + JPEG[15:]))
        self.assertFalse(jpeg_structure_ok(JPEG + b'junk'))
        self.assertFalse(jpeg_structure_ok(JPEG[:-1]))
        self.assertTrue(jpeg_structure_ok(JPEG[:-2] + JPEG[15:]))

    def test_missing_distance_is_invalid_and_bad_vectors_rejected(self):
        self.assertNotEqual(_sensor_from_json({}).dist_front_status, 0)
        for bad in ({'imu': None}, {'imu': {'quat': [1]}}, {'wheels': {'odometer': [0, 0, 0, None]}}):
            with self.assertRaises(ValueError):
                _sensor_from_json(bad)


class SDKTests(unittest.TestCase):
    def setUp(self):
        _stop_requested.clear()
        self.temp = tempfile.TemporaryDirectory()
        self.env = patch.dict(os.environ, PEO_CONFIG_PATH=str(Path(self.temp.name) / 'config.json'))
        self.env.start()
        self.original = peo._bridge.client
        self.fake = Mock(connected=True, sys_info=None, last_rtt_ms=None)
        self.fake.wait_ready.return_value = True
        self.fake.send_json.return_value = True
        self.fake.send_json_batch.return_value = True
        self.fake.stop_and_wait.return_value = True
        peo._bridge.client = self.fake
        peo._bridge._connected = True

    def tearDown(self):
        peo.cam._reset()
        peo._bridge._connected = False
        peo._bridge._snap = None
        peo._bridge.client = self.original
        _stop_requested.clear()
        self.env.stop()
        self.temp.cleanup()

    def test_public_fresh_is_bool_and_honors_s3_stale(self):
        peo._bridge._on_sensor(SensorData())
        self.assertIs(peo.fresh, True)
        peo._bridge._on_sensor(SensorData(link_stale=True))
        self.assertIs(peo.fresh, False)
        peo._bridge._connected = False
        self.assertIs(peo.fresh, False)
        self.assertIsNone(peo.vl53.front_mm)

    def test_stop_is_callable_after_cancel_and_timeout_is_visible(self):
        peo._request_stop()
        self.fake.stop_all.assert_called_once()
        peo.stop()
        self.fake.stop_and_wait.return_value = False
        with self.assertRaises(TimeoutError):
            peo.stop()
        with self.assertRaises(peo.ScriptStopped):
            peo.drive_all(20)

    def test_repeated_targets_are_resent_and_all_validated_first(self):
        peo.drive_all(20)
        peo.stop()
        peo.drive_all(20)
        self.assertEqual(self.fake.send_json_batch.call_count, 2)
        for bad in (float('nan'), float('inf'), 1e100, True):
            with self.assertRaises(ValueError):
                peo.drive(0, 10, 1, bad)
        self.assertEqual(self.fake.send_json_batch.call_count, 2)

    def test_camera_zero_timeout_stale_and_off_reset(self):
        peo.cam.on()
        peo.cam._on_frame(1, JPEG)
        self.assertEqual(peo.cam.read_raw(0), (JPEG, 1))
        self.assertEqual(peo.cam._latest_raw(0), (JPEG, 1))
        peo.cam._last_t -= 2
        self.assertEqual(peo.cam.read_raw(0), (None, -1))
        peo.cam.off()
        peo.cam.on()
        self.assertEqual(peo.cam.read_raw(0), (None, -1))

    def test_reject_bad_led_pid_and_resolution(self):
        for color in ((256, 0, 0), (True, 0, 0), (1.5, 0, 0)):
            with self.assertRaises(ValueError): peo.ws2812(*color)
        with self.assertRaises(ValueError): peo.ws2812(1, 2, 3, 356)
        with self.assertRaises(ValueError): peo.set_pid(kp=float('nan'))
        with self.assertRaises(TypeError): peo.set_pid(1, 2, kp=3)
        with self.assertRaises(ValueError): peo.cam.set_resolution('invalid')
        self.fake.send_json.assert_not_called()

    def test_failed_connect_and_passive_disconnect_release_lease(self):
        bridge = _Bridge()
        bridge.client = Mock(connected=False)
        bridge.client.connect.side_effect = ConnectionError('offline')
        bridge._lease = Mock()
        with self.assertRaises(ConnectionError): bridge.connect()
        self.assertTrue(bridge._lease.release.called)
        bridge._connected = True
        bridge._on_sensor(SensorData())
        bridge._on_disconnected()
        self.assertIsNone(bridge._snapshot())
        self.assertFalse(bridge.fresh)

    def test_config_corruption_and_tick_reset(self):
        bridge = _Bridge()
        Path(bridge._config_path()).write_text('[]')
        bridge._load_config()
        self.assertEqual(bridge._config, {})
        bridge._on_sensor(SensorData(sys_tick_ms=10000))
        bridge._recovered = True
        bridge._on_sensor(SensorData(sys_tick_ms=100))
        self.assertFalse(bridge._recovered)

    def test_config_monitor_reapplies_after_stale_recovery(self):
        bridge = _Bridge()
        bridge.client = Mock(connected=True)
        bridge.client.send_json_batch.return_value = True
        bridge._connected = True
        bridge._on_sensor(SensorData())
        bridge._start_monitor()
        try:
            self.assertTrue(eventually(lambda: bridge.client.send_json_batch.call_count == 1))
            bridge._on_sensor(SensorData(link_stale=True))
            self.assertTrue(eventually(lambda: not bridge._recovered))
            bridge._on_sensor(SensorData(sys_tick_ms=1))
            self.assertTrue(eventually(lambda: bridge.client.send_json_batch.call_count == 2))
        finally:
            bridge._monitor_stop.set()
            bridge._monitor_t.join(1)

    @unittest.skipUnless(importlib.util.find_spec('cv2') and importlib.util.find_spec('numpy'),
                         'vision dependencies unavailable')
    def test_small_real_jpeg_decode_is_cached_but_copies_are_independent(self):
        import cv2
        import numpy as np
        ok, data = cv2.imencode('.jpg', np.zeros((120, 160, 3), dtype=np.uint8))
        self.assertTrue(ok)
        raw = data.tobytes()
        self.assertLess(len(raw), 5000)
        self.assertTrue(jpeg_structure_ok(raw))
        peo.cam.on()
        peo.cam._on_frame(42, raw)
        with patch('cv2.imdecode', wraps=cv2.imdecode) as decode:
            first, seq = peo.cam.read(0)
            first[:] = 255
            second, _ = peo.cam.read(0)
            self.assertEqual(decode.call_count, 1)
        self.assertEqual(seq, 42)
        self.assertTrue(np.all(second == 0))


@unittest.skipUnless(importlib.util.find_spec('PySide6'), 'GUI dependencies unavailable')
class GUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        from app.gui import MainWindow
        _stop_requested.clear()
        self.window = MainWindow()

    def tearDown(self):
        self.window.close()
        _stop_requested.clear()

    def test_shell_busy_prevents_script_and_completion_uses_whole_matches(self):
        win = self.window
        win.ide_edit.setPlainText('pass')
        win._shell_busy.set()
        win._on_run_script()
        self.assertFalse(win._script_running)
        win._shell_busy.clear()
        win._shell_completer = Mock()
        win._shell_completer.complete.side_effect = ['print(', None]
        win.shell_input.setText('pri')
        win._shell_complete()
        self.assertEqual(win.shell_input.text(), 'print(')

    def test_output_queue_is_bounded_and_drain_has_budget(self):
        from app.gui import _StreamRelay
        win = self.window
        relay = _StreamRelay(win._shell_out_q)
        for _ in range(1000): relay.write('x' * 10000)
        self.assertLessEqual(win._shell_out_q.qsize(), 256)
        before = win._shell_out_q.qsize()
        win._drain_shell_out()
        self.assertEqual(win._shell_out_q.qsize(), before - 64)

    def test_video_mailbox_coalesces_notifications(self):
        win = self.window
        def producer():
            for i in range(100): win._queue_frame(i, JPEG)
        thread = threading.Thread(target=producer)
        thread.start(); thread.join()
        self.assertEqual(win._frame_mailbox[0], 99)
        self.assertTrue(win._frame_notification)
        # Discard the deliberately synthetic JPEG before Qt tries to decode it.
        win._frame_mailbox = None
        self.app.processEvents()
        self.assertFalse(win._frame_notification)


if __name__ == '__main__':
    unittest.main(verbosity=2)
