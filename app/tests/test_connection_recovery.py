"""Local simulated servers only; never sends commands to either physical car."""
import base64
import copy
import http.server
import json
import queue
import socket
import socketserver
import struct
import threading
import time
import types
import unittest
from unittest.mock import patch

from carapp.config import DEFAULTS, normalize_connection_config
from carapp.core.camera import CameraStream
from carapp.core.connection import diagnose, require_wifi
from carapp.core.discovery import _looks_like_car
from carapp.core.rosbridge import RosBridge
from carapp.features.tracking_pick import TrackingPickControls
from carapp.main import App
from carapp.model.state import CarState


def wait_for(predicate, seconds=3):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(.01)
    return False


def read_exact(sock, count):
    data = b""
    while len(data) < count:
        part = sock.recv(count - len(data))
        if not part:
            raise EOFError()
        data += part
    return data


class WSHandler(socketserver.BaseRequestHandler):
    def handle(self):
        self.server.clients.append(self.request)
        self.request.settimeout(2)
        try:
            head = b""
            while b"\r\n\r\n" not in head:
                head += read_exact(self.request, 1)
            self.server.attempts += 1
            if self.server.reject:
                self.request.sendall(b"HTTP/1.1 503 Starting\r\n\r\n")
                return
            self.request.sendall(b"HTTP/1.1 101 Switching Protocols\r\n\r\n")
            while True:
                first, second = read_exact(self.request, 2)
                length = second & 127
                if length == 126:
                    length = struct.unpack(">H", read_exact(self.request, 2))[0]
                mask = read_exact(self.request, 4)
                payload = read_exact(self.request, length)
                payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
                if self.server.silent:
                    continue
                if first & 15 == 9:
                    self.request.sendall(bytes([0x8a, len(payload)]) + payload)
                elif first & 15 == 1:
                    self.server.messages.append(json.loads(payload))
        except (OSError, EOFError):
            pass


class WSServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self):
        super().__init__(("127.0.0.1", 0), WSHandler)
        self.reject = False
        self.silent = False
        self.messages = []
        self.clients = []
        self.attempts = 0


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.server = WSServer()
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.statuses = []
        self.bridge = RosBridge("127.0.0.1", self.server.server_address[1],
                                on_status=lambda value, detail: self.statuses.append(value))
        self.bridge.RETRY_SECONDS = .02
        self.bridge.HEARTBEAT_SECONDS = .04
        self.bridge.DEAD_SECONDS = .3
        self.bridge.WATCHDOG_SECONDS = .01

    def tearDown(self):
        self.bridge.stop()
        if self.bridge._thread:
            self.bridge._thread.join(2)
        for client in self.server.clients:
            try:
                client.shutdown(socket.SHUT_RDWR)
                client.close()
            except OSError:
                pass
        self.server.shutdown()
        self.server.server_close()

    def test_services_start_late_without_restarting_app(self):
        self.server.reject = True
        self.bridge.start()
        self.assertTrue(wait_for(lambda: self.server.attempts >= 2))
        self.assertFalse(self.bridge.connected)
        self.server.reject = False
        self.assertTrue(wait_for(lambda: self.bridge.connected))

    def test_reboot_resubscribes_without_replaying_movement(self):
        self.bridge.subscribe("/odom", 100)
        self.bridge.start()
        self.assertTrue(wait_for(lambda: len(self.server.messages) >= 1))
        self.bridge.publish("/controller/cmd_vel", {"linear": {"x": .06}})
        self.assertTrue(wait_for(lambda: len(self.server.messages) >= 2))
        self.server.clients[-1].shutdown(socket.SHUT_RDWR)
        self.server.clients[-1].close()
        self.assertTrue(wait_for(lambda: self.server.attempts >= 2 and self.bridge.connected))
        self.assertTrue(wait_for(lambda: sum(m["op"] == "subscribe" for m in self.server.messages) >= 2))
        self.assertEqual(sum(m["op"] == "publish" for m in self.server.messages), 1)
        self.assertIn(False, self.statuses)

    def test_silent_dead_connection_is_detected_and_retried(self):
        self.server.silent = True
        self.bridge.start()
        self.assertTrue(wait_for(lambda: self.bridge.connected))
        self.assertTrue(wait_for(lambda: self.server.attempts >= 2))
        self.assertIn(False, self.statuses)

    def test_pong_keeps_quiet_connection_alive(self):
        self.bridge.start()
        self.assertTrue(wait_for(lambda: self.bridge.connected))
        time.sleep(.5)
        self.assertEqual(self.server.attempts, 1)
        self.assertTrue(self.bridge.connected)

    def test_stop_during_handshake_does_not_reconnect(self):
        self.server.reject = True
        self.bridge.start()
        self.assertTrue(wait_for(lambda: self.server.attempts > 0))
        self.bridge.stop()
        self.bridge._thread.join(2)
        self.assertFalse(self.bridge._thread.is_alive())
        self.assertFalse(self.bridge.connected)


class CameraRecoveryTests(unittest.TestCase):
    def test_camera_retries_startup_failure_and_stream_eof(self):
        frame = base64.b64decode("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+jR1sAAAAASUVORK5CYII=")
        class Handler(http.server.BaseHTTPRequestHandler):
            def do_GET(self):
                self.server.attempts += 1
                if self.server.attempts == 1:
                    self.send_error(503)
                    return
                self.send_response(200)
                self.end_headers()
                self.wfile.write(frame)
            def log_message(self, *args):
                pass
        server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        server.attempts = 0
        threading.Thread(target=server.serve_forever, daemon=True).start()
        frames = queue.Queue(1)
        cam = CameraStream("127.0.0.1", server.server_address[1], "/image_raw", "png", frames)
        cam.RETRY_SECONDS = .02
        try:
            cam.start()
            self.assertTrue(wait_for(lambda: server.attempts >= 3 and not frames.empty()))
            self.assertEqual(frames.get()[1], frame)
            self.assertEqual(cam.size, (1, 1))
        finally:
            cam.stop()
            cam.join(2)
            server.shutdown()
            server.server_close()
        self.assertFalse(cam.is_alive())


class ProfileTests(unittest.TestCase):
    def test_old_config_gets_two_distinct_profiles(self):
        cfg = copy.deepcopy(DEFAULTS)
        cfg["cars"] = [{"name": "1 号车", "host": "192.168.150.3"},
                       {"name": "2 号车", "host": "192.168.150.2"}]
        cfg["car"]["host"] = "192.168.150.2"
        normalize_connection_config(cfg)
        self.assertEqual(cfg["car"]["profile_id"], "HW-A0857935")
        self.assertEqual([c["ssid"] for c in cfg["cars"]], ["HW-10CCA58E", "HW-A0857935"])

    def test_hotspot_address_alone_does_not_identify_car(self):
        cfg = copy.deepcopy(DEFAULTS)
        cfg["car"]["host"] = "192.168.149.1"
        normalize_connection_config(cfg)
        self.assertEqual(cfg["car"]["profile_id"], "")

    def test_wrong_or_unknown_ssid_blocks_connection(self):
        for current in ("HW-10CCA58E", "校园网", ""):
            with patch("carapp.core.connection.wifi_ssid", return_value=current):
                with self.assertRaises(ConnectionError):
                    require_wifi("HW-A0857935")
        with patch("carapp.core.connection.wifi_ssid", return_value="HW-A0857935"):
            require_wifi("HW-A0857935")
        with self.assertRaises(ConnectionError):
            require_wifi("")

    def test_hotspot_calibrations_are_separate(self):
        cfg = normalize_connection_config(copy.deepcopy(DEFAULTS))
        cfg["car"].update(host="192.168.149.1", transport="wifi")
        controls = TrackingPickControls.__new__(TrackingPickControls)
        controls.ctx = types.SimpleNamespace(cfg=cfg)
        controls.cfg = cfg["tracking_pick"]
        keys = []
        for profile in cfg["cars"]:
            cfg["car"]["profile_id"] = profile["id"]
            keys.append(controls.key())
        self.assertNotEqual(*keys)

    def test_partial_service_is_discoverable(self):
        with patch("carapp.core.discovery._probe", side_effect=lambda ip, port, **kw: port == 9091):
            self.assertTrue(_looks_like_car("127.0.0.1", 8081, 9091))

    def test_diagnostics_explains_missing_subnet(self):
        cfg = normalize_connection_config(copy.deepcopy(DEFAULTS))
        with patch("carapp.core.connection.local_addresses", return_value=[
                {"InterfaceAlias": "WLAN", "IPAddress": "172.27.0.84", "PrefixLength": 15}]), \
                patch("carapp.core.connection.ping_once", return_value=False):
            report = diagnose(cfg["car"], cfg["cars"])
        self.assertIn("没有目标网段", report)
        self.assertIn("192.168.150.100", report)

    def test_old_vehicle_callbacks_cannot_update_new_vehicle(self):
        app = App.__new__(App)
        app._closing, app._connection_generation = False, 2
        events = []
        app._session_callback(1, events.append, "old")
        app._session_callback(2, events.append, "current")
        self.assertEqual(events, ["current"])

    def test_recovery_stays_on_selected_vehicle_and_keeps_timer(self):
        app = App.__new__(App)
        app._closing, app._connection_notice = False, None
        app.state, app.log = CarState(), lambda text: None
        scheduled = []
        app.root = types.SimpleNamespace(after=lambda ms, cb: scheduled.append((ms, cb)))
        app.discover_car = lambda cb: self.fail("must not automatically switch vehicle")
        app._auto_recover()
        app.state.update(ws_connected=True, cam_connected=True)
        app._auto_recover()
        self.assertEqual(len(scheduled), 2)

    def test_disconnect_cancels_actions_and_clears_telemetry(self):
        app = App.__new__(App)
        app._closing, app.state, app.log = False, CarState(), lambda text: None
        app.state.update(ws_connected=True, battery_mv=8000, pose=(1, 2, 3), node_count=33)
        stopped = []
        app.features = [types.SimpleNamespace(stop_all=lambda: stopped.append(True), tracking=None)]
        app._on_ros_status(False, "power off")
        self.assertEqual(stopped, [True])
        self.assertIsNone(app.state.get("pose"))
        self.assertIsNone(app.state.get("battery_mv"))


if __name__ == "__main__":
    unittest.main()
