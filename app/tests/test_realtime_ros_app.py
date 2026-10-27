"""App reception keeps lidar out of the Tk queue, using only fake transports."""
import copy
import math
import queue
import threading
import unittest
from unittest.mock import Mock, patch

from carapp.config import DEFAULTS
from carapp.core.laser_scan import LaserMonitor
from carapp.core.realtime_ros import RealtimeRosDelivery
from carapp.features.base import AppContext
from carapp.features.cruise import CruiseFeature
from carapp.features.mapping import MappingFeature
from carapp.main import App


def message(sequence=1):
    return dict(header=dict(frame_id='lidar_frame', stamp=dict(sec=100, nanosec=sequence)),
                angle_min=0., angle_increment=math.tau / 400,
                range_min=.1, range_max=10., ranges=[4.] * 400)


class RealtimeRosAppTests(unittest.TestCase):
    def setUp(self):
        self.app = App.__new__(App)
        self.app._closing = False
        self.app._connection_generation = 1
        self.app._ui_queue = queue.Queue()
        self.app._realtime_ros = RealtimeRosDelivery()
        self.monitor = LaserMonitor()
        self.app._realtime_ros.begin(1, {'/scan_raw': (self.monitor.update,)},
                                      (self.monitor.reset,))
        self.app._realtime_ros.set_connected(1, True)

    def test_lidar_receipt_runs_on_the_receiving_thread_without_queue_drain(self):
        pending_ui_callback = Mock()
        self.app._post(pending_ui_callback)
        finished, error = threading.Event(), []

        def receive():
            try:
                self.app._receive_ros_message(1, '/scan_raw', message())
            except BaseException as exc:
                error.append(exc)
            finally:
                finished.set()

        receiver = threading.Thread(target=receive)
        receiver.start()
        self.assertTrue(finished.wait(1))
        receiver.join(1)
        self.assertEqual(error, [])
        self.assertEqual(self.monitor.snapshot()['sequence'], 1)
        self.assertEqual(self.app._ui_queue.qsize(), 1)
        pending_ui_callback.assert_not_called()

    def test_receive_timestamp_is_captured_before_callback_work(self):
        with patch('carapp.main.time.monotonic', return_value=123.45):
            self.app._receive_ros_message(1, '/scan_raw', message())
        self.assertEqual(self.monitor.snapshot()['received_at'], 123.45)
        self.assertTrue(self.app._ui_queue.empty())

    def test_non_realtime_telemetry_stays_on_ui_queue(self):
        payload = {'voltage': 8000}
        with patch('carapp.main.time.monotonic', return_value=123.45):
            self.app._receive_ros_message(1, '/ros_robot_controller/battery', payload)
        callback, args = self.app._ui_queue.get_nowait()
        self.assertEqual(callback, self.app._session_callback)
        self.assertEqual(args, (1, self.app._on_ros_message,
                                '/ros_robot_controller/battery', payload, 123.45))
        self.assertEqual(self.monitor.snapshot()['sequence'], 0)

    def test_stale_transport_never_changes_current_monitor_or_queues_lidar(self):
        self.app._realtime_ros.begin(2, {'/scan_raw': (self.monitor.update,)},
                                      (self.monitor.reset,))
        self.app._realtime_ros.set_connected(2, True)
        self.app._receive_ros_message(2, '/scan_raw', message(sequence=2))
        expected = self.monitor.snapshot()
        self.app._receive_ros_message(1, '/scan_raw', message(sequence=3))
        self.app._receive_ros_status(1, False, 'old socket closed')
        self.assertEqual(self.monitor.snapshot(), expected)
        self.assertTrue(self.app._ui_queue.empty())

    def test_disconnect_clears_scan_before_ui_status_is_processed(self):
        self.app._receive_ros_message(1, '/scan_raw', message())
        self.app._receive_ros_status(1, False, 'disconnected')
        self.assertEqual(self.monitor.snapshot()['sequence'], 0)
        callback, args = self.app._ui_queue.get_nowait()
        self.assertEqual(callback, self.app._session_callback)
        self.assertEqual(args, (1, self.app._on_ros_status, False, 'disconnected'))
        self.app._receive_ros_message(1, '/scan_raw', message(sequence=2))
        self.assertEqual(self.monitor.snapshot()['sequence'], 0)

    def test_closing_app_ignores_both_message_and_status(self):
        self.app._closing = True
        self.app._receive_ros_message(1, '/scan_raw', message())
        self.app._receive_ros_message(1, '/odom', {})
        self.app._receive_ros_status(1, True)
        self.assertEqual(self.monitor.snapshot()['sequence'], 0)
        self.assertTrue(self.app._ui_queue.empty())

    def test_start_bridge_wires_both_real_feature_consumers_and_disconnect_resets_them(self):
        class FakeBridge:
            def __init__(self, host, port, **callbacks):
                self.host, self.port, self.callbacks = host, port, callbacks
                self.connected = False

            def start(self):
                self.connected = True
                self.callbacks['on_status'](True, 'simulated transport')

            def stop(self):
                self.connected = False
                self.callbacks['on_status'](False, 'simulated disconnect')

        self.app.cfg = copy.deepcopy(DEFAULTS)
        self.app.bridge = None
        ctx = AppContext(self.app)
        cruise, mapping = CruiseFeature(ctx), MappingFeature(ctx)
        self.app.features = [cruise, mapping]
        try:
            with patch('carapp.main.RosBridge', FakeBridge):
                self.app._start_bridge()
            self.assertEqual(self.app.bridge.host, self.app.cfg['car']['host'])
            self.assertEqual(self.app.bridge.port, self.app.cfg['car']['ws_port'])
            self.assertTrue(self.app._realtime_ros.handles('/scan_raw'))
            # The transport status is intentionally left unprocessed by Tk.
            self.assertEqual(self.app._ui_queue.qsize(), 1)
            self.app._ui_queue.get_nowait()
            with patch('carapp.main.time.monotonic', return_value=123.45):
                self.app.bridge.callbacks['on_message']('/scan_raw', message())
            self.assertEqual(cruise.monitor.snapshot()['sequence'], 1)
            self.assertEqual(cruise.monitor.snapshot()['received_at'], 123.45)
            self.assertEqual(mapping._scan_at, 123.45)
            self.assertTrue(self.app._ui_queue.empty())
            self.app.bridge.stop()
            self.assertEqual(cruise.monitor.snapshot()['sequence'], 0)
            self.assertEqual(mapping._scan_at, 0.)
            self.assertEqual(self.app._ui_queue.qsize(), 1)
        finally:
            self.app._invalidate_ros_data()
            mapping.feed.close()
            mapping.feed._worker.join(1)


if __name__ == '__main__':
    unittest.main()
