"""Latest-scan delivery and session races; no Tk, sockets, or vehicle access."""
import math
import threading
import unittest
from unittest.mock import patch

from carapp.core.cruise import CruisePolicy, CruiseSettings
from carapp.core.laser_scan import LaserMonitor
from carapp.core.realtime_ros import RealtimeRosDelivery


def message(sequence=1, distance=4.):
    ranges = [4.] * 400
    ranges[10] = distance
    return dict(header=dict(frame_id='lidar_frame', stamp=dict(sec=100, nanosec=sequence)),
                angle_min=0., angle_increment=math.tau / 400,
                range_min=.1, range_max=10., ranges=ranges)


class RealtimeScanDeliveryTests(unittest.TestCase):
    def setUp(self):
        self.monitor = LaserMonitor()
        self.delivery = RealtimeRosDelivery()
        self.handlers = {'/scan_raw': (self.monitor.update,)}
        self.delivery.begin(1, self.handlers, (self.monitor.reset,))
        self.delivery.set_connected(1, True)

    def send(self, sequence=1, received_at=100., distance=4., generation=1):
        return self.delivery.deliver(generation, '/scan_raw',
                                     message(sequence, distance), received_at)

    def test_latest_scan_is_available_without_an_interface_queue(self):
        for sequence in range(1, 101):
            self.assertTrue(self.send(sequence, received_at=100. + sequence / 1000))
        latest = self.monitor.snapshot()
        self.assertEqual(latest['sequence'], 100)
        self.assertAlmostEqual(latest['received_at'], 100.1)
        self.assertEqual(latest['ranges'][10], 4.)
        self.assertTrue(self.delivery.handles('/scan_raw'))
        self.assertFalse(self.delivery.handles('/odom'))

    def test_waiting_connection_and_unregistered_topic_do_not_deliver(self):
        self.delivery.begin(2, self.handlers, (self.monitor.reset,))
        self.assertFalse(self.send(generation=2))
        self.delivery.set_connected(2, True)
        self.assertFalse(self.delivery.deliver(2, '/odom', message(), 100.))
        self.assertEqual(self.monitor.snapshot()['sequence'], 0)

    def test_old_generation_cannot_update_or_clear_the_new_vehicle(self):
        self.send()
        self.delivery.begin(2, self.handlers, (self.monitor.reset,))
        self.delivery.set_connected(2, True)
        self.assertTrue(self.send(sequence=2, received_at=102., generation=2))
        expected = self.monitor.snapshot()
        self.assertFalse(self.send(sequence=3, received_at=103., generation=1))
        self.delivery.set_connected(1, False)
        self.delivery.set_connected(1, True)
        self.assertEqual(self.monitor.snapshot(), expected)

    def test_disconnect_clears_scan_and_reconnect_accepts_repeated_car_timestamp(self):
        self.send()
        self.delivery.set_connected(1, False)
        self.assertEqual(self.monitor.snapshot()['sequence'], 0)
        self.assertFalse(self.send(sequence=2))
        self.delivery.set_connected(1, True)
        # A restarted publisher may begin with the same timestamp as before.
        self.assertTrue(self.send(received_at=102.))
        self.assertEqual(self.monitor.snapshot()['received_at'], 102.)

    def test_duplicate_connected_notification_does_not_erase_fresh_data(self):
        self.send()
        expected = self.monitor.snapshot()
        self.delivery.set_connected(1, True)
        self.assertEqual(self.monitor.snapshot(), expected)

    def test_invalidate_clears_scan_and_rejects_all_old_callbacks(self):
        self.send()
        self.delivery.invalidate()
        self.assertEqual(self.monitor.snapshot()['sequence'], 0)
        self.assertFalse(self.send(sequence=2))
        self.delivery.set_connected(1, True)
        self.assertFalse(self.send(sequence=3))

    def test_switch_waits_for_inflight_update_then_resets_before_accepting_new_data(self):
        entered, release, switched = threading.Event(), threading.Event(), threading.Event()
        errors = []

        def slow_update(msg, received_at):
            entered.set()
            if not release.wait(2):
                raise AssertionError('test did not release the in-flight update')
            self.monitor.update(msg, received_at)

        self.delivery.begin(1, {'/scan_raw': (slow_update,)}, (self.monitor.reset,))
        self.delivery.set_connected(1, True)

        def updating():
            try:
                self.delivery.deliver(1, '/scan_raw', message(), 100.)
            except BaseException as exc:
                errors.append(exc)

        def switching():
            try:
                self.delivery.begin(2, self.handlers, (self.monitor.reset,))
                switched.set()
            except BaseException as exc:
                errors.append(exc)

        update_thread = threading.Thread(target=updating)
        switch_thread = threading.Thread(target=switching)
        try:
            update_thread.start()
            self.assertTrue(entered.wait(1))
            switch_thread.start()
            self.assertFalse(switched.wait(.05))
        finally:
            release.set()
            update_thread.join(2)
            if switch_thread.ident is not None:
                switch_thread.join(2)
        self.assertFalse(update_thread.is_alive())
        self.assertFalse(switch_thread.is_alive())
        self.assertEqual(errors, [])
        self.assertTrue(switched.is_set())
        self.assertEqual(self.monitor.snapshot()['sequence'], 0)
        self.delivery.set_connected(2, True)
        self.assertTrue(self.send(sequence=2, received_at=102., generation=2))
        self.assertEqual(self.monitor.snapshot()['received_at'], 102.)

    def test_skipped_intermediate_scans_do_not_replay_old_algorithm_updates(self):
        self.send()
        policy, settings = CruisePolicy(), CruiseSettings()
        self.assertEqual(policy.decide(self.monitor.snapshot(), settings, 100.)[:2], (.06, 0.))
        for sequence in range(2, 11):
            self.send(sequence, received_at=100.1, distance=.4 if sequence == 10 else 4.)
        latest = self.monitor.snapshot()
        with patch.object(policy.algorithm, 'update', wraps=policy.algorithm.update) as update:
            result = policy.decide(latest, settings, 100.1)
            policy.decide(latest, settings, 100.2)
        update.assert_called_once()
        self.assertIs(update.call_args.args[0], latest)
        self.assertEqual(result[:2], (.01, -.36))
        self.assertEqual(policy.last_sequence, latest['sequence'])


if __name__ == '__main__':
    unittest.main()
