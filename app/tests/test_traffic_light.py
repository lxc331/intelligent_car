"""Traffic-light pause/resume boundaries without Tk, sockets or a vehicle."""
import copy
import math
import threading
import time
import types
import unittest

from carapp.config import DEFAULTS
from carapp.core.traffic_light import TrafficLightController, TrafficLightPolicy
from carapp.core.cruise import CruisePolicy, CruiseRunner, CruiseSettings, LaserMonitor
from carapp.core.teleop import TeleopController
from carapp.features.base import AppContext
from carapp.model.state import CarState


def wait_until(predicate, timeout=1.):
    end = time.monotonic() + timeout
    while not predicate() and time.monotonic() < end:
        time.sleep(.005)
    if not predicate():
        raise AssertionError("traffic-light worker did not reach expected state")


class TrafficLightPolicyTests(unittest.TestCase):
    def setUp(self):
        self.now = [10.]
        self.policy = TrafficLightPolicy(clock=lambda: self.now[0])
        self.policy.enabled = True

    def frame(self, at, red=False, error=""):
        self.now[0] = at
        self.policy.observe(red, at, error)
        return self.policy.snapshot()

    def clear(self, start=10.):
        for instant in (start, start + .4, start + .81):
            self.frame(instant)
        self.assertFalse(self.policy.snapshot()["blocked"])

    def test_disabled_does_not_restrict_motion(self):
        self.policy.enabled = False
        self.assertFalse(self.policy.snapshot()["blocked"])
        self.frame(10., red=True)
        self.assertFalse(self.policy.snapshot()["blocked"])

    def test_enable_waits_for_three_distinct_fresh_clear_frames_over_point_eight_seconds(self):
        self.assertTrue(self.policy.snapshot()["blocked"])
        self.assertTrue(self.frame(10.)["blocked"])
        self.assertTrue(self.frame(10.81)["blocked"])
        self.assertFalse(self.frame(10.9)["blocked"])

    def test_many_frames_before_clear_duration_do_not_release(self):
        for instant in (10., 10.1, 10.2, 10.7):
            self.assertTrue(self.frame(instant)["blocked"])
        self.assertFalse(self.frame(10.81)["blocked"])

    def test_first_red_stops_and_red_flicker_restarts_clear_confirmation(self):
        self.clear()
        self.assertTrue(self.frame(11., red=True)["blocked"])
        for instant in (11.2, 11.6):
            self.assertTrue(self.frame(instant)["blocked"])
        self.assertTrue(self.frame(11.7, red=True)["blocked"])
        for instant in (11.9, 12.2):
            self.assertTrue(self.frame(instant)["blocked"])
        self.assertFalse(self.frame(12.71)["blocked"])

    def test_error_is_a_hold_and_does_not_count_as_clear(self):
        self.clear()
        state = self.frame(11., error="broken image")
        self.assertTrue(state["blocked"])
        self.assertEqual(state["error"], "broken image")
        self.assertTrue(self.frame(11.2)["blocked"])
        self.assertTrue(self.frame(11.6)["blocked"])
        self.assertFalse(self.frame(12.01)["blocked"])

    def test_timeout_holds_and_new_clear_sequence_cannot_reuse_old_frames(self):
        self.clear()
        self.now[0] = 12.1
        self.assertTrue(self.policy.snapshot()["blocked"])
        self.assertTrue(self.frame(12.2)["blocked"])
        self.assertTrue(self.frame(12.6)["blocked"])
        self.assertFalse(self.frame(13.01)["blocked"])

    def test_gap_is_detected_even_when_no_snapshot_was_requested(self):
        self.clear()
        self.assertTrue(self.frame(12.2)["blocked"])
        self.assertTrue(self.frame(12.6)["blocked"])
        self.assertFalse(self.frame(13.01)["blocked"])

    def test_old_duplicate_future_and_expired_frames_cannot_clear_red(self):
        self.frame(10., red=True)
        for timestamp in (10., 9., 12., 8.):
            self.now[0] = 10.9
            self.policy.observe(False, timestamp)
            self.assertTrue(self.policy.snapshot()["blocked"])
        self.assertEqual(self.policy.last_at, 10.)


class TrafficLightWorkerTests(unittest.TestCase):
    def controller(self, detector=None):
        self.now = [10.]
        self.stopped = threading.Event()
        self.calls = []
        def detect(payload, mode):
            self.calls.append((payload, mode))
            if payload == b"bad":
                raise ValueError("bad image")
            return types.SimpleNamespace(red=payload == b"red")
        control = TrafficLightController(self.stopped.set,
            detector=detector or types.SimpleNamespace(detect=detect), clock=lambda: self.now[0])
        self.addCleanup(control.close)
        return control

    @staticmethod
    def last_at(control):
        with control.lock:
            return control.policy.last_at

    def send(self, control, at, payload=b"clear", handler=None):
        self.now[0] = at
        if handler:
            handler(at, payload, "jpeg")
        else:
            control.submit(at, payload, "jpeg")
        wait_until(lambda: self.last_at(control) == at)

    def clear(self, control, start=10., handler=None):
        for instant in (start, start + .4, start + .81):
            self.send(control, instant, handler=handler)
        self.assertFalse(control.snapshot()["blocked"])

    def test_preexisting_camera_handler_survives_toggle_and_no_ui_tick_is_needed(self):
        control = self.controller()
        handler = control.frame_handler()
        control.set_enabled(True)
        self.assertTrue(self.stopped.wait(.5))
        self.clear(control, handler=handler)
        self.stopped.clear()
        self.send(control, 11.2, b"red", handler)
        self.assertTrue(control.snapshot()["blocked"])
        self.assertTrue(self.stopped.wait(.5))

    def test_disabled_does_no_decoding_and_reset_rejects_old_camera_frames(self):
        control = self.controller()
        old_handler = control.frame_handler()
        old_handler(10., b"red", "jpeg")
        self.assertEqual(self.calls, [])
        control.set_enabled(True)
        self.clear(control, handler=old_handler)
        control.reset()
        self.now[0] = 11.5
        old_handler(11.5, b"clear", "jpeg")
        self.assertIsNone(self.last_at(control))
        self.assertTrue(control.snapshot()["blocked"])
        self.clear(control, start=12., handler=control.frame_handler())

    def test_decode_failure_keeps_hold_until_valid_clear_sequence(self):
        control = self.controller()
        control.set_enabled(True)
        self.clear(control)
        self.send(control, 11.1, b"bad")
        self.assertTrue(control.snapshot()["blocked"])
        self.assertEqual(control.snapshot()["error"], "bad image")
        self.clear(control, start=11.4)

    def test_timeout_emits_stop_without_new_frames_or_ui_ticks(self):
        control = self.controller()
        control.set_enabled(True)
        self.clear(control)
        self.stopped.clear()
        self.now[0] = 12.1
        self.assertTrue(self.stopped.wait(.5))
        self.assertTrue(control.snapshot()["blocked"])

    def test_slow_decoder_cannot_delay_independent_watchdog_timeout_stop(self):
        entered, release = threading.Event(), threading.Event()
        def detect(payload, mode):
            if payload == b"slow":
                entered.set()
                release.wait(2.)
            return types.SimpleNamespace(red=False)
        control = self.controller(types.SimpleNamespace(detect=detect))
        control.set_enabled(True)
        self.clear(control)
        try:
            self.now[0] = 11.1
            control.submit(11.1, b"slow", "jpeg")
            self.assertTrue(entered.wait(.5))
            self.stopped.clear()
            self.now[0] = 12.4
            self.assertTrue(self.stopped.wait(.5))
            self.assertTrue(control.snapshot()["blocked"])
            self.assertEqual(self.last_at(control), 10.81)
            self.assertFalse(release.is_set())
        finally:
            release.set()

    def test_close_stops_watchdog_without_later_stop_or_status_callbacks(self):
        now, stops, statuses = [10.], [], []
        control = TrafficLightController(lambda: stops.append(now[0]), statuses.append,
            detector=types.SimpleNamespace(detect=lambda payload, mode: types.SimpleNamespace(red=False)),
            clock=lambda: now[0])
        self.addCleanup(control.close)
        control.set_enabled(True)
        wait_until(lambda: bool(stops))
        control.close()
        self.assertFalse(control.watchdog.is_alive())
        before = len(stops), len(statuses)
        now[0] += 10.
        control.wake.set()
        time.sleep(.25)
        self.assertEqual((len(stops), len(statuses)), before)

    def test_in_flight_old_result_is_discarded_after_toggle_or_camera_reset(self):
        for operation in ("toggle", "reset"):
            with self.subTest(operation=operation):
                entered, release = threading.Event(), threading.Event()
                def detect(payload, mode):
                    entered.set()
                    release.wait(1.)
                    return types.SimpleNamespace(red=False)
                control = self.controller(types.SimpleNamespace(detect=detect))
                control.set_enabled(True)
                control.submit(10., b"old", "jpeg")
                self.assertTrue(entered.wait(.5))
                if operation == "toggle":
                    control.set_enabled(False)
                    control.set_enabled(True)
                else:
                    control.reset()
                release.set()
                wait_until(lambda: not control.thread.is_alive() or control.pending is None)
                time.sleep(.03)
                self.assertIsNone(self.last_at(control))
                self.assertTrue(control.snapshot()["blocked"])
                control.close()

    def test_close_rejects_late_in_flight_detection_and_new_frames(self):
        entered, release = threading.Event(), threading.Event()
        def detect(payload, mode):
            entered.set()
            release.wait(2.)
            return types.SimpleNamespace(red=True)
        control = self.controller(types.SimpleNamespace(detect=detect))
        control.set_enabled(True)
        control.submit(10., b"red", "jpeg")
        self.assertTrue(entered.wait(.5))
        closer = threading.Thread(target=control.close)
        closer.start()
        wait_until(control.halt.is_set)
        release.set()
        closer.join(1.)
        self.assertFalse(closer.is_alive())
        self.assertFalse(control.thread.is_alive())
        self.assertFalse(control.snapshot()["enabled"])
        self.assertIsNone(self.last_at(control))
        control.submit(10.5, b"red", "jpeg")
        self.assertIsNone(control.pending)


class TrafficLightMotionGateTests(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.state = CarState()
        self.state.update(ws_connected=True)
        self.control = TrafficLightController(lambda: None,
            detector=types.SimpleNamespace(detect=lambda payload, mode: types.SimpleNamespace(red=False)))
        self.addCleanup(self.control.close)
        self.app = types.SimpleNamespace(cfg=copy.deepcopy(DEFAULTS), state=self.state,
            bridge=types.SimpleNamespace(publish=self.publish), traffic_light=self.control,
            log=lambda message: None)
        self.ctx = AppContext(self.app)

    def publish(self, topic, message):
        self.sent.append((topic, message))
        return True

    def values(self):
        message = self.sent[-1][1]
        return message["linear"]["x"], message["linear"]["y"], message["angular"]["z"]

    def make_clear(self):
        now = time.monotonic()
        with self.control.lock:
            for stamp in (now - .9, now - .4, now):
                self.control.policy.observe(False, stamp)

    def test_all_axes_are_replaced_with_zero_and_no_stale_command_is_replayed(self):
        self.control.set_enabled(True)
        self.assertTrue(self.ctx.publish_twist(.1, .2, vy=.03))
        self.assertEqual(self.values(), (0., 0., 0.))
        self.assertEqual(self.state.get("base_cmd"), (0., 0.))
        self.assertEqual(self.state.get("base_vy"), 0.)
        self.make_clear()
        count = len(self.sent)
        self.control._maintain()
        self.assertEqual(len(self.sent), count)
        self.assertTrue(self.ctx.publish_twist(.1, .2, vy=.03))
        self.assertEqual(self.values(), (.1, .03, .2))

    def test_network_failure_is_propagated_even_when_replacing_with_zero(self):
        self.control.set_enabled(True)
        self.app.bridge.publish = lambda topic, message: False
        self.assertFalse(self.ctx.publish_twist(.1, .2, vy=.03))

    def test_red_update_cannot_overtake_a_velocity_publication(self):
        self.control.set_enabled(True)
        self.make_clear()
        entered, release, red_received = threading.Event(), threading.Event(), threading.Event()
        original = self.publish
        def blocking_publish(topic, message):
            entered.set()
            release.wait(1.)
            return original(topic, message)
        self.app.bridge.publish = blocking_publish
        sending = threading.Thread(target=lambda: self.ctx.publish_twist(.1, .2))
        def observe_red():
            with self.control.lock:
                self.control.policy.observe(True, time.monotonic())
            red_received.set()
        receiving = threading.Thread(target=observe_red)
        try:
            sending.start()
            self.assertTrue(entered.wait(.5))
            receiving.start()
            self.assertFalse(red_received.wait(.03))
            release.set()
            sending.join(1.)
            receiving.join(1.)
            self.assertTrue(red_received.is_set())
            self.assertTrue(self.ctx.publish_twist(.1, .2))
            self.assertEqual(self.values(), (0., 0., 0.))
        finally:
            release.set()
            sending.join(1.)
            if receiving.ident is not None:
                receiving.join(1.)

    def test_disabling_preserves_normal_command_and_missing_service_is_compatible(self):
        self.assertFalse(self.ctx.motion_interlock()["enabled"])
        self.assertTrue(self.ctx.publish_twist(.1, -.2, vy=-.03))
        self.assertEqual(self.values(), (.1, -.03, -.2))
        del self.app.traffic_light
        self.assertFalse(self.ctx.motion_interlock()["blocked"])
        self.assertTrue(self.ctx.publish_twist(.1, -.2, vy=-.03))

    def test_held_teleop_resumes_but_explicit_stop_during_red_does_not(self):
        self.control.set_enabled(True)
        remote = TeleopController(self.ctx, autostart=False)
        self.addCleanup(remote.close)
        remote.press("w")
        self.assertEqual(self.values(), (0., 0., 0.))
        self.assertIn("w", remote.pressed)
        self.make_clear()
        remote.tick()
        self.assertGreater(self.values()[0], 0.)
        with self.control.lock:
            self.control.policy.observe(True, time.monotonic())
        remote.tick()
        self.assertEqual(self.values(), (0., 0., 0.))
        remote.stop()
        self.make_clear()
        remote.tick()
        self.assertFalse(remote.pressed)
        self.assertEqual(self.values(), (0., 0., 0.))


class TrafficLightCruiseTests(unittest.TestCase):
    @staticmethod
    def scan_message(sequence, *, left=4., target=None):
        ranges = [4.] * 400
        ranges[10] = left
        if target:
            ranges[target[0]] = target[1]
        return dict(header=dict(frame_id="lidar_frame", stamp=dict(sec=100, nanosec=sequence)),
            angle_min=0., angle_increment=math.tau / 400,
            range_min=.1, range_max=10., ranges=ranges)

    def test_all_modes_pause_without_cancel_and_resume_from_fresh_scan_and_new_pid(self):
        for mode in ("avoid", "follow", "guard"):
            with self.subTest(mode=mode):
                now, hold, sent = [100.], [False], []
                traffic = types.SimpleNamespace(
                    snapshot=lambda: dict(enabled=True, blocked=hold[0], reason="red"),
                    guard_motion=lambda publish: publish(hold[0]))
                app = types.SimpleNamespace(cfg=copy.deepcopy(DEFAULTS), state=CarState(),
                    log=lambda message: None, traffic_light=traffic,
                    bridge=types.SimpleNamespace(connected=True,
                        publish=lambda topic, message: sent.append(message) or True))
                app.state.update(ws_connected=True, cam_connected=True)
                monitor = LaserMonitor()
                monitor.update(self.scan_message(1,
                    left=.4 if mode == "avoid" else 4.,
                    target=(25, .45) if mode != "avoid" else None), now[0])
                runner = CruiseRunner(AppContext(app), monitor, clock=lambda: now[0])
                settings = CruiseSettings(mode=mode)
                runner.configure(settings)
                runner.running = True  # Exercise deterministic steps, without a timer thread.
                try:
                    runner.step()
                    self.assertTrue(any((sent[-1]["linear"]["x"], sent[-1]["angular"]["z"])))
                    previous_algorithm = runner.policy.algorithm
                    hold[0] = True
                    for instant in (100.2, 101.1):
                        now[0] = instant
                        runner.step()
                        self.assertTrue(runner.running)
                        self.assertFalse(runner.cancel.is_set())
                        self.assertTrue(runner.policy.waiting_for_scan)
                        self.assertEqual((sent[-1]["linear"]["x"], sent[-1]["angular"]["z"]), (0., 0.))
                    hold[0] = False
                    runner.step()
                    self.assertTrue(runner.running)
                    self.assertTrue(runner.policy.waiting_for_scan)
                    self.assertEqual(sent[-1]["linear"]["x"], 0.)
                    now[0] = 101.2
                    monitor.update(self.scan_message(2,
                        target=(375, .4) if mode != "avoid" else None), now[0])
                    expected = CruisePolicy().decide(monitor.snapshot(), settings, now[0])
                    runner.step()
                    self.assertTrue(runner.running)
                    self.assertFalse(runner.cancel.is_set())
                    self.assertFalse(runner.policy.waiting_for_scan)
                    self.assertIsNot(runner.policy.algorithm, previous_algorithm)
                    self.assertEqual((sent[-1]["linear"]["x"], sent[-1]["angular"]["z"]), expected[:2])
                    if mode == "avoid":
                        self.assertEqual(expected[:2], (.06, 0.))
                    else:
                        self.assertLess(expected[1], 0.)
                finally:
                    runner.stop()


if __name__ == "__main__":
    unittest.main()
