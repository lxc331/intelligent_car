"""Traffic context and opt-in speed limits without a vehicle or ROS."""
import copy
import math
import threading
import time
import types
import unittest

from carapp.config import DEFAULTS
from carapp.core.traffic_light import TrafficLightController
from carapp.core.traffic_light_detector import TrafficLightDetection, TrafficObject
from carapp.core.traffic_scene import TrafficScenePolicy, limit_twist
from carapp.features.base import AppContext
from carapp.model.state import CarState


def obj(label, box, score=.9):
    return TrafficObject(label, score, box)


WALK = obj("crosswalk", (260, 320, 120, 60))
RED = obj("red", (300, 60, 40, 40))


class TrafficSceneTests(unittest.TestCase):
    def activate(self, policy, objects=(WALK,), start=10.):
        for stamp in (start, start + .3, start + .6):
            policy.observe(objects, (640, 480), stamp)
        self.assertTrue(policy.snapshot()["crosswalk_active"])

    def test_red_outside_confirmed_crosswalk_does_not_trigger(self):
        policy = TrafficScenePolicy()
        for stamp in (10., 10.3, 10.6):
            self.assertFalse(policy.observe((RED,), (640, 480), stamp))
        self.assertIsNone(policy.snapshot()["speed_limit"])

    def test_three_consecutive_distinct_near_crosswalk_frames_required(self):
        policy = TrafficScenePolicy()
        self.assertFalse(policy.observe((WALK, RED), (640, 480), 10.))
        self.assertFalse(policy.observe((WALK, RED), (640, 480), 10.))
        self.assertEqual(policy.confirmed, 1)
        policy.observe((), (640, 480), 10.3)
        self.assertEqual(policy.confirmed, 0)
        self.assertFalse(policy.observe((WALK, RED), (640, 480), 10.6))
        self.assertFalse(policy.observe((WALK, RED), (640, 480), 10.9))
        self.assertTrue(policy.observe((WALK, RED), (640, 480), 11.2))
        self.assertEqual(policy.snapshot()["speed_limit"], .1)

    def test_far_and_side_crosswalks_do_not_open_a_junction(self):
        for walk in (obj("crosswalk", (260, 260, 120, 40)),
                     obj("crosswalk", (0, 320, 80, 60))):
            policy = TrafficScenePolicy()
            for stamp in (10., 10.3, 10.6):
                self.assertFalse(policy.observe((walk, RED), (640, 480), stamp))
            self.assertFalse(policy.active)

    def test_red_area_and_crosswalk_distance_normalize_across_resolutions(self):
        for scale in (.5, 1., 2.):
            def resized(item):
                return obj(item.label, tuple(value * scale for value in item.box))
            policy = TrafficScenePolicy()
            for stamp in (10., 10.3, 10.6):
                policy.observe((resized(WALK),), (640*scale, 480*scale), stamp)
            tiny = resized(obj("red", (300, 60, 20, 20)))
            self.assertFalse(policy.observe((tiny,), (640*scale, 480*scale), 10.9))
            self.assertTrue(policy.observe((resized(RED),), (640*scale, 480*scale), 11.2))
            self.assertTrue(policy.observe((tiny,), (640*scale, 480*scale), 11.5))

    def test_long_red_wait_does_not_expire_passage_and_clear_passage_expires(self):
        policy = TrafficScenePolicy()
        self.activate(policy)
        for index in range(1, 41):
            self.assertTrue(policy.observe((RED,), (640, 480), 10.6 + index*.5))
        self.assertEqual(policy.remaining, 4.)
        for index in range(1, 3):
            policy.observe((), (640, 480), 30.6 + index*.5, paused=True)
        self.assertEqual(policy.remaining, 4.)
        for index in range(1, 5):
            policy.observe((), (640, 480), 31.6 + index)
        self.assertFalse(policy.active)
        self.assertIsNone(policy.snapshot()["speed_limit"])

    def test_camera_gap_resets_previous_junction_before_new_red(self):
        policy = TrafficScenePolicy()
        self.activate(policy)
        self.assertFalse(policy.observe((RED,), (640, 480), 13.))
        self.assertFalse(policy.active)

    def test_park_approach_and_route_signs_only_report_and_limit_speed(self):
        policy = TrafficScenePolicy()
        policy.observe((obj("park", (300, 60, 40, 40)),
                        obj("crosswalk", (260, 180, 120, 40))), (640, 480), 10.)
        self.assertEqual(policy.snapshot()["speed_limit"], .1)
        self.assertIn("停车标志", policy.snapshot()["scene"])
        policy.reset()
        close = obj("crosswalk", (260, 360, 120, 40))
        right = obj("right", (300, 60, 40, 40))
        self.activate(policy, (close, right))
        self.assertIn("右转路口", policy.snapshot()["scene"])
        policy.reset()
        self.activate(policy, (WALK, obj("go", (300, 60, 40, 40))))
        self.assertIn("直行路口", policy.snapshot()["scene"])

    def test_speed_limit_preserves_lower_speeds_curvature_and_zero_commands(self):
        self.assertEqual(limit_twist(.06, 0., .3, .1), (.06, 0., .3))
        limited = limit_twist(.15, 0., .3, .1)
        self.assertAlmostEqual(limited[0], .1)
        self.assertAlmostEqual(limited[2], .2)
        limited = limit_twist(-.15, .15, -.3, .1)
        self.assertAlmostEqual(math.hypot(*limited[:2]), .1)
        self.assertLess(limited[0], 0.)
        self.assertGreater(limited[1], 0.)
        self.assertLess(limited[2], 0.)
        self.assertEqual(limit_twist(0., 0., .3, .1), (0., 0., .3))
        self.assertEqual(limit_twist(0., 0., 0., .1), (0., 0., 0.))
        self.assertEqual(limit_twist(.15, 0., .3, None), (.15, 0., .3))


class TrafficSceneSwitchTests(unittest.TestCase):
    def setUp(self):
        self.now = [10.]
        self.calls, self.stops, self.sent = [], [], []
        self.next = TrafficLightDetection(False, "", image_size=(640, 480))
        def detect(payload, mode):
            self.calls.append(payload)
            return self.next
        self.control = TrafficLightController(lambda: self.stops.append(True),
            detector=types.SimpleNamespace(detect=detect), clock=lambda: self.now[0])
        self.addCleanup(self.control.close)
        app = types.SimpleNamespace(cfg=copy.deepcopy(DEFAULTS), state=CarState(), traffic_light=self.control,
            bridge=types.SimpleNamespace(publish=lambda topic, message: self.sent.append(message) or True),
            log=lambda message: None)
        self.ctx = AppContext(app)

    def send(self, at, objects):
        self.now[0] = at
        self.next = TrafficLightDetection(any(o.label == "red" for o in objects), "",
                                         objects=objects, image_size=(640, 480))
        self.control.submit(at, b"frame", "jpeg")
        deadline = time.monotonic()+1.
        while time.monotonic() < deadline:
            with self.control.lock:
                if self.control.policy.last_at == at:
                    return
            time.sleep(.005)
        self.fail("scene worker did not finish")

    def activate(self):
        self.control.set_enabled(True)
        for at in (10., 10.4, 10.81):
            self.send(at, ())
        for at in (11.1, 11.4, 11.7):
            self.send(at, (WALK,))
        self.assertFalse(self.control.snapshot()["blocked"])
        self.assertEqual(self.control.snapshot()["speed_limit"], .1)

    def test_disabled_never_decodes_limits_or_sends_background_stops(self):
        self.control.submit(10., b"frame", "jpeg")
        time.sleep(.12)
        self.assertEqual(self.calls, [])
        self.assertEqual(self.stops, [])
        self.ctx.publish_twist(.15, .3)
        self.assertEqual(self.sent[-1]["linear"]["x"], .15)
        self.ctx.stop_base()
        self.assertEqual(len(self.sent), 4)
        self.assertTrue(all(message["linear"]["x"] == 0. for message in self.sent[-3:]))

    def test_switch_off_immediately_clears_scene_limit_and_pending_stop(self):
        self.activate()
        self.ctx.publish_twist(.15, .3)
        self.assertAlmostEqual(self.sent[-1]["linear"]["x"], .1)
        self.assertAlmostEqual(self.sent[-1]["angular"]["z"], .2)
        self.send(12., (RED,))
        self.assertTrue(self.control.snapshot()["blocked"])
        self.control.set_enabled(False)
        count = len(self.stops)
        state = self.control.snapshot()
        self.assertFalse(state["crosswalk_active"])
        self.assertIsNone(state["speed_limit"])
        self.assertFalse(state["blocked"])
        self.ctx.publish_twist(.15, .3)
        self.assertEqual(self.sent[-1]["linear"]["x"], .15)
        self.assertEqual(self.sent[-1]["angular"]["z"], .3)
        time.sleep(.22)
        self.assertEqual(len(self.stops), count)
        self.control.set_enabled(True)
        for at in (12.3, 12.7, 13.11):
            self.send(at, (RED,))
        self.assertFalse(self.control.snapshot()["blocked"])
        self.assertFalse(self.control.snapshot()["crosswalk_active"])

    def test_camera_reset_and_stale_frames_cannot_reuse_a_previous_junction(self):
        self.activate()
        self.control.reset()
        self.assertIsNone(self.control.snapshot()["speed_limit"])
        self.assertFalse(self.control.snapshot()["crosswalk_active"])
        for at in (12., 12.4, 12.81):
            self.send(at, (RED,))
        self.assertFalse(self.control.snapshot()["blocked"])


if __name__ == "__main__":
    unittest.main()
