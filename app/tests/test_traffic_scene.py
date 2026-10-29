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
        # The red light is still tracked on its own: it is what makes the camera
        # dip to look for the crosswalk.
        self.assertTrue(policy.snapshot()["red_pending"])

    def test_red_first_then_crosswalk_arms_the_stop(self):
        policy = TrafficScenePolicy()
        self.assertFalse(policy.observe((RED,), (640, 480), 10.))
        self.assertFalse(policy.observe((RED,), (640, 480), 10.3))
        self.assertTrue(policy.red_pending)
        self.assertFalse(policy.red_required)
        self.assertEqual(policy.snapshot()["scene"], "前方红灯，确认斑马线")
        self.assertFalse(policy.observe((WALK, RED), (640, 480), 10.6))
        self.assertFalse(policy.observe((WALK, RED), (640, 480), 10.9))
        self.assertTrue(policy.observe((WALK, RED), (640, 480), 11.2))
        self.assertTrue(policy.red_required)
        self.assertTrue(policy.snapshot()["crosswalk_active"])
        self.assertEqual(policy.snapshot()["scene"], "红灯确认，路口停车")

    def test_red_stop_holds_when_the_lamp_leaves_the_view(self):
        # While stopped the camera lifts to the lamp, then dips again; a single
        # missed frame of red must not release the stop.
        policy = TrafficScenePolicy(red_frames=2, red_hold=.6)
        for stamp in (10., 10.3):
            policy.observe((RED,), (640, 480), stamp)
        for stamp in (10.6, 10.9, 11.2):
            policy.observe((WALK, RED), (640, 480), stamp)
        self.assertTrue(policy.red_required)
        self.assertTrue(policy.observe((WALK,), (640, 480), 11.5))
        self.assertFalse(policy.observe((WALK,), (640, 480), 11.9))
        self.assertFalse(policy.red_required)

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

    def test_two_frame_confirmation_holds_through_a_single_missed_detection(self):
        policy = TrafficScenePolicy(red_frames=2, red_hold=.6)
        self.activate(policy)
        self.assertFalse(policy.observe((RED,), (640, 480), 10.9))
        self.assertTrue(policy.observe((RED,), (640, 480), 11.2))
        self.assertTrue(policy.observe((), (640, 480), 11.5))
        self.assertFalse(policy.observe((), (640, 480), 12.3))
        for bad in (0, 6, True, 1.5, "2"):
            with self.subTest(bad=bad), self.assertRaisesRegex(ValueError, "红灯确认帧数无效"):
                TrafficScenePolicy(red_frames=bad)
        for bad in (-.1, float("nan")):
            with self.subTest(bad=bad), self.assertRaisesRegex(ValueError, "红灯保持时间无效"):
                TrafficScenePolicy(red_hold=bad)

    def test_red_area_threshold_can_be_lowered_for_small_lights(self):
        small = obj("red", (300, 60, 16, 15))
        for policy, expected in ((TrafficScenePolicy(), False),
                                 (TrafficScenePolicy(red_area_min=200.), True)):
            self.activate(policy)
            self.assertIs(policy.observe((WALK, small), (640, 480), 10.9), expected)
            self.assertEqual(policy.snapshot()["red_area_min"], 200. if expected else 700.)
        for bad in (0., -1., float("nan"), float("inf")):
            with self.subTest(bad=bad), self.assertRaisesRegex(ValueError, "红灯面积门限无效"):
                TrafficScenePolicy(red_area_min=bad)

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


GREEN = obj("green", (300, 40, 40, 40))


class GreenReleaseTests(unittest.TestCase):
    """「绿灯确认才放行」是可选项，默认关；打开后要看到绿灯才松刹车。"""

    def policy(self, **extra):
        values = dict(red_area_min=200., red_frames=1, red_hold=0.,
                      green_area_min=200., release_green_frames=2, release_timeout=5.)
        values.update(extra)
        return TrafficScenePolicy(**values)

    def activate(self, policy, start=10.):
        for stamp in (start, start + .3, start + .6):
            policy.observe((WALK,), (640, 480), stamp)
        self.assertTrue(policy.snapshot()["crosswalk_active"])
        return start + .9

    def test_default_keeps_the_old_behaviour(self):
        policy = TrafficScenePolicy(red_area_min=200., red_frames=1, red_hold=0.)
        self.assertEqual(policy.release_green_frames, 0)
        stamp = self.activate(policy)
        policy.observe((RED,), (640, 480), stamp)
        self.assertTrue(policy.snapshot()["red_pending"])
        policy.observe((), (640, 480), stamp + .3)
        self.assertFalse(policy.snapshot()["red_pending"])
        self.assertFalse(policy.snapshot()["green_pending"])

    def test_red_gone_but_green_missing_keeps_holding(self):
        policy = self.policy()
        stamp = self.activate(policy)
        policy.observe((RED,), (640, 480), stamp)
        self.assertTrue(policy.snapshot()["crosswalk_active"])
        self.assertTrue(policy.snapshot()["red_pending"])
        for step in range(1, 4):
            policy.observe((), (640, 480), stamp + .3 * step)
        snapshot = policy.snapshot()
        self.assertTrue(snapshot["red_pending"])
        self.assertTrue(snapshot["waiting_green"])
        self.assertIn("等待绿灯确认", snapshot["scene"])

    def test_green_confirmation_releases_the_brake(self):
        policy = self.policy()
        stamp = self.activate(policy)
        policy.observe((RED,), (640, 480), stamp)
        for step in range(1, 4):
            policy.observe((), (640, 480), stamp + .3 * step)
        self.assertTrue(policy.snapshot()["red_pending"])
        policy.observe((GREEN,), (640, 480), stamp + 1.2)
        self.assertFalse(policy.snapshot()["green_pending"], "一帧不够")
        policy.observe((GREEN,), (640, 480), stamp + 1.5)
        snapshot = policy.snapshot()
        self.assertTrue(snapshot["green_pending"])
        self.assertFalse(snapshot["red_pending"])
        self.assertFalse(snapshot["waiting_green"])

    def test_release_timeout_frees_a_junction_with_a_broken_lamp(self):
        policy = self.policy(release_timeout=2.)
        stamp = self.activate(policy)
        policy.observe((RED,), (640, 480), stamp)
        for step in range(1, 10):
            policy.observe((), (640, 480), stamp + .3 * step)
        self.assertFalse(policy.snapshot()["red_pending"])
        self.assertFalse(policy.snapshot()["waiting_green"])

    def test_parameters_are_validated(self):
        for extra in (dict(green_area_min=0.), dict(release_green_frames=-1),
                      dict(release_green_frames=6), dict(release_green_frames=True),
                      dict(release_timeout=-.1), dict(release_timeout=float("nan"))):
            with self.assertRaises(ValueError):
                TrafficScenePolicy(**extra)


if __name__ == "__main__":
    unittest.main()
