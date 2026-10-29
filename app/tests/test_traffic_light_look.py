import math
import threading
import types
import unittest
from unittest.mock import Mock

from carapp.config import DEFAULTS
from carapp.core.traffic_light import JunctionViews
from carapp.features.traffic_light import TrafficLightControl, _build, _build_look


class JunctionViewsTests(unittest.TestCase):
    def make(self, **values):
        settings = dict(pitch=1720, dip=1450, restore=1500,
                        delay=1.4, hold=8., retry=.8, dip_hold=1.2, settle=0.)
        settings.update(values)
        return JunctionViews(settings["pitch"], settings["dip"], settings["restore"],
                             settings["delay"], settings["hold"], settings["retry"],
                             settings["dip_hold"], settings["settle"])

    def test_red_first_dips_to_confirm_the_crosswalk_then_lifts(self):
        look = self.make()
        action, notice = look.step(True, False, 10.)
        self.assertEqual(action, JunctionViews.DIP)
        self.assertIn("1450", notice)
        self.assertEqual(look.view, JunctionViews.DIP)
        action, notice = look.step(True, True, 10.5)
        self.assertEqual(action, JunctionViews.LIFT)
        self.assertIn("1720", notice)
        self.assertEqual(look.view, JunctionViews.LIFT)
        self.assertEqual(look.step(True, True, 11.), (None, "红灯确认，斑马线前停车"))
        self.assertEqual(look.step(True, True, 11.2), (None, ""))

    def test_a_confirmed_crosswalk_alone_still_lifts_the_camera(self):
        look = self.make()
        self.assertEqual(look.step(False, False, 10.)[0], None)
        action, notice = look.step(False, True, 11.)
        self.assertEqual(action, JunctionViews.LIFT)
        self.assertIn("1720", notice)
        self.assertTrue(look.lifted)

    def test_a_red_light_that_never_produces_a_crosswalk_returns_forward(self):
        look = self.make()
        self.assertEqual(look.step(True, False, 1.)[0], JunctionViews.DIP)
        self.assertEqual(look.step(True, False, 2.1), (None, ""))
        action, notice = look.step(True, False, 2.3)
        self.assertEqual(action, JunctionViews.RESTORE)
        self.assertIn("1500", notice)
        self.assertEqual(look.view, JunctionViews.FORWARD)
        # The retry gate keeps the camera forward for retry seconds.
        self.assertEqual(look.step(True, False, 3.0), (None, ""))
        self.assertEqual(look.step(True, False, 3.2)[0], JunctionViews.DIP)

    def test_a_long_red_keeps_the_lift(self):
        look = self.make(hold=1., retry=.5)
        look.step(True, False, 1.)
        look.step(True, True, 1.5)
        self.assertEqual(look.step(True, True, 2.), (None, "红灯确认，斑马线前停车"))
        self.assertEqual(look.step(True, True, 9.), (None, ""))
        self.assertEqual(look.step(True, True, 30.), (None, ""))
        self.assertTrue(look.lifted)

    def test_lamp_clearing_restores_the_forward_view_after_the_delay(self):
        look = self.make(hold=1., retry=.5)
        look.step(True, False, 1.)
        look.step(True, True, 1.5)
        self.assertEqual(look.step(True, True, 1.6), (None, "红灯确认，斑马线前停车"))
        # Crosswalk still confirmed right before the stop line: keep watching.
        self.assertEqual(look.step(False, True, 2.0), (None, ""))
        # Once the junction is left the forward view comes back after the delay.
        action, notice = look.step(False, False, 3.0)
        self.assertEqual(action, JunctionViews.RESTORE)
        self.assertIn("离开路口", notice)
        self.assertIn("1500", notice)
        self.assertEqual(look.view, JunctionViews.FORWARD)

    def test_a_lamp_that_never_reads_red_gives_the_view_back(self):
        look = self.make(hold=2., retry=.5)
        self.assertEqual(look.step(False, True, 1.)[0], JunctionViews.LIFT)
        self.assertEqual(look.step(False, True, 2.5), (None, ""))
        action, notice = look.step(False, True, 3.1)
        self.assertEqual(action, JunctionViews.RESTORE)
        self.assertIn("未确认红灯", notice)
        self.assertEqual(look.view, JunctionViews.FORWARD)
        self.assertEqual(look.step(False, True, 3.0), (None, ""))

    def test_an_idle_junction_does_not_move_the_camera(self):
        look = self.make()
        self.assertEqual(look.step(False, False, 1.), (None, ""))
        self.assertEqual(look.view, JunctionViews.FORWARD)
        self.assertFalse(look.lifted)

    def test_frames_right_after_a_view_switch_are_ignored(self):
        look = self.make(settle=.3)
        self.assertEqual(look.step(True, False, 1.)[0], JunctionViews.DIP)
        # The gimbal is still moving: a crosswalk seen now must not lift it.
        self.assertEqual(look.step(True, True, 1.2), (None, ""))
        self.assertEqual(look.view, JunctionViews.DIP)
        self.assertEqual(look.step(True, True, 1.35)[0], JunctionViews.LIFT)
        self.assertEqual(look.view, JunctionViews.LIFT)

    def test_invalid_settings_are_rejected(self):
        for values in (dict(pitch=1000), dict(pitch=2300), dict(pitch=True),
                       dict(dip=1100), dict(dip="1450"), dict(restore="1500"),
                       dict(delay=-1.), dict(delay=math.nan), dict(hold=math.inf),
                       dict(hold="x"), dict(retry=-.5), dict(dip_hold=-1.),
                       dict(settle=-.1), dict(settle="x")):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.make(**values)



class LookWiringTests(unittest.TestCase):
    """The feature must publish the dip/lift/restore angles from scene state."""

    def make(self, **state):
        control = TrafficLightControl.__new__(TrafficLightControl)
        control.ctx = types.SimpleNamespace(log=Mock(), publish_servo=Mock())
        control.look = _build_look(DEFAULTS)
        control.halt = threading.Event()
        values = dict(enabled=True, red_pending=False, crosswalk_active=False)
        values.update(state)
        control.controller = types.SimpleNamespace(snapshot=Mock(return_value=values))
        return control

    def test_a_red_light_ahead_publishes_the_dip_angle(self):
        control = self.make(red_pending=True)
        control._apply_look()
        control.ctx.publish_servo.assert_called_once_with({"pitch": 1450})

    def test_a_confirmed_crosswalk_without_red_publishes_the_lift_angle(self):
        control = self.make(crosswalk_active=True)
        control._apply_look()
        control.ctx.publish_servo.assert_called_once_with({"pitch": 1720})

    def test_a_disabled_switch_restores_the_forward_view(self):
        control = self.make(crosswalk_active=True)
        control._apply_look()
        control.ctx.publish_servo.assert_called_once_with({"pitch": 1720})
        self.assertTrue(control.look.lifted)
        control.controller.snapshot.return_value["enabled"] = False
        control._apply_look()
        control.ctx.publish_servo.assert_called_with({"pitch": 1500})
        self.assertFalse(control.look.lifted)

    def test_a_red_light_at_a_confirmed_crosswalk_keeps_the_view(self):
        control = self.make(red_pending=True, crosswalk_active=True)
        control._apply_look()
        control.ctx.publish_servo.assert_not_called()

    def test_settings_wire_the_shape_filter_three_frames_and_settle(self):
        detector, scene = _build(DEFAULTS)
        self.assertEqual(detector.red_aspect, (.6, 1.6))
        self.assertEqual(scene.red_frames, 3)
        self.assertAlmostEqual(_build_look(DEFAULTS).settle, .3)


if __name__ == "__main__":
    unittest.main()
