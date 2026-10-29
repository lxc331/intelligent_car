"""Stop-line approach framework: distance + traffic state -> speed limit only."""
import unittest

from carapp.config import DEFAULTS
from carapp.core.junction_drive import (CrosswalkApproach, DriveCommand, StopLineDrive,
                                        approach_parameters, parameters)


def scene(red=False, active=False):
    return {"red_pending": red, "red_required": bool(red and active),
            "crosswalk_active": active}


class ParameterTests(unittest.TestCase):
    def test_defaults_are_present_and_ordered(self):
        values = parameters(DEFAULTS)
        self.assertLess(values["brake_distance"], values["stop_distance"])
        self.assertLess(values["stop_distance"], values["slow_distance"])
        for key in ("approach_speed", "slow_speed", "crawl_speed", "go_speed"):
            self.assertGreater(values[key], 0.)

    def test_missing_key_is_reported(self):
        cfg = {"junction": dict(DEFAULTS["junction"])}
        del cfg["junction"]["crawl_speed"]
        with self.assertRaises(ValueError) as caught:
            parameters(cfg)
        self.assertIn("缺少路口参数", str(caught.exception))

    def test_non_positive_or_non_finite_values_are_rejected(self):
        for key, value in (("approach_speed", 0.), ("slow_speed", -1.),
                           ("brake_distance", float("nan")), ("crawl_timeout", float("inf"))):
            cfg = {"junction": dict(DEFAULTS["junction"], **{key: value})}
            with self.assertRaises(ValueError) as caught:
                parameters(cfg)
            self.assertIn("路口参数无效", str(caught.exception))

    def test_distance_order_is_enforced(self):
        cfg = {"junction": dict(DEFAULTS["junction"], brake_distance=1.2)}
        with self.assertRaises(ValueError) as caught:
            parameters(cfg)
        self.assertIn("路口距离门限", str(caught.exception))


class StopLineDriveTests(unittest.TestCase):
    def setUp(self):
        self.drive = StopLineDrive.from_config(DEFAULTS)
        self.now = 100.

    def step(self, scene_state, distance):
        self.now += .3
        return self.drive.step(scene_state, distance, self.now)

    def test_idle_without_red_or_junction(self):
        command = self.step(scene(), 4.)
        self.assertEqual(command, DriveCommand(None, StopLineDrive.IDLE, ""))

    def test_red_without_distance_stays_stopped(self):
        command = self.step(scene(red=True), None)
        self.assertEqual(command.speed, 0.)
        self.assertEqual(command.phase, StopLineDrive.HOLD)
        self.assertIn("保持停车", command.note)

    def test_speed_profile_shrinks_as_the_stop_line_gets_closer(self):
        far = self.step(scene(red=True), 5.)
        self.assertEqual((far.speed, far.phase), (self.drive.params["approach_speed"], StopLineDrive.APPROACH))
        slowing = self.step(scene(red=True), 2.)
        self.assertEqual((slowing.speed, slowing.phase), (self.drive.params["slow_speed"], StopLineDrive.SLOW))
        crawling = self.step(scene(red=True), .6)
        self.assertEqual((crawling.speed, crawling.phase), (self.drive.params["crawl_speed"], StopLineDrive.CRAWL))
        holding = self.step(scene(red=True), .2)
        self.assertEqual((holding.speed, holding.phase), (0., StopLineDrive.HOLD))

    def test_crawl_never_lasts_forever(self):
        for _ in range(3):
            self.assertEqual(self.step(scene(red=True), .6).phase, StopLineDrive.CRAWL)
        self.now += self.drive.params["crawl_timeout"] + .1
        command = self.drive.step(scene(red=True), .6, self.now)
        self.assertEqual((command.speed, command.phase), (0., StopLineDrive.HOLD))
        self.assertIn("爬行超时", command.note)

    def test_green_starts_and_then_approaches(self):
        started = self.step(scene(active=True), .2)
        self.assertEqual((started.speed, started.phase), (self.drive.params["go_speed"], StopLineDrive.GO))
        self.assertIn("起步", started.note)
        approaching = self.step(scene(active=True), 2.)
        self.assertEqual(approaching.speed, self.drive.params["approach_speed"])
        self.assertEqual(approaching.phase, StopLineDrive.GO)

    def test_red_while_inside_the_junction_still_holds(self):
        self.step(scene(active=True), 2.)
        crawling = self.step(scene(red=True, active=True), .6)
        self.assertEqual(crawling.speed, self.drive.params["crawl_speed"])
        self.assertEqual(crawling.phase, StopLineDrive.CRAWL)
        holding = self.step(scene(red=True, active=True), .2)
        self.assertEqual(holding.speed, 0.)
        self.assertEqual(holding.phase, StopLineDrive.HOLD)

    def test_negative_or_non_finite_distance_is_rejected(self):
        for bad in (-.1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                self.drive.step(scene(red=True), bad, self.now)

    def test_reset_clears_the_phase_and_crawl_timer(self):
        self.step(scene(red=True), .6)
        self.assertIsNotNone(self.drive.crawl_started)
        self.drive.reset()
        self.assertEqual(self.drive.phase, StopLineDrive.IDLE)
        self.assertIsNone(self.drive.crawl_started)

    def test_command_is_comparable_and_readonly_slots(self):
        command = self.step(scene(red=True), 5.)
        self.assertEqual(command, DriveCommand(command.speed, command.phase, command.note))
        self.assertFalse(hasattr(command, "__dict__"))


    def test_measured_speed_scale_is_used_for_the_finish(self):
        approach = CrosswalkApproach(row_slope=-0.002220, row_intercept=1.36125,
                                     row_trigger=470., finish_travel=.22,
                                     approach_speed=.10, crawl_speed=.04, speed_scale=2.)
        approach.step(476., True, 1., self.now)
        crawling = approach.step(None, True, 1.10, self.now)
        self.assertEqual(crawling.phase, CrosswalkApproach.CRAWL)
        self.assertIn("0.02", crawling.note)
        holding = approach.step(None, True, 1.13, self.now)
        self.assertEqual(holding.phase, CrosswalkApproach.HOLD)

    def test_defaults_carry_the_measured_speed_scale(self):
        approach = CrosswalkApproach.from_config(DEFAULTS)
        self.assertGreater(approach.params["speed_scale"], 1.)

if __name__ == "__main__":
    unittest.main()


class CrosswalkApproachParameterTests(unittest.TestCase):
    def test_defaults_come_from_settings(self):
        values = approach_parameters(DEFAULTS)
        self.assertLess(values["row_slope"], 0.)
        self.assertGreater(values["finish_travel"], 0.)
        self.assertLessEqual(values["crawl_speed"], values["approach_speed"])

    def test_missing_key_is_reported(self):
        cfg = {"junction": dict(DEFAULTS["junction"])}
        del cfg["junction"]["row_trigger"]
        with self.assertRaises(ValueError) as caught:
            approach_parameters(cfg)
        self.assertIn("缺少路口接近参数", str(caught.exception))

    def test_bad_values_are_rejected(self):
        cases = (("row_slope", .002), ("row_trigger", 0.), ("finish_travel", 0.),
                 ("approach_speed", 0.), ("crawl_speed", .5), ("row_intercept", float("nan")),
                 ("speed_scale", 0.))
        for key, value in cases:
            cfg = {"junction": dict(DEFAULTS["junction"], **{key: value})}
            with self.assertRaises(ValueError) as caught:
                approach_parameters(cfg)
            message = str(caught.exception)
            self.assertTrue("路口接近" in message or "斑马线行号斜率" in message
                            or "实际速度比例" in message, message)


class CrosswalkApproachTests(unittest.TestCase):
    def setUp(self):
        self.approach = CrosswalkApproach(row_slope=-0.002220, row_intercept=1.36125,
                                          row_trigger=470., finish_travel=.22,
                                          approach_speed=.10, crawl_speed=.04)
        self.now = 1000.

    def step(self, row, red=True, travel=0.):
        return self.approach.step(row, red, travel, self.now)

    def test_row_maps_to_a_shortening_distance(self):
        far = self.approach.distance(378)
        near = self.approach.distance(476)
        self.assertAlmostEqual(far, .52, places=2)
        self.assertAlmostEqual(near, .30, places=2)

    def test_green_light_does_not_approach(self):
        command = self.step(470., red=False)
        self.assertIsNone(command.speed)
        self.assertEqual(command.phase, CrosswalkApproach.IDLE)

    def test_no_row_creeps_to_get_a_measurement(self):
        command = self.step(None)
        self.assertEqual(command.speed, .04)
        self.assertEqual(command.phase, CrosswalkApproach.CRAWL)

    def test_far_row_approaches_at_approach_speed(self):
        command = self.step(378.)
        self.assertEqual(command.speed, .10)
        self.assertEqual(command.phase, CrosswalkApproach.APPROACH)
        self.assertFalse(self.approach.triggered)

    def test_row_at_the_trigger_starts_the_odometry_finish(self):
        command = self.step(476., travel=1.)
        self.assertTrue(self.approach.triggered)
        self.assertEqual(command.speed, .04)
        self.assertEqual(command.phase, CrosswalkApproach.CRAWL)
        self.assertIn("里程计收尾", command.note)

    def test_odometry_finish_stops_after_finish_travel(self):
        self.step(476., travel=1.)
        halfway = self.step(None, travel=1.10)
        self.assertEqual(halfway.speed, .04)
        self.assertIn("0.12", halfway.note)
        holding = self.step(None, travel=1.23)
        self.assertEqual(holding.speed, 0.)
        self.assertEqual(holding.phase, CrosswalkApproach.HOLD)
        self.assertIn("停车", holding.note)

    def test_reset_clears_the_trigger(self):
        self.step(476., travel=1.)
        self.approach.reset()
        self.assertFalse(self.approach.triggered)
        self.assertEqual(self.approach.phase, CrosswalkApproach.IDLE)

    def test_bad_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            self.step(-1.)
        with self.assertRaises(ValueError):
            self.step(470., travel=-.1)
        with self.assertRaises(ValueError):
            self.step(470., travel=float("nan"))


    def test_measured_speed_scale_is_used_for_the_finish(self):
        approach = CrosswalkApproach(row_slope=-0.002220, row_intercept=1.36125,
                                     row_trigger=470., finish_travel=.22,
                                     approach_speed=.10, crawl_speed=.04, speed_scale=2.)
        approach.step(476., True, 1., self.now)
        crawling = approach.step(None, True, 1.10, self.now)
        self.assertEqual(crawling.phase, CrosswalkApproach.CRAWL)
        self.assertIn("0.02", crawling.note)
        holding = approach.step(None, True, 1.13, self.now)
        self.assertEqual(holding.phase, CrosswalkApproach.HOLD)

    def test_defaults_carry_the_measured_speed_scale(self):
        approach = CrosswalkApproach.from_config(DEFAULTS)
        self.assertGreater(approach.params["speed_scale"], 1.)

if __name__ == "__main__":
    unittest.main()
