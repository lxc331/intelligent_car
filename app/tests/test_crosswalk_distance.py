# -*- coding: utf-8 -*-
import math
import unittest

from carapp.core.crosswalk_distance import CrosswalkDistance


def make_row(distance, height=.30, pitch=35., focal=480., center=240.):
    angle = math.atan(height / distance)
    slope = math.tan(angle - math.radians(pitch))
    return center + slope * focal


class GeometryTests(unittest.TestCase):
    def test_distance_grows_as_the_box_moves_up_the_frame(self):
        model = CrosswalkDistance(height=.30, pitch=35.)
        near = model.from_box((0, 400, 200, 30))
        far = model.from_box((0, 260, 200, 30))
        self.assertGreater(far, near)
        self.assertGreater(near, 0.)

    def test_round_trip_against_the_synthetic_camera(self):
        model = CrosswalkDistance(height=.30, pitch=35., focal_px=480., center_y=240.)
        for distance in (0.6, 1.0, 2.0, 3.5):
            row = make_row(distance)
            self.assertAlmostEqual(model.distance(row), distance, delta=distance * .02)

    def test_pixel_rows_above_the_horizon_are_rejected(self):
        model = CrosswalkDistance(height=.30, pitch=10., focal_px=480., center_y=240.)
        with self.assertRaises(ValueError):
            model.distance(0)

    def test_box_shape_and_finiteness_are_validated(self):
        model = CrosswalkDistance()
        with self.assertRaises(ValueError):
            model.from_box((1, 2, 3))
        with self.assertRaises(ValueError):
            model.from_box((1, 2, 3, float("nan")))

    def test_invalid_parameters(self):
        for kwargs in ({"height": 0.}, {"height": 5.}, {"pitch": -1.}, {"pitch": 90.},
                       {"focal_px": 1.}, {"center_y": float("inf")}):
            with self.assertRaises(ValueError):
                CrosswalkDistance(**kwargs)

    def test_angle_is_reported_in_degrees_below_the_horizon(self):
        model = CrosswalkDistance(height=.3, pitch=40., focal_px=480., center_y=240.)
        self.assertAlmostEqual(model.angle(240), 40., places=6)
        self.assertGreater(model.angle(400), model.angle(300))


class CalibrationTests(unittest.TestCase):
    def test_calibration_recovers_the_synthetic_camera(self):
        samples = [(make_row(d, .32, 28., 500., 240.), d) for d in (0.8, 1.4, 2.2, 3.0, 4.0)]
        fitted = CrosswalkDistance(focal_px=500., center_y=240.).calibrated(samples)
        self.assertAlmostEqual(fitted.height, .32, delta=.02)
        self.assertAlmostEqual(fitted.pitch, 28., delta=1.0)
        for distance in (1.0, 2.0, 3.0):
            self.assertAlmostEqual(fitted.distance(make_row(distance, .32, 28., 500., 240.)),
                                   distance, delta=distance * .05)

    def test_calibration_needs_at_least_two_samples(self):
        with self.assertRaises(ValueError):
            CrosswalkDistance.fit([(300, 1.0)])

    def test_calibration_rejects_non_positive_distances(self):
        with self.assertRaises(ValueError):
            CrosswalkDistance.fit([(300, 1.0), (280, 0.)])


if __name__ == "__main__":
    unittest.main()
