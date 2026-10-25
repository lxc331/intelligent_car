"""Compare the PC port against the unmodified vendor callbacks, without ROS or a car.

The reference is extracted from the bundled source with AST, including its real
PID implementation.  The tiny NumPy substitute is only the indexing/minimum
subset used by that source, keeping this test usable with the desktop runtime.
"""
import ast
import math
import random
import types
import unittest
import warnings
from pathlib import Path

from carapp.core.vendor_lidar import VendorLidarAlgorithm


class _Array:
    def __init__(self, values):
        self.values = list(values)

    def __len__(self):
        return len(self.values)

    def __iter__(self):
        return iter(self.values)

    def __getitem__(self, index):
        if isinstance(index, tuple):
            index = index[0]
        if isinstance(index, _Array):
            index = index.values
        if isinstance(index, list):
            if all(isinstance(value, bool) for value in index):
                return _Array(value for value, keep in zip(self.values, index) if keep)
            return _Array(self.values[value] for value in index)
        result = self.values[index]
        return _Array(result) if isinstance(index, slice) else result

    def nonzero(self):
        return ([index for index, value in enumerate(self.values) if value != 0],)

    def min(self):
        return min(self.values)


class _NumpySubset:
    array = staticmethod(_Array)
    append = staticmethod(lambda left, right: _Array([*left, *right]))
    isfinite = staticmethod(lambda values: _Array(math.isfinite(value) for value in values))


class _Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0., y=0., z=0.)
        self.angular = types.SimpleNamespace(x=0., y=0., z=0.)


def _source_nodes(path, names, *, class_name=None):
    # Vendor docstrings contain old backslash escapes; Python 3.13 warns when
    # parsing them, although the controller itself is unaffected.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", SyntaxWarning)
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    nodes = tree.body
    if class_name:
        nodes = next(node.body for node in nodes if isinstance(node, ast.ClassDef)
                     and node.name == class_name)
    selected = [node for node in nodes if isinstance(node, (ast.FunctionDef, ast.ClassDef))
                and node.name in names]
    if len(selected) != len(names):
        raise AssertionError("Bundled vendor reference has changed: " + str(path))
    return ast.Module(body=selected, type_ignores=[])


class VendorReference:
    """Execute the actual vendor method; deliberately retain no-publish commands."""
    def __init__(self, mode="avoid", detection_distance=.6, speed=.06,
                 scan_angle=90., lidar_type="standard", now=0.):
        root = Path(__file__).resolve().parents[2]
        source = root / "ClawPi智能小车" / "3 源码资料" / "src"
        clock = types.SimpleNamespace(value=now)
        clock.time = lambda: clock.value
        try:
            import numpy as np
        except ImportError:
            np = _NumpySubset
        namespace = dict(math=math, time=clock, np=np, Twist=_Twist,
                         MAX_SCAN_ANGLE=240)
        pid_path = source / "driver" / "sdk" / "sdk" / "pid.py"
        exec(compile(_source_nodes(pid_path, {"PID"}), str(pid_path), "exec"), namespace)
        common_path = source / "driver" / "sdk" / "sdk" / "common.py"
        exec(compile(_source_nodes(common_path, {"set_range"}), str(common_path), "exec"), namespace)
        namespace["common"] = types.SimpleNamespace(set_range=namespace["set_range"])
        callback_path = source / "app" / "app" / "lidar_controller.py"
        exec(compile(_source_nodes(callback_path, {"lidar_callback"}, class_name="LidarController"),
                     str(callback_path), "exec"), namespace)
        self.clock = clock
        self.command = (0., 0.)
        self.published = 0
        import threading
        logger = types.SimpleNamespace(info=lambda *_: None)
        self.node = types.SimpleNamespace(
            running_mode={"avoid": 1, "follow": 2, "guard": 3}[mode],
            threshold=detection_distance, speed=speed, scan_angle=math.radians(scan_angle),
            lidar_type="G4" if lidar_type == "G4" else "LD19", last_act=0, timestamp=0.,
            pid_yaw=namespace["PID"](6.6, 0., .01),
            pid_dist=namespace["PID"](1.7, 0., .16), lock=threading.RLock(),
            get_logger=lambda: logger,
            mecanum_pub=types.SimpleNamespace(publish=self._publish))
        self.callback = namespace["lidar_callback"]

    def _publish(self, twist):
        self.command = (twist.linear.x, twist.angular.z)
        self.published += 1

    def update(self, scan, now):
        self.clock.value = now
        self.callback(self.node, types.SimpleNamespace(**scan))
        return self.command


def _indices(lidar_type="standard", count=400, scan_angle=90.):
    increment = math.tau / count
    indices = list(range(count))
    maximum = int(math.radians(120) / increment)
    if lidar_type == "G4":
        minimum = int(math.radians(60) / increment)
        maximum += minimum
        left = indices[::-1][minimum:maximum][::-1]
        right = indices[minimum:maximum][::-1]
    else:
        left, right = indices[:maximum], indices[::-1][:maximum]
    angle_index = int(math.radians(scan_angle / 2) / increment + .50)
    return left[:angle_index], right[:angle_index]


def _scan(left=2., right=2., *, lidar_type="standard", count=400, scan_angle=90.):
    ranges = [2.] * count
    left_indices, right_indices = _indices(lidar_type, count, scan_angle)
    for index in left_indices:
        ranges[index] = left
    for index in right_indices:
        ranges[index] = right
    return dict(ranges=ranges, angle_increment=math.tau / count, angle_min=0.,
                range_min=.1, range_max=10., header=dict(frame_id="lidar_frame"))


def _target(angle, distance, *, lidar_type="standard", count=400, scan_angle=90.):
    scan = _scan(lidar_type=lidar_type, count=count, scan_angle=scan_angle)
    left, right = _indices(lidar_type, count, scan_angle)
    merged = right[::-1] + left
    index = round((math.radians(angle) + math.radians(scan_angle / 2)) / scan["angle_increment"])
    scan["ranges"][merged[index]] = distance
    return scan


class VendorParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = Path(__file__).resolve().parents[2]/"ClawPi智能小车/3 源码资料/src"
        required = ("app/app/lidar_controller.py", "driver/sdk/sdk/pid.py", "driver/sdk/sdk/common.py")
        if not all((source/relative).is_file() for relative in required):
            raise unittest.SkipTest("Local vendor sources are absent; numerical reference comparison skipped")

    def pair(self, **settings):
        return VendorReference(**settings), VendorLidarAlgorithm(**settings)

    def compare(self, pair, scan, now):
        expected, actual = pair[0].update(scan, now), pair[1].update(scan, now)
        self.assertEqual(len(actual), 2)
        self.assertAlmostEqual(actual[0], expected[0], places=11)
        self.assertAlmostEqual(actual[1], expected[1], places=11)
        self.assertEqual(tuple(pair[1].command), tuple(actual))
        return actual

    def test_avoid_all_four_threshold_regions_and_exact_threshold(self):
        for lidar_type in ("standard", "G4"):
            for left, right in ((2., 2.), (.5, 2.), (2., .5), (.5, .5),
                                (.6, 2.), (2., .6), (.6, .6)):
                with self.subTest(lidar_type=lidar_type, left=left, right=right):
                    self.compare(self.pair(lidar_type=lidar_type),
                                 _scan(left, right, lidar_type=lidar_type), .1)

    def test_avoid_hold_window_retains_command_then_rechecks_at_deadline(self):
        for sides in ((.5, 2.), (.5, .5), (2., .5)):
            with self.subTest(sides=sides):
                pair = self.pair()
                self.compare(pair, _scan(*sides), .1)
                deadline = pair[0].node.timestamp
                self.compare(pair, _scan(), deadline - .001)
                self.assertGreater(abs(pair[1].command[1]), 0.)
                self.compare(pair, _scan(), deadline)
                self.assertEqual(pair[1].command, (.06, 0.))

    def test_last_act_changes_turn_sign_across_regions(self):
        pair = self.pair()
        for step, sides in enumerate(((.5, 2.), (2., .5), (.5, 2.),
                                      (.5, .5), (2., .5), (.5, 2.), (2., 2.))):
            self.compare(pair, _scan(*sides), 1. + step * 10.)

    def test_slice_limits_ignore_points_outside_vendor_front_fan(self):
        for lidar_type in ("standard", "G4"):
            pair = self.pair(lidar_type=lidar_type)
            scan = _scan(lidar_type=lidar_type)
            left, right = _indices(lidar_type)
            for index in set(range(400)) - set(left + right):
                scan["ranges"][index] = .05
            self.compare(pair, scan, .1)
            self.assertEqual(pair[1].command, (.06, 0.))

    def test_configured_threshold_speed_and_nonintegral_scan_rounding(self):
        for threshold, speed, angle in ((.3, .02, 30.), (1.5, .15, 61.), (.9, .1, 90.)):
            with self.subTest(threshold=threshold, angle=angle):
                pair = self.pair(detection_distance=threshold, speed=speed, scan_angle=angle)
                self.compare(pair, _scan(threshold, 2., scan_angle=angle), .1)

    def test_follow_distance_control_near_target_far_target_and_vendor_deadband(self):
        for threshold in (.3, .6, 1.5):
            for distance in (.1, .18, .2, .22, threshold / 2, threshold - .01,
                             threshold, threshold + .01):
                with self.subTest(threshold=threshold, distance=distance):
                    pair = self.pair(mode="follow", detection_distance=threshold)
                    self.compare(pair, _target(0., distance), .1)

    def test_follow_turn_deadband_and_both_lidar_variants(self):
        for lidar_type in ("standard", "G4"):
            for angle in (-30., -6., -4., 0., 4., 6., 30.):
                with self.subTest(lidar_type=lidar_type, angle=angle):
                    pair = self.pair(mode="follow", lidar_type=lidar_type)
                    self.compare(pair, _target(angle, .4, lidar_type=lidar_type), .1)

    def test_follow_pid_derivative_uses_actual_frame_intervals_and_clear_behavior(self):
        pair = self.pair(mode="follow", detection_distance=.9)
        for now, angle, distance in ((.1, 9., .44), (.35, 12., .46), (.9, 0., .2),
                                     (2., 8., .43), (2.07, -9., .42), (4., 4., 1.2),
                                     (4.5, -7., .44)):
            self.compare(pair, _target(angle, distance), now)

    def test_guard_turns_only_and_never_translates(self):
        for lidar_type in ("standard", "G4"):
            pair = self.pair(mode="guard", lidar_type=lidar_type)
            for step, angle in enumerate((-30., -4., 15., 0., 30.)):
                result = self.compare(pair, _target(angle, .4, lidar_type=lidar_type), .1 + step * .2)
                self.assertEqual(result[0], 0.)
            self.compare(pair, _target(20., .6, lidar_type=lidar_type), 2.)
            self.assertEqual(pair[1].command, (0., 0.))

    def test_nearest_tie_uses_first_point_in_right_to_left_merge(self):
        pair = self.pair(mode="follow")
        scan = _target(-20., .4)
        positive = _target(20., .4)
        scan["ranges"] = [min(a, b) for a, b in zip(scan["ranges"], positive["ranges"])]
        self.compare(pair, scan, .1)
        self.assertLess(pair[1].command[1], 0.)

    def test_zero_nan_and_infinity_are_excluded_without_moving_other_samples(self):
        for mode in ("avoid", "follow", "guard"):
            with self.subTest(mode=mode):
                pair = self.pair(mode=mode)
                scan = _target(20., .4)
                left, right = _indices()
                for index, value in zip(left[5:9] + right[5:9],
                                        (0., math.nan, math.inf, -math.inf) * 2):
                    scan["ranges"][index] = value
                self.compare(pair, scan, .1)

    def test_deterministic_stateful_sequences_match_vendor_for_1800_frames(self):
        randomizer = random.Random(1025)
        comparisons = 0
        for lidar_type in ("standard", "G4"):
            for mode in ("avoid", "follow", "guard"):
                pair = self.pair(lidar_type=lidar_type, mode=mode, scan_angle=61.,
                                 detection_distance=.8, speed=.05)
                now = 0.
                for frame in range(300):
                    # Zero and uneven intervals also exercise PID derivative
                    # timing, while long intervals expire the timed arc hold.
                    now += randomizer.choice((0., .01, .03, .1, .2, 1., 3.))
                    scan = _scan(randomizer.uniform(.1, 2.), randomizer.uniform(.1, 2.),
                                 lidar_type=lidar_type, scan_angle=61.)
                    if mode != "avoid":
                        target = _target(randomizer.uniform(-25., 25.),
                                         randomizer.uniform(.1, 1.5),
                                         lidar_type=lidar_type, scan_angle=61.)
                        scan["ranges"] = [min(a, b) for a, b in zip(scan["ranges"], target["ranges"])]
                    with self.subTest(lidar_type=lidar_type, mode=mode, frame=frame, now=now):
                        self.compare(pair, scan, now)
                    comparisons += 1
        self.assertEqual(comparisons, 1800)


if __name__ == "__main__":
    unittest.main()
