"""Per-car servo calibration.

The bench car (HW-A0857935) was confirmed on 2026-10-31 to use the same servo
direction, limits and views as HW-10CCA58E, so both entries of CALIBRATIONS now
carry the 1st car's measured values. No hardware required.
Run: python -m unittest tests.test_servo_calibration -v
"""
import copy
import json
import os
import tempfile
import types
import unittest

from carapp.config import (CALIBRATIONS, DEFAULTS, apply_calibration, load_config,
                           normalize_servo_config)
from carapp.core.teleop import TeleopController
from carapp.features.base import AppContext
from carapp.model.state import CarState


def calibrated(profile):
    cfg = copy.deepcopy(DEFAULTS)
    cfg["car"]["profile_id"] = profile
    return normalize_servo_config(apply_calibration(cfg))


class ApplyCalibrationTests(unittest.TestCase):
    def test_second_car_gets_the_measured_envelope(self):
        # 2026-10-31: this car runs the 1st car's measured servo set (see the
        # comment on CALIBRATIONS["HW-A0857935"]).
        cfg = calibrated("HW-A0857935")
        servo = cfg["servo"]
        self.assertEqual(tuple(servo["limits"]["pitch"]), (1150, 2250))
        self.assertEqual(tuple(servo["limits"]["arm"]), (1200, 2000))
        self.assertEqual(servo["actions"]["look_down"], 1300)
        self.assertEqual(servo["actions"]["look_forward"], 1500)
        self.assertEqual(servo["actions"]["arm_up"], 1200)
        self.assertEqual(servo["actions"]["arm_down"], 1700)
        self.assertEqual(servo["signs"], {"pitch": 1, "yaw": 1})
        self.assertEqual(cfg["traffic_light"]["confirm_pitch"], 1720)
        self.assertEqual(cfg["traffic_light"]["dip_pitch"], 1450)
        self.assertEqual(cfg["traffic_light"]["restore_pitch"], 1500)

    def test_first_car_keeps_the_original_envelope(self):
        cfg = calibrated("HW-10CCA58E")
        servo = cfg["servo"]
        self.assertEqual(tuple(servo["limits"]["pitch"]), (1150, 2250))
        self.assertEqual(servo["actions"]["look_down"], 1300)
        self.assertEqual(servo["signs"], {"pitch": 1, "yaw": 1})
        self.assertEqual(cfg["traffic_light"]["confirm_pitch"], 1720)
        self.assertEqual(cfg["traffic_light"]["dip_pitch"], 1450)

    def test_unknown_profile_keeps_the_saved_values(self):
        cfg = copy.deepcopy(DEFAULTS)
        cfg["car"]["profile_id"] = "HW-UNKNOWN"
        cfg["servo"]["limits"]["pitch"] = [1150, 2000]
        cfg["servo"]["actions"]["look_down"] = 1700
        apply_calibration(cfg)
        self.assertEqual(tuple(cfg["servo"]["limits"]["pitch"]), (1150, 2000))
        self.assertEqual(cfg["servo"]["actions"]["look_down"], 1700)

    def test_calibration_wins_over_stale_saved_values(self):
        """settings.json 里可能是旧记录（另一台车的数值），标定必须覆盖它。"""
        cfg = copy.deepcopy(DEFAULTS)
        cfg["car"]["profile_id"] = "HW-A0857935"
        cfg["servo"]["limits"]["pitch"] = [1150, 1800]   # 旧记录的物理上限
        cfg["servo"]["actions"]["look_down"] = 1600
        cfg["traffic_light"]["confirm_pitch"] = 1380
        cfg = normalize_servo_config(apply_calibration(cfg))
        self.assertEqual(tuple(cfg["servo"]["limits"]["pitch"]), (1150, 2250))
        self.assertEqual(cfg["servo"]["actions"]["look_down"], 1300)
        self.assertEqual(cfg["traffic_light"]["confirm_pitch"], 1720)

    def test_saved_presets_are_replaced_by_the_calibration(self):
        """预设也随装配方向变化，标定表整体替换旧预设，再统一夹进限位。"""
        cfg = copy.deepcopy(DEFAULTS)
        cfg["servo"]["presets"] = {"旧预设": {"pitch": 1900, "yaw": 500,
                                               "arm": 1100, "gripper": 2400}}
        cfg["car"]["profile_id"] = "HW-A0857935"
        cfg = normalize_servo_config(apply_calibration(cfg))
        presets = cfg["servo"]["presets"]
        self.assertEqual(set(presets), {"待机（张爪）", "俯视观察", "夹取准备", "抬起持物"})
        for pose in presets.values():
            for joint, value in pose.items():
                lo, hi = cfg["servo"]["limits"][joint]
                self.assertTrue(lo <= value <= hi, (joint, value, lo, hi))


class LoadConfigTests(unittest.TestCase):
    def write(self, values):
        handle, path = tempfile.mkstemp(suffix=".json")
        with os.fdopen(handle, "w", encoding="utf-8") as f:
            json.dump(values, f, ensure_ascii=False)
        self.addCleanup(os.unlink, path)
        return path

    def test_settings_file_profile_selects_the_calibration(self):
        path = self.write({"car": {"profile_id": "HW-A0857935"}})
        cfg = load_config(path)
        self.assertEqual(tuple(cfg["servo"]["limits"]["pitch"]), (1150, 2250))
        self.assertEqual(cfg["servo"]["actions"]["look_forward"], 1500)
        self.assertEqual(cfg["traffic_light"]["confirm_pitch"], 1720)

    def test_saved_old_car_two_values_are_replaced_by_the_calibration(self):
        path = self.write({"car": {"profile_id": "HW-A0857935"},
                           "servo": {"limits": {"pitch": [1150, 1800]},
                                     "actions": {"look_down": 1600}},
                           "traffic_light": {"confirm_pitch": 1380, "dip_pitch": 1600}})
        cfg = load_config(path)
        self.assertEqual(tuple(cfg["servo"]["limits"]["pitch"]), (1150, 2250))
        self.assertEqual(cfg["servo"]["actions"]["look_down"], 1300)
        self.assertEqual(cfg["traffic_light"]["confirm_pitch"], 1720)
        self.assertEqual(cfg["traffic_light"]["dip_pitch"], 1450)

    def test_calibration_table_only_lists_known_cars(self):
        self.assertEqual(set(CALIBRATIONS), {"HW-10CCA58E", "HW-A0857935"})


class TeleopDirectionTests(unittest.TestCase):
    def controller(self, profile):
        cfg = copy.deepcopy(DEFAULTS)
        cfg["car"]["profile_id"] = profile
        cfg = normalize_servo_config(apply_calibration(cfg))
        now = [100.]
        sent = []
        app = types.SimpleNamespace(
            cfg=cfg, state=CarState(),
            bridge=types.SimpleNamespace(publish=lambda topic, msg: sent.append((topic, msg)) or True),
            log=lambda text: None)
        app.state.update(ws_connected=True)
        controller = TeleopController(AppContext(app), clock=lambda: now[0], autostart=False)
        self.addCleanup(controller.close)
        return app, controller

    def test_pitch_key_moves_towards_the_sky_on_car_one(self):
        app, controller = self.controller("HW-10CCA58E")
        app.state.update(servo_targets=dict(pitch=1500, yaw=1500))
        controller.press("i")
        self.assertEqual(app.state.get("servo_targets")["pitch"], 1540)
        controller.release("i")
        controller.press("k")
        self.assertEqual(app.state.get("servo_targets")["pitch"], 1500)

    def test_pitch_key_moves_the_same_way_on_both_cars(self):
        # 2026-10-31: the bench car uses the 1st car's direction, so "i" raises
        # the numbers (looks up) on both profiles.
        app, controller = self.controller("HW-A0857935")
        app.state.update(servo_targets=dict(pitch=1450, yaw=1500))
        controller.press("i")
        self.assertEqual(app.state.get("servo_targets")["pitch"], 1490)
        controller.release("i")
        controller.press("k")
        self.assertEqual(app.state.get("servo_targets")["pitch"], 1450)

    def test_pitch_never_leaves_the_car_two_envelope(self):
        app, controller = self.controller("HW-A0857935")
        app.state.update(servo_targets=dict(pitch=2240, yaw=1500))
        controller.press("i")
        self.assertEqual(app.state.get("servo_targets")["pitch"], 2250)
        controller.release("i")
        app.state.update(servo_targets=dict(pitch=1160, yaw=1500))
        controller.press("k")
        self.assertEqual(app.state.get("servo_targets")["pitch"], 1150)

    def test_yaw_stays_left_for_larger_numbers_on_both_cars(self):
        for profile in ("HW-10CCA58E", "HW-A0857935"):
            app, controller = self.controller(profile)
            app.state.update(servo_targets=dict(pitch=1500, yaw=1500))
            controller.press("j")
            self.assertEqual(app.state.get("servo_targets")["yaw"], 1540, profile)
            controller.release("j")


if __name__ == "__main__":
    unittest.main()
