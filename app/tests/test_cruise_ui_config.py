"""Vendor mode configuration and UI lifecycle; no network or vehicle access."""
import copy
import json
import math
import tempfile
import time
import tkinter as tk
import types
import unittest
from pathlib import Path
from unittest.mock import patch

from carapp.config import DEFAULTS, load_config, migrate_cruise_config, save_config
from carapp.core.cruise import CruiseSettings
from carapp.features.base import AppContext
from carapp.features.cruise import CruiseFeature
from carapp.model.state import CarState


class VendorCruiseConfigTests(unittest.TestCase):
    def test_old_dual_distances_become_vendor_threshold_without_touching_calibration_or_disk(self):
        saved = {"cruise": {"front_back_distance": .42, "side_distance": .15, "speed": .08},
                 "tracking_pick": {"calibrations": {"HW-A0857935": {"red": {"x": .23, "y": .76}}}}}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"settings.json"
            path.write_text(json.dumps(saved, ensure_ascii=False), encoding="utf-8")
            before = path.read_bytes()
            cfg = load_config(str(path))
            self.assertEqual(cfg["cruise"], dict(detection_distance=.42, speed=.08, mode="avoid",
                                                  scan_angle=90., lidar_type="standard"))
            self.assertEqual(cfg["tracking_pick"]["calibrations"], saved["tracking_pick"]["calibrations"])
            self.assertEqual(path.read_bytes(), before)

    def test_new_threshold_wins_and_obsolete_distances_are_removed(self):
        cfg = {"cruise": dict(detection_distance=.85, front_back_distance=.4,
                               side_distance=.12, speed=.10, mode="guard")}
        self.assertEqual(migrate_cruise_config(cfg)["cruise"],
                         dict(detection_distance=.85, speed=.10, mode="guard"))

    def test_old_thresholds_are_clipped_to_vendor_supported_range(self):
        for old, expected in ((.1, .3), (.29, .3), (.6, .6), (1.5, 1.5), (2., 1.5)):
            with self.subTest(old=old):
                cfg = migrate_cruise_config({"cruise": dict(front_back_distance=old, speed=.15)})
                self.assertEqual(cfg["cruise"]["detection_distance"], expected)
                self.assertEqual(cfg["cruise"]["speed"], .15)

    def test_invalid_previous_values_use_defaults_and_valid_speeds_survive(self):
        for invalid in (None, "bad", math.nan, math.inf):
            cfg = migrate_cruise_config({"cruise": dict(detection_distance=invalid, speed=invalid, mode="unknown")})
            self.assertEqual(cfg["cruise"], dict(detection_distance=.6, speed=.06, mode="avoid"))
        for speed in (.02, .03, .06, .10, .15):
            cfg = migrate_cruise_config({"cruise": dict(speed=speed)})
            self.assertEqual(cfg["cruise"]["speed"], speed)

    def test_round_trip_keeps_vendor_mode_hidden_scan_parameters_and_other_settings(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"settings.json"
            cfg = copy.deepcopy(DEFAULTS)
            cfg["cruise"] = dict(detection_distance=.9, speed=.07, mode="follow", scan_angle=60., lidar_type="G4")
            cfg["tracking_pick"]["calibrations"] = {"custom": {"target": 123}}
            save_config(cfg, str(path))
            reloaded = load_config(str(path))
            self.assertEqual(reloaded["cruise"], cfg["cruise"])
            self.assertEqual(reloaded["tracking_pick"], cfg["tracking_pick"])


class _CameraPanel(tk.Frame):
    def __init__(self, parent, ctx):
        super().__init__(parent)
        self._last_frame = None

    def on_tick(self, state):
        pass

    def on_close(self):
        pass


class VendorCruiseWidgetTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.logs = []
        self.sent = []
        self.state = CarState()
        self.state.update(ws_connected=True, cam_connected=True)
        self.camera = types.SimpleNamespace(recording=False)
        self.camera.start_record = lambda path: setattr(self.camera, "recording", True)
        self.camera.stop_record = lambda: setattr(self.camera, "recording", False)
        self.app = types.SimpleNamespace(cfg=copy.deepcopy(DEFAULTS), state=self.state, root=self.root,
            camera=self.camera, features=[], frame_listeners=[], log=self.logs.append,
            bridge=types.SimpleNamespace(connected=True, publish=lambda topic, msg: self.sent.append(msg) or True))
        self.feature = CruiseFeature(AppContext(self.app))
        with patch("carapp.features.cruise.CameraPanel", _CameraPanel):
            self.feature.build(self.root)
        self.app.features.append(self.feature)
        self.feature.on_activate()
        message = dict(header=dict(frame_id="lidar_frame", stamp=dict(sec=100, nanosec=1)),
                       angle_min=0., angle_increment=math.tau/400, range_min=.1, range_max=10., ranges=[4.]*400)
        self.feature.on_ros_message("/scan_raw", message, time.monotonic())

    def tearDown(self):
        self.feature.on_close()
        self.root.destroy()

    def test_single_threshold_modes_and_hidden_settings(self):
        feature = self.feature
        self.assertEqual(feature.title, "自动巡航 + 实时画面")
        self.assertFalse(hasattr(feature, "front_back_distance"))
        self.assertFalse(hasattr(feature, "side_distance"))
        self.app.cfg["cruise"].update(scan_angle=60., lidar_type="G4")
        for mode in ("avoid", "follow", "guard"):
            feature.mode.set(mode)
            feature.detection_distance.set(".75")
            feature.speed.set(".09")
            with patch("carapp.features.cruise.save_config"):
                self.assertTrue(feature.apply_settings())
            expected = CruiseSettings(.75, .09, mode, 60., "G4")
            self.assertEqual(feature.runner.settings, expected)
            self.assertEqual(self.app.cfg["cruise"], expected.__dict__)
            self.assertIn(feature.MODES[mode], feature.applied.cget("text"))

    def test_invalid_inputs_and_save_failure_leave_active_settings_unchanged(self):
        feature = self.feature
        original = feature.runner.settings
        original_cfg = copy.deepcopy(self.app.cfg["cruise"])
        for invalid in ("nan", ".29", "1.51", "bad"):
            feature.detection_distance.set(invalid)
            self.assertFalse(feature.apply_settings())
            self.assertEqual(feature.runner.settings, original)
            self.assertEqual(self.app.cfg["cruise"], original_cfg)
        feature.detection_distance.set(".9")
        with patch("carapp.features.cruise.save_config", side_effect=OSError("disk unavailable")):
            self.assertFalse(feature.apply_settings())
        self.assertEqual(feature.runner.settings, original)
        self.assertEqual(self.app.cfg["cruise"], original_cfg)

    def test_mode_switch_closes_previous_record_before_changing_mode_and_never_restarts(self):
        feature = self.feature
        with tempfile.TemporaryDirectory() as folder:
            feature.record_dir = folder
            feature.record_camera = self.camera
            feature.record_settings = dict(feature.runner.settings.__dict__)
            feature.owns_recording = True
            self.camera.recording = True
            feature.runner.running = True
            feature.cam._last_frame = (0., b"frame", "jpeg")
            feature._notify("避障测试记录")
            feature.mode.set("follow")
            with patch("carapp.features.cruise.save_config"), patch.object(feature.runner, "start") as start:
                feature._mode_changed()
            start.assert_not_called()
            self.assertFalse(feature.motion_active)
            self.assertFalse(self.camera.recording)
            self.assertIsNone(feature.record_dir)
            self.assertEqual(feature.runner.settings.mode, "follow")
            self.assertIn("后退", feature.mode_help.cget("text"))
            report = json.loads((Path(folder)/"运行记录.json").read_text(encoding="utf-8"))
            self.assertEqual(report["algorithm"], "vendor_lidar_controller")
            self.assertEqual(report["mode"], "avoid")
            self.assertEqual(report["settings"]["mode"], "avoid")
            self.assertNotIn("vehicle_geometry", report)
            self.assertTrue(report["events"])
            self.assertTrue(all(event["mode"] == "avoid" for event in report["events"]))
            self.assertIn("ranges", report["events"][0]["raw_scan"])
            self.assertIn("decision", report["events"][0])
            self.assertTrue(list(Path(folder).glob("阶段_*.jpg")))
            self.assertTrue(all(msg["linear"]["x"] == msg["angular"]["z"] == 0. for msg in self.sent))

    def test_guard_description_start_gate_and_vehicle_change_reset(self):
        feature = self.feature
        feature.mode.set("guard")
        with patch("carapp.features.cruise.save_config"):
            feature._mode_changed()
        self.assertIn("只转向", feature.mode_help.cget("text"))
        feature.active = False
        with patch.object(feature.runner, "start") as start:
            feature.start()
            start.assert_not_called()
        self.assertGreater(feature.monitor.snapshot()["received_at"], 0.)
        feature.on_vehicle_change()
        self.assertEqual(feature.monitor.snapshot()["received_at"], 0.)
        self.assertFalse(feature.motion_active)

    def test_notification_defers_disk_io_and_preserves_borrowed_recording(self):
        feature = self.feature
        with tempfile.TemporaryDirectory() as folder:
            feature.record_dir = folder
            feature.record_camera = self.camera
            feature.record_settings = dict(feature.runner.settings.__dict__)
            feature.owns_recording = False
            self.camera.recording = True
            feature.cam._last_frame = (0., b"jpeg", "jpeg")
            with patch("builtins.open", side_effect=AssertionError("worker must not write files")):
                feature._notify("通知")
            feature.stop_all()
            self.assertTrue(self.camera.recording)
            self.assertTrue((Path(folder)/"运行记录.json").exists())

    def test_sensor_text_uses_g4_configured_angle_and_one_decision_frame(self):
        feature = self.feature
        self.app.cfg["cruise"].update(scan_angle=30., lidar_type="G4")
        feature.mode.set("follow")
        with patch("carapp.features.cruise.save_config"):
            self.assertTrue(feature.apply_settings())
        message = dict(header=dict(frame_id="lidar_frame", stamp=dict(sec=101, nanosec=1)),
                       angle_min=0., angle_increment=math.tau/400, range_min=.1, range_max=10., ranges=[4.]*400)
        message["ranges"][0] = .31  # Standard front beam; excluded by vendor G4 indexing.
        message["ranges"][205] = .75
        message["ranges"][185] = .82
        now = time.monotonic()
        feature.on_ros_message("/scan_raw", message, now)
        feature.on_tick({})
        self.assertIn("左前 0.75 m｜右前 0.82 m", feature.sensor.cget("text"))
        feature.runner.policy.decide(feature.monitor.snapshot(), feature.runner.settings, now)
        feature.runner.running = True
        newer = copy.deepcopy(message)
        newer["header"]["stamp"]["nanosec"] = 2
        newer["ranges"][205] = .45
        feature.on_ros_message("/scan_raw", newer, time.monotonic())
        feature.on_tick({})
        text = feature.sensor.cget("text")
        self.assertIn("左前 0.75 m｜右前 0.82 m", text)
        self.assertIn("最近目标 0.75 m", text)
        self.assertNotIn("0.45", text)
        feature.runner.running = False
        feature.on_tick({})
        self.assertIn("左前 0.45 m｜右前 0.82 m", feature.sensor.cget("text"))

    def test_global_stop_button_remains_available_with_hidden_cruise(self):
        from carapp.main import App
        from carapp.ui.topbar import TopBar
        self.app.ctx=AppContext(self.app)
        self.app.stop_all_motion=lambda:App.stop_all_motion(self.app)
        bar=TopBar(self.root,self.app)
        self.feature.runner.running=True
        self.feature.on_deactivate()
        self.assertTrue(self.feature.runner.running)
        self.feature.on_background_tick(self.state.snapshot())
        bar.update_state(self.state.snapshot())
        self.assertIn("后台",bar.lbl_cruise.cget("text"))
        bar.btn_stop_all.invoke()
        self.assertFalse(self.feature.motion_active)

    def test_background_wait_state_is_visible_without_finishing_recording(self):
        self.feature.runner.running=True
        self.feature.runner.policy.wait_for_scan()
        state=self.state.snapshot()
        self.feature.on_background_tick(state)
        self.assertTrue(state['cruise_running'])
        self.assertTrue(state['cruise_waiting'])


if __name__ == "__main__":
    unittest.main()
