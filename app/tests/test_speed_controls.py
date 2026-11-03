"""Shared speed control: one widget, every page that moves the car.

The mapping page already had this; the remote, gripper and cruise pages now use
the same picker, and the chosen value is saved so it survives a restart.
"""
import copy
import tkinter as tk
import types
import unittest
from unittest.mock import Mock, patch

from carapp.config import DEFAULTS
from carapp.features.cruise import CruiseFeature
from carapp.features.gripper import GripperFeature
from carapp.features.remote import RemoteFeature
from carapp.ui import theme
from carapp.ui.speedpicker import SpeedPicker


def make_widgets(test):
    root = tk.Tk()
    root.withdraw()
    test.addCleanup(root.destroy)
    return root


class SpeedPickerTests(unittest.TestCase):
    def setUp(self):
        self.root = make_widgets(self)
        self.applied = []
        self.picker = SpeedPicker(self.root, label="速度（m/s）", low=.02, high=.10,
                                  value=.05, presets=(.03, .06), on_apply=self.applied.append)

    def test_preset_click_applies_and_reports_the_value(self):
        self.picker.set_value(.06)
        self.assertEqual(self.picker.value(), .06)
        self.assertEqual(self.applied, [.06])
        self.assertEqual(self.picker.var.get(), "0.06")

    def test_typed_value_is_clamped_to_the_range(self):
        self.picker.var.set("0.50")
        self.assertEqual(self.picker.apply(), .10)
        self.assertEqual(self.picker.var.get(), "0.10")
        self.picker.var.set("abc")
        self.assertEqual(self.picker.apply(), .10)  # keeps the last valid value
        self.picker.var.set(".001")
        self.assertEqual(self.picker.apply(), .02)

    def test_initial_value_clamps_and_formats(self):
        other = SpeedPicker(self.root, label="速度", low=.02, high=.10, value=.4)
        self.assertEqual(other.value(), .10)
        self.assertEqual(other.var.get(), "0.10")


class _CameraPanel(tk.Frame):
    """Stand-in so page builds never touch the real camera widget."""

    def __init__(self, parent, ctx=None):
        super().__init__(parent)

    def on_close(self):
        pass


class PageSpeedTests(unittest.TestCase):
    """Pages keep ``var_level`` / ``speed`` for compatibility, but the picker wins."""

    def setUp(self):
        self.root = make_widgets(self)
        self.ctx = types.SimpleNamespace(
            cfg=copy.deepcopy(DEFAULTS), state=types.SimpleNamespace(get=lambda *a, **k: None),
            log=Mock(), app=types.SimpleNamespace(cam_topic="/image_raw"))
        self.ctx.cfg["base"]["speed_levels"] = {"慢": .06, "中": .10, "快": .15}

    def test_remote_speed_is_applied_and_saved(self):
        feature = RemoteFeature(self.ctx)
        feature.controller = Mock()
        with patch("carapp.features.remote.save_config") as save:
            feature.speed_changed(.10)
        feature.controller.set_speed.assert_called_with(.10, self.ctx.cfg["base"]["turn_speed"])
        self.assertEqual(self.ctx.cfg["base"]["speed"], .10)
        save.assert_called_once_with(self.ctx.cfg)

    def test_gripper_speed_overrides_the_level_menu_and_is_saved(self):
        feature = GripperFeature(self.ctx)
        feature.base_cfg = self.ctx.cfg["base"]
        feature.var_level = types.SimpleNamespace(get=lambda: "慢")
        self.assertEqual(feature._speed(), .06)
        with patch("carapp.features.gripper.save_config") as save:
            feature.speed_changed(.12)
        self.assertEqual(feature._speed(), .12)
        self.assertEqual(self.ctx.cfg["base"]["speed"], .12)
        save.assert_called_once_with(self.ctx.cfg)

    def test_cruise_page_picks_speed_with_the_shared_control(self):
        root = make_widgets(self)
        from carapp.features.base import AppContext
        from carapp.model.state import CarState
        state = CarState()
        camera = types.SimpleNamespace(recording=False, start_record=lambda p: None,
                                       stop_record=lambda: None)
        bridge = types.SimpleNamespace(connected=True, publish=Mock(return_value=True))
        app = types.SimpleNamespace(cfg=copy.deepcopy(DEFAULTS), state=state, root=root,
                                    camera=camera, features=[], frame_listeners=[],
                                    log=Mock(), bridge=bridge)
        feature = CruiseFeature(AppContext(app))
        with patch("carapp.features.cruise.CameraPanel", _CameraPanel):
            feature.build(root)
        self.addCleanup(feature.on_close)
        self.assertIs(feature.speed, feature.speed_picker.var)
        feature.speed_picker.set_value(.12)
        with patch("carapp.features.cruise.save_config"):
            feature.apply_settings()
        self.assertEqual(app.cfg["cruise"]["speed"], .12)


if __name__ == "__main__":
    unittest.main()
