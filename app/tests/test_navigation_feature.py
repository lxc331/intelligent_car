"""Navigation page integration without SSH, camera recording or vehicle motion."""
import copy
from pathlib import Path
import time
import tkinter as tk
import types
import unittest
from unittest.mock import Mock, patch

from carapp.config import DEFAULTS
from carapp.core.map_data import MapModel
from carapp.features.base import AppContext
from carapp.features.navigation import NavigationFeature
from carapp.model.state import CarState


class Session:
    def __init__(self):
        self.state = dict(connected=True, running=True, ready=True, busy=False, motion_active=False)
        self.calls, self.cancels, self.closed = [], [], False

    def snapshot(self):
        return dict(self.state)

    def submit(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return True

    def request_cancel(self, stop=False):
        self.cancels.append(stop)
        return True

    def close_async(self):
        self.closed = True

    def drain(self):
        return []


class Preview(tk.Frame):
    def __init__(self, parent, ctx):
        super().__init__(parent)
        self.on_tick, self.on_close = Mock(), Mock()


class NavigationFeatureTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.logs, self.commands = [], []
        self.state = CarState()
        self.state.update(ws_connected=True, cam_connected=True)
        self.camera = types.SimpleNamespace(recording=True, record_path="offline")
        self.app = types.SimpleNamespace(cfg=copy.deepcopy(DEFAULTS), state=self.state, root=self.root,
            features=[], frame_listeners=[], camera=self.camera, log=self.logs.append,
            bridge=types.SimpleNamespace(publish=lambda topic, msg: self.commands.append(msg) or True))
        self.feature = NavigationFeature(AppContext(self.app))
        with patch("carapp.features.navigation.MappingCamera", Preview), \
                patch("carapp.features.navigation.local_password", return_value=""):
            self.feature.build(self.root)
        self.app.features = [self.feature]
        self.session = Session()
        self.feature.session = self.session
        self.feature.engaged = True
        self.feature._record = types.SimpleNamespace(camera=self.camera, event=Mock(), frame=Mock(), finish=Mock())
        grid = dict(header=dict(frame_id="map"), data=[0] * 16,
            info=dict(width=4, height=4, resolution=.5,
                origin=dict(position=dict(x=0, y=0), orientation=dict(x=0, y=0, z=0, w=1))))
        grid["data"][3] = 100
        self.bundle = types.SimpleNamespace(source_path=Path("offline.yaml"), grid_message=grid,
            width=4, height=4, resolution=.5, origin=(0.,0.,0.),
            as_upload=lambda: dict(yaml_text="image: map.pgm", image_bytes=b"P5"))
        self.feature.bundle = self.bundle
        self.feature.model.accept_map(grid, time.monotonic())
        self.feature.goal = (.5, .5, 0.)
        self.feature.monitor.snapshot = Mock(return_value=dict(error="", received_at=time.monotonic()))

    def tearDown(self):
        self.feature.on_close()
        self.root.destroy()

    def result(self, operation, **result):
        self.feature._handle_event(dict(operation=operation, result=result, error=None,
                                       target=self.feature.goal))

    def test_navigation_waits_for_successful_preview_and_is_motion_owner_immediately(self):
        self.assertTrue(self.feature.navigate())
        self.assertTrue(self.feature.motion_active)
        self.assertEqual(self.session.calls[-1][0], ("plan", .5, .5, 0.))
        self.assertFalse(any(call[0][0] == "navigate" for call in self.session.calls))
        self.result("plan", path=[(0., 0., 0.), (.5, .5, 0.)], length=.71)
        self.assertEqual(self.session.calls[-1][0], ("navigate", .5, .5, 0.))

    def test_stop_cancels_pending_auto_navigation_even_when_plan_finishes_late(self):
        self.feature.navigate()
        self.feature.stop_all()
        self.result("plan", path=[(0.,0.), (.5,.5)], length=.71)
        self.assertFalse(any(call[0][0] == "navigate" for call in self.session.calls))
        self.assertTrue(self.session.cancels)
        self.assertTrue(all(not msg["linear"]["x"] and not msg["angular"]["z"] for msg in self.commands))

    def test_failed_path_does_not_launch_movement(self):
        self.feature.navigate()
        self.result("plan", path=[], length=0.)
        self.assertFalse(self.feature.motion_active)
        self.assertFalse(any(call[0][0] == "navigate" for call in self.session.calls))

    def test_old_goal_preview_cannot_launch_new_goal(self):
        self.feature.navigate()
        self.feature._handle_event(dict(operation="plan", result=dict(path=[(0.,0.),(1.,1.)],length=1.),
                                        error=None, target=(1.,1.,0.)))
        self.assertEqual(self.feature._navigate_after_plan, self.feature.goal)
        self.assertFalse(any(call[0][0] == "navigate" for call in self.session.calls))
        self.result("plan", path=[(0.,0.),(.5,.5)],length=.71)
        self.assertEqual(self.session.calls[-1][0][0], "navigate")

    def test_target_marking_never_moves_and_rejects_occupied_unknown_outside(self):
        self.feature.bundle = self.feature.model.snapshot()["grid"]
        self.feature.select_pose("goal", 1.6, .1, 0.)
        self.assertEqual(self.feature.goal, (.5,.5,0.))
        self.feature.select_pose("goal", -1., 0., 0.)
        self.assertEqual(self.feature.goal, (.5,.5,0.))
        self.feature.select_pose("goal", .75, .75, 1.)
        self.assertEqual(self.feature.goal, (.75,.75,1.))
        self.assertTrue(all(call[0][0] != "navigate" for call in self.session.calls))

    def test_navigation_requires_localization_fresh_scan_camera_and_other_motion_clear(self):
        self.assertTrue(self.feature._ready())
        self.session.state["ready"] = False
        self.assertFalse(self.feature.navigate())
        self.session.state["ready"] = True
        self.app.features.append(types.SimpleNamespace(motion_active=True))
        self.assertFalse(self.feature._ready())
        self.app.features.pop()
        self.state.update(cam_connected=False)
        self.assertFalse(self.feature._ready())
        self.state.update(cam_connected=True)
        self.feature.monitor.snapshot.return_value["received_at"] -= 5.
        self.assertFalse(self.feature._ready())

    def test_hidden_page_maintains_permission_and_does_not_render(self):
        self.feature.on_deactivate()
        self.session.state["motion_active"] = True
        self.feature.map_panel.on_tick = Mock()
        self.feature.on_background_tick(self.state.snapshot())
        self.assertFalse(self.session.cancels)
        self.assertTrue(self.feature.motion_active)
        self.feature.map_panel.on_tick.assert_not_called()
        self.feature.cam.on_tick.assert_not_called()
        self.state.update(cam_connected=False)
        self.feature.on_background_tick(self.state.snapshot())
        self.assertTrue(self.session.cancels)

    def test_vehicle_change_closes_session_clears_goal_pose_and_old_frames(self):
        self.feature._last_frame = (1., b"old", "jpeg")
        self.app.cfg["car"]["host"] = "192.0.2.1"
        self.feature.on_background_tick(self.state.snapshot())
        self.assertTrue(self.session.closed)
        self.assertIsNone(self.feature.session)
        self.assertIsNone(self.feature.goal)
        self.assertIsNone(self.feature._last_frame)
        self.assertIsNotNone(self.feature.model.snapshot()["grid"])

    def test_start_validates_speed_before_remote_start(self):
        self.session.state["running"] = False
        for value in ("nan", "inf", "0.01", "0.16", "bad"):
            self.feature.speed.set(value)
            self.feature.start()
        self.assertFalse(any(call[0][0] == "start" for call in self.session.calls))

    def test_map_import_completion_uses_predecoded_model_without_ui_map_decode(self):
        model = MapModel()
        model.accept_map(self.bundle.grid_message, time.monotonic())
        with patch.object(model, "accept_map", side_effect=AssertionError("UI decoded map")):
            self.feature._install_map(self.bundle, model)
        self.assertIs(self.feature.model, model)
        self.assertIs(self.feature.map_panel.model, model)
        self.feature._reset_pose()
        self.assertIsNotNone(model.snapshot()["grid"])

    def test_scan_has_only_realtime_data_route_and_resets_on_session_change(self):
        self.assertIn("/scan_raw", self.feature.realtime_ros_handlers())
        self.feature.monitor.update = Mock()
        self.feature.realtime_ros_handlers()["/scan_raw"]({"ranges": []}, 12.)
        self.feature.monitor.update.assert_called_once_with({"ranges": []}, 12.)
        self.feature.monitor.reset = Mock()
        self.feature.reset_realtime_ros()
        self.feature.monitor.reset.assert_called_once_with()

    def test_disconnected_session_keeps_motion_ownership_until_async_stop_finishes(self):
        self.session.state["motion_active"] = True
        self.feature.disconnect()
        self.assertIsNone(self.feature.session)
        self.assertTrue(self.feature.motion_active)
        self.assertIn(self.session, self.feature._retired_sessions)
        self.assertIsNotNone(self.feature._record)
        self.session.state.update(motion_active=False, busy=False)
        self.feature.on_background_tick(self.state.snapshot())
        self.assertFalse(self.feature.motion_active)
        self.assertFalse(self.feature._retired_sessions)
        self.assertIsNone(self.feature._record)

    def test_canceled_or_stale_events_cannot_rearm_or_submit_navigation(self):
        for marker in ("stale", "canceled"):
            event = dict(operation="start", result=dict(running=True), error=None)
            event[marker] = True
            self.feature.engaged = False
            self.feature._handle_event(event)
            self.assertFalse(self.feature.engaged)
            self.assertFalse(self.session.calls)


if __name__ == "__main__":
    unittest.main()
