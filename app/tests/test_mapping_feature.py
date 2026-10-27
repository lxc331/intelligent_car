"""Page integration and independent motion gates; all communication is fake."""
import copy
import tempfile
import time
import threading
import tkinter as tk
import types
import unittest
from unittest.mock import patch, Mock

from carapp.config import DEFAULTS
from carapp.features.base import AppContext
from carapp.features.mapping import MappingFeature
from carapp.core.mapping_session import MappingSession
from carapp.model.state import CarState


class Camera(tk.Frame):
    def __init__(self, parent, ctx):
        super().__init__(parent)
    def on_tick(self, st):
        pass
    def on_close(self):
        pass


class Session:
    def __init__(self):
        self.state = dict(connected=True, running=True, owned=True, busy=False, operation="")
        self.calls = []
        self.closed = False
    def snapshot(self):
        return dict(self.state)
    def submit(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return True
    def close_async(self):
        self.closed = True
    def drain(self):
        return []


class MappingFeatureTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.logs, self.sent = [], []
        self.state = CarState()
        self.state.update(ws_connected=True, cam_connected=True)
        self.app = types.SimpleNamespace(cfg=copy.deepcopy(DEFAULTS), state=self.state, root=self.root,
            camera=None, features=[], frame_listeners=[], log=self.logs.append,
            bridge=types.SimpleNamespace(publish=lambda topic,msg:self.sent.append(msg) or True))
        self.app.bridge.subscribe = Mock(return_value=True)
        self.feature = MappingFeature(AppContext(self.app))
        with patch("carapp.features.mapping.MappingCamera", Camera), \
             patch("carapp.features.mapping.local_password", return_value=""):
            self.feature.build(self.root)
        self.app.features = [self.feature]
        self.feature.on_activate()
        self.session = Session()
        self.feature.session = self.session
        self.feature.engaged = True
        self.valid_map()

    def valid_map(self):
        now = time.monotonic()
        self.feature.model.accept_map(dict(header=dict(frame_id="map"),data=[0]*100,
            info=dict(width=10,height=10,resolution=.05,origin=dict(position=dict(x=0,y=0),
                orientation=dict(x=0,y=0,z=0,w=1)))),now)
        self.feature.model.accept_odom(dict(header=dict(frame_id="map"),child_frame_id="base_footprint",
            pose=dict(pose=dict(position=dict(x=0,y=0),orientation=dict(x=0,y=0,z=0,w=1)))), now)
        self.feature._scan_at = now

    def tearDown(self):
        self.feature.on_close()
        self.root.destroy()

    def test_motion_requires_current_map_pose_camera_and_exclusive_ownership(self):
        self.assertTrue(self.feature._ready())
        self.app.features.append(types.SimpleNamespace(motion_active=True))
        self.assertFalse(self.feature._ready())
        self.app.features.pop()
        self.state.update(cam_connected=False)
        self.assertFalse(self.feature._ready())
        self.state.update(cam_connected=True)
        self.feature._scan_at = time.monotonic()-2
        self.assertFalse(self.feature._ready())
        self.valid_map()
        self.feature.model.reset()
        self.assertFalse(self.feature._ready())

    def test_background_save_blocks_motion_but_status_poll_does_not(self):
        self.session.state.update(busy=True, operation="save_map")
        self.assertFalse(self.feature._ready())
        self.session.state["operation"] = "status"
        self.assertTrue(self.feature._ready())
        self.feature.on_deactivate()
        self.assertFalse(self.feature._ready())

    def test_status_poll_does_not_disable_start_but_foreground_action_does(self):
        self.session.state.update(busy=True, operation="status")
        self.feature.on_tick(self.state.snapshot())
        self.assertEqual(self.feature.btn_start.cget("state"), "normal")
        self.assertEqual(self.feature.btn_browse.cget("state"), "normal")
        self.session.state.update(operation="save_map")
        self.feature.on_tick(self.state.snapshot())
        self.assertEqual(self.feature.btn_start.cget("state"), "disabled")

    def test_completed_poll_waits_full_interval_before_next_poll(self):
        self.feature._last_poll = time.monotonic()-30
        self.feature._handle_event(dict(operation="status", error=None,
                                        result=dict(connected=True, running=False)))
        self.feature.on_background_tick(self.state.snapshot())
        self.assertFalse(any(args[0] == "status" for args, _ in self.session.calls))

    def test_start_button_during_slow_poll_dispatches_after_query_finishes(self):
        entered, release, started = threading.Event(), threading.Event(), threading.Event()
        class Backend:
            def connect(self):
                return dict(connected=True, running=False, owned=False)
            def status(self):
                entered.set(); release.wait(2)
                return dict(connected=True, running=False, owned=False)
            def start(self):
                started.set()
                return dict(connected=True, running=True, owned=True)
            def stop(self):
                return dict(running=False, owned=False)
            def close(self):
                pass
        session = MappingSession(Backend())
        session.submit("connect")
        end = time.monotonic()+2
        while session.snapshot()["busy"] and time.monotonic()<end:
            time.sleep(.005)
        self.assertTrue(session.snapshot()["connected"])
        self.feature.session = session
        self.feature.engaged = False
        session.drain()
        session.submit("status")
        self.assertTrue(entered.wait(1))
        try:
            self.feature.on_tick(self.state.snapshot())
            self.assertEqual(self.feature.btn_start.cget("state"), "normal")
            with patch.object(self.feature, "_start_record") as record:
                self.feature.btn_start.invoke()
            record.assert_called_once()
            self.assertEqual(session.snapshot()["operation"], "start")
            self.assertFalse(started.is_set())
            release.set()
            self.assertTrue(started.wait(1))
        finally:
            release.set()

    def test_save_uses_fixed_app_root_and_explicit_host_scope(self):
        self.feature.remote_dir.set("/home/pi/maps")
        self.feature.scope.set("小车宿主机")
        self.feature.map_name.set("沙盘A")
        original = copy.deepcopy(self.app.cfg["tracking_pick"])
        with patch("carapp.features.mapping.save_config"):
            self.feature.save_map()
        args, kwargs = self.session.calls[-1]
        self.assertEqual(args[:3], ("save_map", "/home/pi/maps", "沙盘A"))
        self.assertEqual(args[3], str(self.feature.local_root))
        self.assertEqual(self.feature.local_root.name, "savedMap")
        self.assertEqual(kwargs, dict(scope="host", serialize=True))
        self.assertEqual(original, self.app.cfg["tracking_pick"])

    def test_partial_save_does_not_claim_double_destination_success(self):
        self.feature._handle_event(dict(operation="save_map", result=dict(remote_dir="/saved/map"),
                                        error="connection lost"))
        self.assertIn("车端已保存", self.feature.status.cget("text"))
        self.assertNotIn("双端保存完成", self.feature.status.cget("text"))

    def test_disconnect_clears_old_vehicle_map_and_session(self):
        self.feature.disconnect()
        self.assertTrue(self.session.closed)
        self.assertIsNone(self.feature.session)
        self.assertIsNone(self.feature.model.snapshot()["grid"])
        self.assertFalse(self.feature.motion_active)

    def test_ros_disconnect_never_restarts_mapping(self):
        self.feature._was_connected = True
        st = self.state.snapshot()
        st["ws_connected"] = False
        self.feature.on_background_tick(st)
        self.assertIsNone(self.feature.session)
        self.assertFalse(any(args[0] == "start" for args, _ in self.session.calls))

    def test_external_process_status_cannot_rearm_after_finish(self):
        self.feature.finish()
        self.assertFalse(self.feature.engaged)
        self.session.state.update(running=True, owned=False)
        self.assertFalse(self.feature._ready())

    def test_failed_stop_is_not_reported_as_success(self):
        self.feature._handle_event(dict(operation="stop",error=None,
            result=dict(running=True, owned=True, status="stop_failed")))
        self.assertIn("尚未停止", self.feature.status.cget("text"))

    def test_started_slam_retries_map_subscription_with_an_explicit_type(self):
        self.feature._handle_event(dict(operation="start",error=None,result=dict(owned=True,running=True)))
        self.app.bridge.subscribe.assert_called_once_with("/map",throttle_ms=1000,
                                                         message_type="nav_msgs/msg/OccupancyGrid")

    def test_camera_replacement_requires_explicit_reattach_and_keeps_slam(self):
        record = Mock()
        record.camera = object()
        self.feature._record = record
        self.feature.on_background_tick(self.state.snapshot())
        self.assertFalse(self.feature.engaged)
        self.assertIsNone(self.feature._record)
        record.finish.assert_called_once()
        self.assertFalse(any(args[0] == "stop" for args, _ in self.session.calls))

    def test_hidden_page_drains_session_and_checks_permission_without_drawing(self):
        self.feature.on_deactivate()
        event = dict(operation="status", error=None, result=dict(running=False))
        self.session.drain = Mock(return_value=[event])
        with patch.object(self.feature, "_handle_event") as handle, \
             patch.object(self.feature.teleop, "on_background_tick") as permission, \
             patch.object(self.feature.map_panel, "on_tick") as draw_map, \
             patch.object(self.feature.cam, "on_tick") as draw_camera:
            self.feature.on_background_tick(self.state.snapshot())
        handle.assert_called_once_with(event)
        permission.assert_called_once()
        draw_map.assert_not_called()
        draw_camera.assert_not_called()


if __name__ == "__main__":
    unittest.main()
