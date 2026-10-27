"""Visible-only rendering and hidden-page maintenance, with no vehicle I/O."""
import copy
import gc
import io
import queue
import tkinter as tk
import types
import unittest
from unittest.mock import Mock, patch

from PIL import Image

from carapp.config import DEFAULTS
from carapp.core.terminal_buffer import TerminalBuffer
from carapp.features.base import Feature
from carapp.features.cruise import CruiseFeature
from carapp.features.terminal import TerminalFeature
from carapp.features.tracking_pick import TrackingPickControls
from carapp.features.gripper import GripperFeature
from carapp.model.state import CarState
from carapp.ui import camerapanel


class CameraRenderTests(unittest.TestCase):
    def setUp(self):
        gc.collect()
        self.root = tk.Tk()
        self.root.withdraw()
        self.camera = types.SimpleNamespace(recording=False, fps=lambda:10., mbps=lambda:5.)
        self.app = types.SimpleNamespace(camera=self.camera, cam_topic="/image_raw")
        self.ctx = types.SimpleNamespace(cfg=copy.deepcopy(DEFAULTS), app=self.app,
                                         add_frame_listener=Mock(), log=Mock())
        self.panel = camerapanel.CameraPanel(self.root, self.ctx)
        data = io.BytesIO()
        Image.new("RGB", (16, 12), "blue").save(data, format="JPEG")
        self.payload = data.getvalue()
        self.state = dict(cam_connected=True, cam_size=(16, 12))

    def tearDown(self):
        self.root.destroy()
        del self.panel, self.root
        gc.collect()

    def test_hidden_panel_does_not_decode_and_renders_only_latest_when_shown(self):
        self.panel._on_frame(1., b"obsolete payload", "jpeg")
        self.panel._on_frame(2., self.payload, "jpeg")
        with patch.object(self.panel, "winfo_ismapped", return_value=False), \
             patch.object(camerapanel.Image, "open", wraps=Image.open) as decode:
            self.panel.on_tick(self.state)
        decode.assert_not_called()
        self.assertEqual(self.panel._pending, (2., self.payload, "jpeg"))
        with patch.object(self.panel, "winfo_ismapped", return_value=True), \
             patch.object(camerapanel.Image, "open", wraps=Image.open) as decode:
            self.panel.on_tick(self.state)
        decode.assert_called_once()
        self.assertIsNone(self.panel._pending)
        self.assertEqual(self.panel._last_frame[0], 2.)
        self.assertIsNotNone(self.panel._imgtk)

    def test_old_camera_payload_is_discarded_after_hidden_vehicle_change(self):
        self.panel._on_frame(1., self.payload, "jpeg")
        self.app.camera = types.SimpleNamespace(recording=False, fps=lambda:10., mbps=lambda:5.)
        with patch.object(self.panel, "winfo_ismapped", return_value=True), \
             patch.object(camerapanel.Image, "open", wraps=Image.open) as decode:
            self.panel.on_tick(self.state)
        decode.assert_not_called()
        self.assertIsNone(self.panel._last_frame)
        self.assertIsNone(self.panel._pending)

    def test_new_camera_frame_received_while_hidden_is_available_when_shown(self):
        self.panel._on_frame(1., b"old", "jpeg")
        self.app.camera = types.SimpleNamespace(recording=False, fps=lambda:10., mbps=lambda:5.)
        self.panel._on_frame(2., self.payload, "jpeg")
        with patch.object(self.panel, "winfo_ismapped", return_value=True):
            self.panel.on_tick(self.state)
        self.assertIsNotNone(self.panel._imgtk)
        self.assertEqual(self.panel._last_frame[0], 2.)

    def test_snapshot_rejects_old_vehicle_without_waiting_for_render(self):
        self.panel._on_frame(1., self.payload, "jpeg")
        self.app.camera = object()
        self.panel.snapshot()
        self.assertIn("还没有画面", self.ctx.log.call_args.args[0])


class BackgroundMaintenanceTests(unittest.TestCase):
    def test_feature_default_background_and_realtime_hooks_are_optional(self):
        feature = Feature(None)
        self.assertIsNone(feature.on_background_tick({}))
        self.assertEqual(feature.realtime_ros_handlers(), {})
        self.assertIsNone(feature.reset_realtime_ros())

    def test_cruise_background_flushes_and_finishes_recording_without_rendering(self):
        feature = CruiseFeature.__new__(CruiseFeature)
        feature._flush_frames = Mock()
        feature._finish_record = Mock()
        feature.record_dir = "local-only-record"
        feature.ctx = types.SimpleNamespace(state=CarState())
        feature.runner = types.SimpleNamespace(running=False)
        feature.on_background_tick({})
        feature._flush_frames.assert_called_once()
        feature._finish_record.assert_called_once()

    def test_terminal_consumes_hidden_output_but_defers_text_and_file_rendering(self):
        feature = TerminalFeature.__new__(TerminalFeature)
        feature._closed = False
        feature._identity = ("host", "vehicle", "wired")
        feature.identity = lambda:feature._identity
        feature.available = True
        feature._auto_attempted = True
        feature._pending_directory = None
        feature._pending_files = []
        feature._listing_request = 4
        feature.remote_scope = lambda:"host"
        feature.model = TerminalBuffer(30, 8)
        feature.model.render = Mock(wraps=feature.model.render)
        feature.session = types.SimpleNamespace(generation=3, events=queue.Queue())
        feature.session.events.put((3, "data", "new remote output"))
        directory = {"path":"/new", "request_id":4, "scope":"host"}
        feature.session.events.put((3, "directory", directory))
        # A late result for the previous request must not replace the newest
        # accepted result while the page is hidden.
        feature.session.events.put((3, "directory", {"path":"/old", "request_id":3, "scope":"host"}))
        feature.session.events.put((3, "directory", {"path":"/wrong-scope", "request_id":4, "scope":"container"}))
        feature.session.events.put((3, "file", {"path":"/new/file"}))
        feature.fit_terminal = Mock()
        feature.directory_result = Mock()
        feature.file_result = Mock()
        feature.text = Mock()
        feature.text.yview.return_value = (0., 1.)
        feature.text.tag_ranges.return_value = ()
        feature.on_background_tick({})
        feature.model.render.assert_not_called()
        feature.fit_terminal.assert_not_called()
        feature.directory_result.assert_not_called()
        feature.file_result.assert_not_called()
        self.assertTrue(feature.session.events.empty())
        feature.on_tick({})
        feature.directory_result.assert_called_once_with(directory)
        feature.file_result.assert_called_once_with({"path":"/new/file"})
        self.assertIn("new remote output", feature.text.insert.call_args.args[1])

    def test_hidden_gripper_refreshes_moving_and_expired_targets_without_painting(self):
        tracking = TrackingPickControls.__new__(TrackingPickControls)
        tracking.cfg = dict(frame_timeout=.5)
        calibration = dict(x=.5, y=.6)
        tracking.calibration = lambda:calibration
        tracking.ctx = types.SimpleNamespace(state=CarState())
        tracking.label = Mock()
        tracking.tracker = Mock()
        feature = GripperFeature.__new__(GripperFeature)
        feature.tracking = tracking
        state = {}
        first, moved = dict(x=.2,y=.3), dict(x=.7,y=.4)
        with patch("carapp.features.tracking_pick.time.monotonic", return_value=10.):
            tracking.tracker.snapshot.return_value = dict(seen_at=9.9,target=first)
            feature.on_background_tick(state)
            self.assertEqual(tracking.ctx.state.get("tracking_target"), first)
            tracking.tracker.snapshot.return_value = dict(seen_at=9.9,target=moved)
            feature.on_background_tick(state)
            self.assertEqual(state["tracking_target"], moved)
            self.assertEqual(tracking.ctx.state.get("tracking_target"), moved)
            self.assertEqual(tracking.ctx.state.get("tracking_calibration"), calibration)
            tracking.tracker.snapshot.return_value = dict(seen_at=9.,target=moved)
            feature.on_background_tick(state)
        self.assertIsNone(state["tracking_target"])
        self.assertIsNone(tracking.ctx.state.get("tracking_target"))
        tracking.label.config.assert_not_called()


if __name__ == "__main__":
    unittest.main()
