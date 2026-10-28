"""Traffic switch placement and camera delivery without a car or network."""
import copy
import queue
import tkinter as tk
import types
import unittest
from unittest.mock import Mock

from carapp.config import DEFAULTS
from carapp.core.camera import CameraStream
from carapp.model.state import CarState
from carapp.ui.topbar import TopBar


class TrafficCameraDeliveryTests(unittest.TestCase):
    def test_camera_receiver_submits_directly_without_gui_tick(self):
        callback = Mock()
        stream = CameraStream("unused", 8080, "/image_raw", "jpeg", queue.Queue(1),
                              frame_handler=callback)
        stream._deliver(b"test frame")
        callback.assert_called_once_with(stream.last_frame_at, b"test frame", "jpeg")
        self.assertEqual(stream.frame_q.get_nowait()[1], b"test frame")
        stream.stop()
        stream._deliver(b"late frame")
        callback.assert_called_once()


class TrafficSwitchTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.geometry("1000x180")
        self.state = CarState()
        self.toggle, self.stop = Mock(), Mock()
        app = types.SimpleNamespace(cfg=copy.deepcopy(DEFAULTS),
            traffic_light=types.SimpleNamespace(toggle=self.toggle), stop_all_motion=self.stop)
        self.bar = TopBar(self.root, app)
        self.bar.pack(fill="x")

    def tearDown(self):
        self.root.destroy()

    def test_switch_and_stop_are_in_the_status_row_and_visible_at_minimum_width(self):
        self.bar.update_state(self.state.snapshot())
        self.root.update()
        button = self.bar.btn_traffic_light
        self.assertEqual(button.master, self.bar.lbl_conn.master.master)
        self.assertEqual(button.master, self.bar.btn_stop_all.master)
        self.assertGreater(button.winfo_width(), 50)
        self.assertLessEqual(button.winfo_rootx() + button.winfo_width(),
                             self.root.winfo_rootx() + self.root.winfo_width())
        button.invoke()
        self.toggle.assert_called_once()
        self.bar.btn_stop_all.invoke()
        self.stop.assert_called_once()

    def test_enabled_red_clear_error_and_disabled_labels(self):
        for enabled, blocked, reason in ((False, False, "未开启"),
                (True, True, "红灯停车"), (True, False, "未见红灯，可行驶"),
                (True, True, "识别异常，停车")):
            with self.subTest(reason=reason):
                self.state.update(traffic_light=dict(enabled=enabled, blocked=blocked, reason=reason),
                                  cruise_running=True, navigation_running=True)
                self.bar.update_state(self.state.snapshot())
                self.assertIn(reason if enabled else "关闭", self.bar.btn_traffic_light.cget("text"))
                self.assertEqual("暂停" in self.bar.lbl_cruise.cget("text"), enabled and blocked)
                self.assertEqual("暂停" in self.bar.lbl_navigation.cget("text"), enabled and blocked)


if __name__ == "__main__":
    unittest.main()
