# -*- coding: utf-8 -*-
"""红绿灯广播展示页的离线验证：不占真实端口、不连小车、不发任何控制指令。"""
import copy
import tkinter as tk
import types
import unittest
from unittest.mock import Mock

from carapp.config import DEFAULTS
from carapp.core.signal_receiver import DEFAULT_PORT
from carapp.features.traffic_broadcast import (OFF_COLOR, TrafficBroadcastFeature,
                                               lamp_state)
from carapp.ui import theme


class FakeReceiver:
    def __init__(self, state=None, running=False, error=None, start_ok=True,
                 age=.4, stale=False):
        self.port = DEFAULT_PORT
        self.running = running
        self.last_error = error
        self.start_ok = start_ok
        self.starts = 0
        self.stops = 0
        self.state = state
        self.age = age
        # 与真实接收器一致：没有报文时一定是 stale。
        self.stale = stale if state is not None else True
        self.packets = 1 if state else 0

    def start(self):
        self.starts += 1
        if self.start_ok:
            self.running = True
            self.last_error = None
        return self.start_ok

    def stop(self):
        self.stops += 1
        self.running = False

    def snapshot(self):
        return dict(state=self.state,
                    peer="192.168.150.1" if self.state else None,
                    age=None if self.state is None else self.age,
                    seq=(self.state or {}).get("seq", 0),
                    packets=self.packets, bad=0, stale=self.stale,
                    error=self.last_error)


class TrafficBroadcastPageTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.ctx = types.SimpleNamespace(cfg=copy.deepcopy(DEFAULTS), log=Mock())
        self.feature = TrafficBroadcastFeature(self.ctx)
        self.feature.build(self.root)
        self.root.update()

    def tearDown(self):
        self.feature.on_close()
        self.root.destroy()

    def listen(self, state=None, **options):
        fake = FakeReceiver(state, **options)
        self.feature._make_receiver = lambda: fake
        self.feature.on_activate()
        self.root.update()
        self.feature.on_tick({})
        return fake

    def fill(self, key, index):
        return self.feature.canvas.itemcget(self.feature._lamp_ids[key][index], "fill")

    def text(self, key):
        return self.feature.canvas.itemcget(self.feature._value_ids[key], "text")

    def test_first_activation_listens_and_closing_stops_the_receiver(self):
        fake = self.listen()
        self.assertEqual(fake.starts, 1)
        self.assertTrue(fake.running)
        self.assertIn("监听 UDP %d" % DEFAULT_PORT, self.feature.status.cget("text"))
        self.assertEqual(self.feature.button.cget("text"), "停止监听")
        self.feature.on_activate()  # 切走再回来沿用同一实例
        self.assertEqual(fake.starts, 1)
        self.feature.on_close()
        self.assertEqual(fake.stops, 1)
        self.assertFalse(fake.running)

    def test_red_and_green_lamps_follow_the_broadcast(self):
        self.listen(dict(h="red", v="green", seq=7, mask_hex="0x21", phase="直行红灯"))
        self.assertEqual(self.fill("h", 0), theme.ERR)
        self.assertEqual(self.fill("h", 1), OFF_COLOR)
        self.assertEqual(self.fill("h", 2), OFF_COLOR)
        self.assertEqual(self.fill("v", 2), theme.OK)
        self.assertEqual(self.text("h"), "红灯")
        self.assertEqual(self.text("v"), "绿灯")
        detail = self.feature.detail.cget("text")
        self.assertIn("序号 7", detail)
        self.assertIn("0x21", detail)
        self.assertIn("直行红灯", detail)

    def test_unrecognised_colour_leaves_the_lamps_dark(self):
        self.listen(dict(h="blue", v=None))
        self.assertEqual([self.fill("h", index) for index in range(3)], [OFF_COLOR] * 3)
        self.assertEqual(self.text("h"), "未知")
        self.assertEqual(self.text("v"), "未知")

    def test_stale_broadcast_keeps_the_last_colours_but_warns(self):
        self.listen(dict(h="red", v="green", seq=3), age=5.2, stale=True)
        self.assertEqual(self.fill("h", 0), theme.ERR)
        self.assertIn("超时", self.text("h"))
        detail = self.feature.detail.cget("text")
        self.assertIn("广播已超时", detail)
        self.assertIn("5.2 s 前", detail)

    def test_waiting_text_before_any_packet(self):
        self.listen()
        self.assertIn("等待路侧广播", self.feature.detail.cget("text"))
        self.assertEqual(self.text("h"), "未知")
        self.assertEqual(self.feature.status.cget("fg"), theme.WARN)

    def test_bind_failure_is_reported_and_can_be_retried(self):
        fake = FakeReceiver(start_ok=False, error="绑定端口 8765 失败：address already in use")
        self.feature._make_receiver = lambda: fake
        self.feature.on_activate()
        self.root.update()
        self.feature.on_tick({})
        self.assertIn("监听失败", self.feature.status.cget("text"))
        self.assertEqual(self.feature.button.cget("text"), "重试监听")
        self.assertFalse(fake.running)
        fake.start_ok = True
        self.feature.toggle()
        self.assertEqual(fake.starts, 2)
        self.assertTrue(fake.running)
        self.assertEqual(self.feature.button.cget("text"), "停止监听")

    def test_manual_stop_is_not_restarted_by_returning_to_the_page(self):
        fake = self.listen()
        self.feature.toggle()
        self.assertFalse(fake.running)
        self.assertEqual(self.feature.button.cget("text"), "开始监听")
        self.feature.on_activate()
        self.assertEqual(fake.starts, 1)

    def test_port_and_timeout_come_from_settings_with_a_safe_fallback(self):
        self.ctx.cfg["signal"] = {"port": 9000, "timeout": 1.5}
        receiver = self.feature._make_receiver()
        self.assertEqual((receiver.port, receiver.timeout), (9000, 1.5))
        self.ctx.cfg["signal"] = {"port": "abc", "timeout": "x"}
        receiver = self.feature._make_receiver()
        self.assertEqual((receiver.port, receiver.timeout), (DEFAULT_PORT, 3.))


class LampStateTests(unittest.TestCase):
    def test_known_aliases_and_unknown_values(self):
        for value in ("red", "RED", "红灯", "r"):
            self.assertEqual(lamp_state(value), "red")
        for value in ("green", "Green", "绿灯", "g"):
            self.assertEqual(lamp_state(value), "green")
        for value in ("yellow", "amber", "黄灯", "y"):
            self.assertEqual(lamp_state(value), "yellow")
        for value in ("", None, "blue", 0):
            self.assertEqual(lamp_state(value), "")


if __name__ == "__main__":
    unittest.main()
