"""Mapping controls: continuous holds, focus ownership and connection loss."""
import copy
import threading
import tkinter as tk
import types
import unittest
from unittest.mock import patch

from carapp.config import DEFAULTS
from carapp.core.key_state import key_is_down
from carapp.core.teleop import TeleopController
from carapp.ui.teleoppanel import TeleopPanel, _MotionGate


class Context:
    def __init__(self, root=None):
        self.root = root
        self.cfg = copy.deepcopy(DEFAULTS)
        self.state = {"ws_connected": True}
        self.sent = []
        self.logs = []

    def publish_twist(self, vx, wz, quiet=False, *, vy=0.):
        self.sent.append((vx, vy, wz))
        return True

    def stop_base(self):
        self.sent.append((0., 0., 0.))

    def log(self, message):
        self.logs.append(message)


class MappingGateTests(unittest.TestCase):
    def test_worker_continues_past_five_seconds_and_disconnect_still_stops(self):
        ctx = Context()
        stopped, continued = threading.Event(), threading.Event()
        ctx.stop_base = lambda: (ctx.sent.append((0., 0., 0.)), stopped.set())
        now = [10.]
        publish = ctx.publish_twist
        def track(vx, wz, quiet=False, *, vy=0.):
            result = publish(vx, wz, quiet=quiet, vy=vy)
            if now[0] >= 70.:
                continued.set()
            return result
        ctx.publish_twist = track
        gate = _MotionGate(ctx)
        controller = TeleopController(gate, clock=lambda: now[0], autostart=True)
        try:
            gate.allowed.set()
            controller.press("w")
            self.assertGreater(ctx.sent[-1][0], 0.)
            now[0] = 70.
            self.assertTrue(continued.wait(.6))
            self.assertEqual(controller.pressed, {"w"})
            self.assertGreater(ctx.sent[-1][0], 0.)
            self.assertFalse(stopped.is_set())
            ctx.state["ws_connected"] = False
            self.assertTrue(stopped.wait(.6))
            self.assertEqual(ctx.sent[-1], (0., 0., 0.))
            self.assertFalse(controller.pressed)
        finally:
            controller.close()

    def test_six_directions_hold_without_a_time_limit_until_release(self):
        ctx = Context()
        now = [1.]
        gate = _MotionGate(ctx)
        controller = TeleopController(gate, clock=lambda: now[0], autostart=False)
        controller.set_speed(.04, .25)
        gate.allowed.set()
        try:
            for key in "wsadqe":
                with self.subTest(key=key):
                    controller.press(key)
                    for elapsed in (4.9, 5.1, 30., 120.):
                        now[0] += elapsed
                        controller.press(key)
                        controller.tick()
                        self.assertEqual(controller.pressed, {key})
                        self.assertTrue(any(ctx.sent[-1]))
                    controller.release(key)
                    self.assertEqual(ctx.sent[-1], (0., 0., 0.))
                    self.assertFalse(controller.pressed)
        finally:
            controller.close()

    def test_revoked_permission_stops_next_worker_publication(self):
        ctx = Context()
        gate = _MotionGate(ctx)
        controller = TeleopController(gate, autostart=False)
        gate.allowed.set()
        controller.press("w")
        gate.allowed.clear()
        controller.tick()
        self.assertEqual(ctx.sent[-1], (0., 0., 0.))
        self.assertFalse(controller.pressed)
        controller.close()


class MappingTeleopUITests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.ctx = Context(self.root)
        self.permitted = True
        with patch("carapp.ui.teleoppanel.TeleopController",
                   side_effect=lambda ctx: TeleopController(ctx, autostart=False)):
            self.panel = TeleopPanel(self.root, self.ctx, ready=lambda: self.permitted)
        self.panel.pack(fill="x")
        self.panel.set_active(True)

    def tearDown(self):
        self.panel.close()
        self.root.destroy()

    def key(self, key, modifiers=0):
        return types.SimpleNamespace(keysym=key, state=modifiers)

    def arm(self):
        self.panel.mode.set("keyboard")
        self.panel.change_mode()
        self.assertTrue(self.panel.enable_keyboard())

    def test_default_speed_six_buttons_and_no_servo_keys(self):
        self.assertEqual(self.panel.speed.get(), "0.04")
        self.assertEqual(set(self.panel.mouse_buttons), set("wsadqe"))
        self.assertEqual(self.panel.controller.speed, .04)
        self.assertEqual(self.panel.controller.turn, .25)
        self.arm()
        with patch.object(self.root, "focus_get", return_value=self.panel.key_surface):
            for key in "ikjltgfhzc":
                self.panel.key_press(self.key(key))
        self.assertFalse(self.panel.motion_active)
        self.assertTrue(all(not any(cmd) for cmd in self.ctx.sent))

    def test_unready_explicit_input_logs_once_but_ticks_stay_quiet(self):
        self.permitted = False
        self.assertFalse(self.panel.mouse_press("w"))
        expected = "建图遥控未就绪：请检查建图连接、地图/定位、雷达与画面，或等待其他动作结束"
        self.assertEqual(self.ctx.logs, [expected])
        for _ in range(4):
            self.panel.on_background_tick(self.ctx.state)
        self.assertEqual(self.ctx.logs, [expected])
        self.panel.mode.set("keyboard")
        self.panel.change_mode()
        self.assertFalse(self.panel.enable_keyboard())
        self.assertEqual(self.ctx.logs, [expected, expected])

    def test_keyboard_needs_explicit_arming_and_its_own_focus(self):
        self.panel.mode.set("keyboard")
        self.panel.change_mode()
        with patch.object(self.root, "focus_get", return_value=self.panel.key_surface):
            self.panel.key_press(self.key("w"))
        self.assertFalse(self.panel.motion_active)
        self.assertTrue(self.panel.enable_keyboard())
        with patch.object(self.root, "focus_get", return_value=self.panel):
            self.panel.key_press(self.key("w"))
        self.assertFalse(self.panel.motion_active)
        with patch.object(self.root, "focus_get", return_value=self.panel.key_surface):
            self.panel.key_press(self.key("w"))
        self.assertTrue(self.panel.motion_active)
        self.assertTrue(self.panel.key_surface.bind("<FocusOut>"))
        self.panel.focus_lost()
        self.assertTrue(self.panel.motion_active)
        self.assertTrue(self.panel._armed)

    def test_mapped_focusout_keeps_hold_then_background_release_stops(self):
        self.root.deiconify()
        self.arm()
        other = tk.Entry(self.root)
        other.pack()
        self.root.update()
        self.panel.enable_keyboard()
        self.panel.key_surface.focus_force()
        self.root.update()
        self.assertIs(self.root.focus_get(), self.panel.key_surface)
        self.panel.key_press(self.key("w"))
        self.assertTrue(self.panel.motion_active)
        other.focus_force()
        self.root.update()
        self.assertTrue(self.panel.motion_active)
        self.assertTrue(self.panel._armed)
        with patch("carapp.ui.teleoppanel.key_is_down", return_value=True):
            self.panel.on_background_tick(self.ctx.state)
        self.assertTrue(self.panel.motion_active)
        # No Tk KeyRelease is delivered to the original control after focus moves.
        with patch("carapp.ui.teleoppanel.key_is_down", return_value=False):
            self.panel.on_background_tick(self.ctx.state)
        self.assertFalse(self.panel.motion_active)
        self.assertTrue(self.panel._armed)
        self.assertFalse(self.panel._unfocused_keys)
        self.assertEqual(self.ctx.sent[-1], (0., 0., 0.))

    def test_unfocused_multiple_keys_release_independently_without_new_input(self):
        self.arm()
        with patch.object(self.root, "focus_get", return_value=self.panel.key_surface):
            for key in "wq":
                self.panel.key_press(self.key(key))
        self.panel.focus_lost()
        with patch("carapp.ui.teleoppanel.key_is_down", side_effect=lambda key: key == "q"):
            self.panel.on_background_tick(self.ctx.state)
        self.assertEqual(self.panel.controller.pressed, {"q"})
        with patch.object(self.root, "focus_get", return_value=self.panel):
            self.panel.key_press(self.key("s"))
        self.assertEqual(self.panel.controller.pressed, {"q"})
        self.panel.set_active(False)
        self.assertFalse(self.panel.motion_active)
        self.assertFalse(self.panel._armed)
        self.assertFalse(self.panel._unfocused_keys)

    def test_disconnect_still_stops_while_unfocused_key_is_held(self):
        self.arm()
        with patch.object(self.root, "focus_get", return_value=self.panel.key_surface):
            self.panel.key_press(self.key("w"))
        self.panel.focus_lost()
        self.ctx.state["ws_connected"] = False
        with patch("carapp.ui.teleoppanel.key_is_down", return_value=True):
            self.panel.on_background_tick(self.ctx.state)
        self.assertFalse(self.panel.motion_active)
        self.assertFalse(self.panel._armed)
        self.assertFalse(self.panel._unfocused_keys)

    def test_queued_old_focusout_does_not_cancel_reenabled_keyboard(self):
        self.root.deiconify()
        other = tk.Entry(self.root)
        other.pack()
        self.arm()
        self.root.update()
        self.panel.key_surface.focus_force()
        self.root.update()
        self.panel.enable_keyboard()
        self.root.update()
        other.focus_set()
        self.panel.enable_keyboard()
        self.root.update()
        self.assertIs(self.root.focus_get(), self.panel.key_surface)
        self.assertTrue(self.panel._armed)
        self.panel.key_surface.event_generate('<KeyPress-w>')
        self.root.update()
        self.assertTrue(self.panel.motion_active)

    def test_mouse_release_and_departure_cancel_motion(self):
        self.assertTrue(self.panel.mouse_press("q"))
        self.assertEqual(self.ctx.sent[-1], (0., .04, 0.))
        self.panel.mouse_release()
        self.assertFalse(self.panel.motion_active)
        self.assertEqual(self.ctx.sent[-1], (0., 0., 0.))
        self.panel.mouse_press("w")
        self.panel.set_active(False)
        self.assertFalse(self.panel.motion_active)
        self.assertFalse(self.panel.mouse_press("w"))
        self.panel.set_active(True)
        self.assertFalse(self.panel.motion_active)

    def test_permission_loss_disconnect_and_recovery_never_resume(self):
        self.panel.mouse_press("w")
        self.permitted = False
        self.panel.on_background_tick(self.ctx.state)
        self.assertFalse(self.panel.motion_active)
        self.permitted = True
        self.panel.on_background_tick(self.ctx.state)
        self.assertFalse(self.panel.motion_active)
        self.panel.mouse_press("w")
        self.ctx.state["ws_connected"] = False
        self.panel.on_background_tick(self.ctx.state)
        self.assertFalse(self.panel.motion_active)
        self.ctx.state["ws_connected"] = True
        self.panel.on_background_tick(self.ctx.state)
        self.assertFalse(self.panel.motion_active)

    def test_permission_loss_and_rearming_do_not_revive_held_keyboard_key(self):
        self.arm()
        with patch.object(self.root, "focus_get", return_value=self.panel.key_surface):
            self.panel.key_press(self.key("w"))
            self.permitted = False
            self.panel.on_background_tick(self.ctx.state)
            self.assertFalse(self.panel.motion_active)
            self.assertFalse(self.panel._armed)
            self.permitted = True
            self.panel.on_background_tick(self.ctx.state)
            self.panel.key_press(self.key("w"))
            self.assertFalse(self.panel.motion_active)
            self.assertTrue(self.panel.enable_keyboard())
            self.panel.key_press(self.key("w"))
            self.assertFalse(self.panel.motion_active)
            self.panel._release_key("w")
            self.panel.key_press(self.key("w"))
            self.assertTrue(self.panel.motion_active)

    def test_new_mouse_press_can_reuse_a_keyboard_blocked_direction(self):
        self.arm()
        with patch.object(self.root, "focus_get", return_value=self.panel.key_surface):
            self.panel.key_press(self.key("w"))
        self.panel.stop()
        self.assertIn("w", self.panel.controller.blocked)
        self.panel.mode.set("mouse")
        self.panel.change_mode()
        self.assertTrue(self.panel.mouse_press("w"))
        self.assertTrue(self.panel.motion_active)

    def test_modifier_keys_space_escape_and_mode_change_stop(self):
        for key, modifiers in (("w", 4), ("w", 8), ("space", 0), ("Escape", 0)):
            self.arm()
            with patch.object(self.root, "focus_get", return_value=self.panel.key_surface):
                self.panel.key_press(self.key("w"))
                self.assertTrue(self.panel.motion_active)
                self.panel.key_press(self.key(key, modifiers))
            self.assertFalse(self.panel.motion_active)
            self.assertFalse(self.panel._armed)
            self.panel._release_key("w")
        self.panel.mode.set("mouse")
        self.panel.change_mode()
        self.panel.mouse_press("w")
        self.panel.mode.set("keyboard")
        self.panel.change_mode()
        self.assertFalse(self.panel.motion_active)

    def test_invalid_speed_keeps_previous_value_and_stops(self):
        for value in (".019", ".101", "nan", "inf", "bad"):
            self.panel.mouse_press("w")
            self.panel.speed.set(value)
            self.assertFalse(self.panel.apply_speed())
            self.assertEqual(self.panel.controller.speed, .04)
            self.assertFalse(self.panel.motion_active)
        self.panel.speed.set(".10")
        self.assertTrue(self.panel.apply_speed())
        self.assertEqual(self.panel.controller.speed, .10)

    def test_keyboard_repeat_debounce_preserves_original_hold(self):
        self.arm()
        with patch.object(self.root, "focus_get", return_value=self.panel.key_surface):
            self.panel.key_press(self.key("w"))
            original = set(self.panel.controller.pressed)
            self.panel.key_release(self.key("w"))
            self.panel.key_press(self.key("w"))
        self.assertTrue(self.panel.motion_active)
        self.assertEqual(self.panel.controller.pressed, original)
        self.assertNotIn("w", self.panel._releases)


class PhysicalKeyStateTests(unittest.TestCase):
    def test_windows_reads_high_bit_and_normalizes_key(self):
        user32 = types.SimpleNamespace(GetAsyncKeyState=lambda code: 0x8001 if code == ord("W") else 1)
        with patch("carapp.core.key_state.sys.platform", "win32"), \
                patch("carapp.core.key_state.ctypes.windll", types.SimpleNamespace(user32=user32), create=True):
            self.assertTrue(key_is_down("w"))
            self.assertFalse(key_is_down("q"))

    def test_other_platform_reports_unavailable(self):
        with patch("carapp.core.key_state.sys.platform", "linux"):
            self.assertIsNone(key_is_down("w"))


if __name__ == "__main__":
    unittest.main()
