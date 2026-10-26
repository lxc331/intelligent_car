"""Mapping controls: focus ownership, connection loss and worker-side limits."""
import copy
import threading
import tkinter as tk
import types
import unittest
from unittest.mock import patch

from carapp.config import DEFAULTS
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
    def test_background_worker_stops_without_gui_ticks(self):
        ctx = Context()
        stopped = threading.Event()
        ctx.stop_base = lambda: (ctx.sent.append((0., 0., 0.)), stopped.set())
        now = [10.]
        gate = _MotionGate(ctx, clock=lambda: now[0])
        controller = TeleopController(gate, autostart=True)
        try:
            gate.allowed.set()
            controller.press("w")
            self.assertGreater(ctx.sent[-1][0], 0.)
            now[0] = 15.01
            self.assertTrue(stopped.wait(.6))
            self.assertEqual(ctx.sent[-1], (0., 0., 0.))
            self.assertTrue(gate.expired)
            self.assertFalse(controller.pressed)
        finally:
            controller.close()

    def test_deadline_runs_in_worker_and_key_repeat_does_not_extend_it(self):
        ctx = Context()
        now = [1.]
        gate = _MotionGate(ctx, clock=lambda: now[0])
        controller = TeleopController(gate, clock=lambda: now[0], autostart=False)
        controller.set_speed(.04, .25)
        gate.allowed.set()
        controller.press("w")
        self.assertEqual(ctx.sent[-1], (.04, 0., 0.))
        deadline = gate.deadline
        for instant in (2., 3., 4., 5., 5.9):
            now[0] = instant
            controller.press("w")
            controller.tick()
            self.assertEqual(gate.deadline, deadline)
        now[0] = 6.
        controller.tick()
        self.assertEqual(ctx.sent[-1], (0., 0., 0.))
        self.assertTrue(gate.expired)
        count = len(ctx.sent)
        now[0] = 7.
        controller.press("w")
        controller.tick()
        self.assertEqual(len(ctx.sent), count)
        controller.release("w")
        controller.press("w")
        self.assertEqual(ctx.sent[-1], (.04, 0., 0.))
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
            self.panel.on_tick(self.ctx.state)
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
        self.assertFalse(self.panel.motion_active)
        self.assertFalse(self.panel._armed)

    def test_mapped_focusout_event_disarms_keyboard(self):
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
        self.assertFalse(self.panel.motion_active)
        self.assertFalse(self.panel._armed)

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
        self.panel.on_tick(self.ctx.state)
        self.assertFalse(self.panel.motion_active)
        self.permitted = True
        self.panel.on_tick(self.ctx.state)
        self.assertFalse(self.panel.motion_active)
        self.panel.mouse_press("w")
        self.ctx.state["ws_connected"] = False
        self.panel.on_tick(self.ctx.state)
        self.assertFalse(self.panel.motion_active)
        self.ctx.state["ws_connected"] = True
        self.panel.on_tick(self.ctx.state)
        self.assertFalse(self.panel.motion_active)

    def test_permission_loss_and_rearming_do_not_revive_held_keyboard_key(self):
        self.arm()
        with patch.object(self.root, "focus_get", return_value=self.panel.key_surface):
            self.panel.key_press(self.key("w"))
            self.permitted = False
            self.panel.on_tick(self.ctx.state)
            self.assertFalse(self.panel.motion_active)
            self.assertFalse(self.panel._armed)
            self.permitted = True
            self.panel.on_tick(self.ctx.state)
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
            original = self.panel._gate.deadline
            self.panel.key_release(self.key("w"))
            self.panel.key_press(self.key("w"))
        self.assertTrue(self.panel.motion_active)
        self.assertEqual(self.panel._gate.deadline, original)
        self.assertNotIn("w", self.panel._releases)


if __name__ == "__main__":
    unittest.main()
