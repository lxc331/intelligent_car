"""Teleoperation tests use a recording bridge, never a real robot."""
import copy
import math
import threading
import time
import tkinter as tk
import types
import unittest
from unittest.mock import patch

from carapp.config import DEFAULTS
from carapp.core.teleop import TeleopController
from carapp.features.base import AppContext
from carapp.features.gripper import GripperFeature
from carapp.features.cruise import CruiseFeature
from carapp.features.remote import RemoteFeature
from carapp.main import App
from carapp.model.state import CarState


class TeleopTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.sent, self.logs = [], []
        self.app = types.SimpleNamespace(cfg=copy.deepcopy(DEFAULTS), state=CarState(),
            bridge=types.SimpleNamespace(publish=lambda topic, msg: self.sent.append((topic, msg)) or True),
            log=self.logs.append)
        self.app.state.update(ws_connected=True)
        self.ctx = AppContext(self.app)
        self.controller = TeleopController(self.ctx, clock=lambda: self.now, autostart=False)

    def tearDown(self):
        self.controller.close()

    def twist(self):
        return next(msg for topic, msg in reversed(self.sent) if topic == '/controller/cmd_vel')

    def servos(self):
        return [msg for topic, msg in self.sent if topic.endswith('/set_state')]

    def test_all_six_base_keys_and_release_to_zero(self):
        for key, axis, sign in [('w','x',1), ('s','x',-1), ('q','y',1), ('e','y',-1),
                                ('a','z',1), ('d','z',-1)]:
            self.controller.press(key)
            part = 'angular' if axis == 'z' else 'linear'
            self.assertGreater(self.twist()[part][axis]*sign, 0)
            if axis == 'y':
                self.assertEqual(self.twist()['angular']['z'], 0)
                self.assertEqual(self.twist()['linear']['x'], 0)
            self.controller.release(key)
            self.assertEqual(self.twist()['linear'], dict(x=0., y=0., z=0.))
            self.assertEqual(self.twist()['angular']['z'], 0)
            self.assertEqual(self.app.state.get('base_vy'), 0.)


    def test_removed_keys_never_reach_transport(self):
        for key in ('z','c','Z','C'):
            self.assertFalse(self.controller.press(key))
        self.assertFalse(self.controller.pressed)
        self.assertEqual(self.sent,[])

    def test_diagonal_is_bounded_and_release_keeps_other_axis(self):
        self.controller.press('w'); self.controller.press('q')
        msg = self.twist()['linear']
        self.assertAlmostEqual(math.hypot(msg['x'], msg['y']), .06)
        self.controller.release('w')
        self.assertEqual(self.twist()['linear']['x'], 0)
        self.assertAlmostEqual(self.twist()['linear']['y'], .06)

    def test_opposite_keys_cancel_without_extra_speed(self):
        self.controller.press('w'); self.controller.press('s')
        self.assertEqual(self.twist()['linear']['x'], 0)
        self.controller.release('s')
        self.assertEqual(self.twist()['linear']['x'], .06)

    def test_hold_timeout_and_repeat_do_not_restart_motion(self):
        self.controller.press('q')
        self.now += 4.9; self.controller.press('q')
        self.now += .2; self.controller.tick()
        self.assertEqual(self.twist()['linear']['y'], 0)
        count = len(self.sent)
        self.controller.press('q'); self.controller.tick()
        self.assertEqual(len(self.sent), count)
        self.controller.release('q'); self.controller.press('q')
        self.assertGreater(self.twist()['linear']['y'], 0)

    def test_servo_endpoints_and_repeat_ignored(self):
        for key, channel, value in [('t',3,1200), ('g',3,1700), ('f',4,2000), ('h',4,1500)]:
            before = len(self.servos())
            self.controller.press(key); self.controller.press(key)
            self.assertEqual(len(self.servos()), before+1)
            self.assertEqual(self.servos()[-1]['state'][0]['id'], [channel])
            self.assertEqual(self.servos()[-1]['state'][0]['position'], [value])
            self.controller.release(key)

    def test_camera_directions_are_incremental(self):
        self.app.state.update(servo_targets=dict(pitch=1500, yaw=1500))
        for key, joint, value in [('i','pitch',1540), ('k','pitch',1500),
                                 ('j','yaw',1540), ('l','yaw',1500)]:
            self.controller.press(key)
            self.assertEqual(self.app.state.get('servo_targets')[joint], value)
            self.controller.release(key)

    def test_camera_clamps_both_margins_and_timeout_stops_repeats(self):
        self.app.state.update(servo_targets=dict(pitch=2240, yaw=660))
        self.controller.press('i'); self.controller.press('l')
        self.assertEqual(self.app.state.get('servo_targets'), dict(pitch=2250, yaw=650))
        self.now += 6; self.controller.tick()
        count = len(self.servos())
        self.controller.press('i'); self.now += 1; self.controller.tick()
        self.assertEqual(len(self.servos()), count)
        self.assertFalse(self.controller.pressed)

    def test_stop_disconnection_and_reconnection_never_resume(self):
        self.controller.press('w')
        self.app.state.update(ws_connected=False)
        self.controller.tick()
        self.app.state.update(ws_connected=True)
        count = len(self.sent)
        self.controller.tick(); self.controller.press('w')
        self.assertEqual(len(self.sent), count)
        self.assertFalse(self.controller.pressed)
        self.controller.release('w'); self.controller.press('w')
        self.assertGreater(self.twist()['linear']['x'], 0)
        self.controller.stop()
        count = len(self.sent)
        self.controller.tick()
        self.assertEqual(len(self.sent), count)

    def test_publish_failure_cancels_all_inputs(self):
        self.controller.press('w')
        self.app.bridge.publish = lambda topic, msg: False
        self.controller.press('q')
        self.assertFalse(self.controller.pressed)
        self.assertEqual(self.app.state.get('base_vy'), 0.)

    def test_lateral_gripper_hold_and_stop(self):
        feature = GripperFeature(self.ctx)
        feature.base_cfg = self.app.cfg['base']
        feature.var_level = types.SimpleNamespace(get=lambda:'慢')
        feature.jog(0, 0, sy=1)
        time.sleep(.02)
        self.assertTrue(any(msg.get('linear', {}).get('y', 0) > 0 for _, msg in self.sent))
        feature.on_deactivate()
        self.assertFalse(feature._motion_thread.is_alive())
        count = len(self.sent)
        time.sleep(.03)
        self.assertEqual(len(self.sent), count)
        self.assertEqual(self.twist()['linear']['y'], 0.)

    def test_watchdog_runs_without_tk_event_loop(self):
        self.app.cfg['base']['jog_max_sec'] = .1
        controller = TeleopController(self.ctx)
        try:
            controller.press('e')
            time.sleep(.25)
            self.assertEqual(self.twist()['linear']['y'], 0.)
            self.assertFalse(controller.pressed)
        finally:
            controller.close()

    def test_invalid_base_values_never_reach_transport(self):
        for values in [dict(vy=float('nan')), dict(vx=float('inf')), dict(wz='bad')]:
            self.assertFalse(self.ctx.publish_twist(**values))
        self.assertEqual(self.sent, [])


class RemoteWidgetTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk(); self.root.withdraw()
        self.sent = []
        self.app = types.SimpleNamespace(cfg=copy.deepcopy(DEFAULTS), state=CarState(), root=self.root,
            camera=None, frame_listeners=[], features=[], log=lambda text:None,
            bridge=types.SimpleNamespace(publish=lambda topic, msg:self.sent.append((topic,msg)) or True))
        self.app.state.update(ws_connected=True)
        self.feature = RemoteFeature(AppContext(self.app))
        self.feature.build(self.root)
        self.app.features.append(self.feature)
        self.feature.on_activate()

    def tearDown(self):
        self.feature.on_close(); self.root.destroy()

    def event(self, key, state=0):
        return types.SimpleNamespace(keysym=key, state=state)

    def arm(self):
        self.feature.var_mode.set('keyboard'); self.feature.change_mode()
        self.feature.enable_keyboard()

    def test_mouse_keyboard_switch_and_tab_exit_stop(self):
        self.feature.mouse_press('q')
        self.assertGreater(self.app.state.get('base_vy'), 0)
        self.arm()
        self.assertEqual(self.app.state.get('base_vy'), 0)
        self.feature.key_press(self.event('w'))
        self.assertGreater(self.app.state.get('base_cmd')[0], 0)
        self.feature.on_deactivate()
        self.assertFalse(self.feature._armed)
        self.assertEqual(self.app.state.get('base_cmd'), (0.,0.))
        count = len(self.sent)
        self.feature.key_press(self.event('w'))
        self.assertEqual(len(self.sent), count)

    def test_keyboard_not_bound_to_root_or_active_in_mouse_mode(self):
        self.assertFalse(self.root.bind('<KeyPress>'))
        self.feature.key_press(self.event('w'))
        self.assertFalse(self.feature.controller.pressed)
        self.arm()
        self.feature.key_press(self.event('w', state=4))
        self.assertFalse(self.feature.controller.pressed)
        self.assertFalse(self.feature._armed)

    def test_repeat_release_pair_is_debounced_but_real_release_stops(self):
        self.arm()
        self.feature.key_press(self.event('q'))
        deadline = self.feature.controller.deadlines['base']
        self.feature.key_release(self.event('q'))
        self.feature.key_press(self.event('q'))
        self.assertFalse(self.feature._releases)
        self.assertEqual(self.feature.controller.deadlines['base'], deadline)
        self.feature.key_release(self.event('q'))
        self.root.after(35, self.root.quit); self.root.mainloop()
        self.assertEqual(self.app.state.get('base_vy'), 0.)

    def test_focus_loss_space_and_escape_disarm_and_stop(self):
        self.assertTrue(self.feature.key_surface.bind('<FocusOut>'))
        for action in [self.feature.focus_lost,
                       lambda:self.feature.key_press(self.event('space')),
                       lambda:self.feature.key_press(self.event('Escape'))]:
            self.arm(); self.feature.key_press(self.event('w'))
            action()
            self.assertFalse(self.feature._armed)
            self.assertFalse(self.feature.controller.pressed)

    def test_mouse_release_stop_and_sequence_exclusion(self):
        self.feature.mouse_press('e'); self.feature.mouse_release()
        self.assertEqual(self.app.state.get('base_vy'), 0.)
        gripper = GripperFeature(AppContext(self.app))
        gripper._busy = True
        self.app.features.append(gripper)
        count = len(self.sent)
        self.feature.mouse_press('w')
        self.assertEqual(len(self.sent), count)

    def test_cruise_stopping_thread_keeps_remote_blocked_until_finished(self):
        cruise = CruiseFeature(AppContext(self.app))
        finished = threading.Event()
        cruise.runner.thread = threading.Thread(target=finished.wait, daemon=True)
        cruise.runner.thread.start()
        self.app.features.append(cruise)
        try:
            count = len(self.sent)
            self.feature.mouse_press('w')
            self.assertEqual(len(self.sent), count)
            finished.set()
            cruise.runner.thread.join(timeout=1.)
            self.feature.mouse_press('w')
            self.assertGreater(self.app.state.get('base_cmd')[0], 0.)
            self.feature.mouse_release()
            self.assertEqual(self.app.state.get('base_cmd'), (0., 0.))
        finally:
            finished.set()
            cruise.runner.thread.join(timeout=1.)

    def test_vehicle_switch_disarms_and_stops_pending_release(self):
        self.arm(); self.feature.key_press(self.event('e'))
        self.feature.key_release(self.event('e'))
        self.feature.on_vehicle_change()
        self.assertFalse(self.feature._releases)
        self.assertFalse(self.feature._armed)
        self.assertEqual(self.app.state.get('base_vy'), 0.)

    def test_removed_keys_are_ignored_when_keyboard_is_enabled(self):
        self.arm()
        count=len(self.sent)
        for key in ('z','c','Z','C'):
            self.assertIsNone(self.feature.key_press(self.event(key)))
            self.assertIsNone(self.feature.key_release(self.event(key)))
        self.assertFalse(self.feature.controller.pressed)
        self.assertFalse(self.feature._releases)
        self.assertEqual(len(self.sent),count)


class AppLifecycleTests(unittest.TestCase):
    def test_full_app_tabs_camera_sharing_and_terminal_key_isolation(self):
        cfg = copy.deepcopy(DEFAULTS)
        cfg['terminal']['auto_connect'] = False
        with patch('carapp.main.RosBridge.start'), patch('carapp.main.RosBridge.publish', return_value=True), \
             patch('carapp.main.CameraStream.start'), patch('carapp.features.terminal.local_login', return_value=''):
            app = App(cfg)
            app.root.withdraw()
            try:
                app.select_tab('remote'); app.root.update()
                self.assertEqual([f.key for f in app.features], ['gripper','remote','cruise','terminal'])
                self.assertEqual(len(app.frame_listeners), 4)  # three previews plus visual tracker
                def descendants(widget):
                    for child in widget.winfo_children():
                        yield child
                        yield from descendants(child)
                for feature in app.features[:2]:
                    controls=list(descendants(feature.cam.master))
                    self.assertFalse(any('自旋' in str(w.cget('text')) for w in controls if isinstance(w,tk.Button)))
                self.assertFalse(hasattr(app.features[0],'_spin_controller'))
                self.assertEqual(len(app.features[0].hold_btns),6)
                remote = app.features[1]
                app.state.update(ws_connected=True)
                remote.var_mode.set('keyboard'); remote.change_mode(); remote.enable_keyboard()
                remote.key_press(types.SimpleNamespace(keysym='q', state=0))
                self.assertGreater(app.state.get('base_vy'), 0.)
                app.select_tab('terminal'); app.root.update()
                self.assertFalse(remote._armed)
                self.assertEqual(app.state.get('base_vy'), 0.)
                self.assertFalse(app.root.bind('<KeyPress>'))
                self.assertTrue(app.features[3].text.bind('<KeyPress>'))
                app.select_tab('remote'); app.root.update()
                self.assertFalse(remote._armed)
            finally:
                app.on_close()

    def test_connect_always_sends_zero_before_subscriptions(self):
        app = App.__new__(App)
        app._closing = False
        app.state = CarState()
        calls = []
        app.ctx = types.SimpleNamespace(stop_base=lambda:calls.append('zero'))
        app.bridge = types.SimpleNamespace(subscribe=lambda *a, **k:calls.append('subscribe'))
        app._request_nodes = lambda:None
        app._on_ros_status(True)
        self.assertEqual(calls, ['zero','subscribe','subscribe'])

    def test_tab_lifecycle_deactivates_previous_feature(self):
        app = App.__new__(App)
        events = []
        old = types.SimpleNamespace(on_deactivate=lambda:events.append('stop-old'))
        new = types.SimpleNamespace(on_activate=lambda:events.append('enter-new'))
        app.features = [old,new]
        app._active_feature = old
        app.nb = types.SimpleNamespace(index=lambda name:1)
        app._tab_changed()
        self.assertEqual(events, ['stop-old','enter-new'])
        app._tab_changed()
        self.assertEqual(len(events), 2)

    def test_external_focus_loss_stops_all_controllers(self):
        app = App.__new__(App)
        app._closing = False; app._had_window_focus = True
        events = []
        app.features = [types.SimpleNamespace(stop_all=lambda:events.append('stop'))]
        app.root = types.SimpleNamespace(focus_get=lambda:None)
        app._check_window_focus()
        self.assertEqual(events, ['stop'])
        self.assertFalse(app._had_window_focus)


if __name__ == '__main__':
    unittest.main()
