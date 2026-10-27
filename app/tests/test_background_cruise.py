"""Background autonomy, temporary scan gaps and explicit stops; no vehicle I/O."""
import copy
import math
import types
import unittest
from unittest.mock import Mock, patch

from carapp.config import DEFAULTS
from carapp.core.cruise import CruisePolicy, CruiseRunner, CruiseSettings
from carapp.core.laser_scan import LaserMonitor
from carapp.features.base import AppContext
from carapp.features.cruise import CruiseFeature
from carapp.features.gripper import GripperFeature
from carapp.main import App
from carapp.model.state import CarState


def scan(sequence=1, distance=4.):
    ranges=[4.]*400
    ranges[10]=distance
    return dict(header=dict(frame_id='lidar_frame',stamp=dict(sec=10,nanosec=sequence)),
                angle_min=0.,angle_increment=math.tau/400,range_min=.1,range_max=10.,ranges=ranges)


class BackgroundCruiseTests(unittest.TestCase):
    def setUp(self):
        self.now=100.
        self.sent=[]
        self.app=App.__new__(App)
        self.app.cfg=copy.deepcopy(DEFAULTS)
        self.app.state=CarState()
        self.app.state.update(ws_connected=True,cam_connected=True)
        self.app.log=Mock()
        self.app.bridge=types.SimpleNamespace(connected=True,publish=lambda topic,msg:self.sent.append(msg) or True)
        self.app.ctx=AppContext(self.app)
        self.cruise=CruiseFeature(self.app.ctx)
        self.app.features=[self.cruise]
        self.cruise.monitor.update(scan(),self.now)
        self.cruise.runner.clock=lambda:self.now
        self.cruise.runner.running=True

    def tearDown(self):
        self.cruise.stop_all()

    def test_tab_exit_keeps_motion_owner_and_recording(self):
        self.cruise.active=True
        self.cruise.record_dir='record remains owned'
        with patch.object(self.cruise,'stop_all') as stop:
            self.cruise.on_deactivate()
        self.assertFalse(self.cruise.active)
        self.assertTrue(self.cruise.motion_active)
        self.assertEqual(self.cruise.record_dir,'record remains owned')
        stop.assert_not_called()
        self.cruise.record_dir=None

    def test_window_focus_loss_keeps_cruise_and_stops_only_active_manual_motion(self):
        manual=types.SimpleNamespace(motion_active=True,stop_all=Mock())
        idle=types.SimpleNamespace(motion_active=False,stop_all=Mock())
        self.app.features += [manual,idle]
        self.app._closing=False
        self.app._had_window_focus=True
        self.app._focus_timer='pending'
        self.app.root=object()
        with patch('carapp.main.window_has_focus',return_value=False),patch.object(self.cruise,'stop_all') as stop:
            self.app._check_window_focus()
        stop.assert_not_called()
        manual.stop_all.assert_called_once()
        idle.stop_all.assert_not_called()
        self.assertTrue(self.cruise.runner.running)
        self.assertEqual(self.sent,[])

    def test_scan_gap_keeps_session_and_claim_then_new_scan_resumes(self):
        runner=self.cruise.runner
        runner.step()
        self.assertGreater(self.sent[-1]['linear']['x'],0.)
        self.now=101.
        runner.step()
        self.assertTrue(runner.running)
        self.assertTrue(self.app.ctx.motion_in_use())
        self.assertTrue(runner.policy.waiting_for_scan)
        self.assertFalse(runner.cancel.is_set())
        self.assertEqual(self.sent[-1]['linear']['x'],0.)
        # Replaying the old car timestamp cannot make an old scan fresh.
        self.cruise.monitor.update(scan(),101.)
        runner.step()
        self.assertEqual(self.sent[-1]['linear']['x'],0.)
        self.cruise.monitor.update(scan(2),101.)
        runner.step()
        self.assertFalse(runner.policy.waiting_for_scan)
        self.assertGreater(self.sent[-1]['linear']['x'],0.)

    def test_explicit_global_stop_prevents_resume_after_new_scan(self):
        self.now=101.
        self.cruise.runner.step()
        self.app.stop_all_motion()
        count=len(self.sent)
        self.cruise.monitor.update(scan(2),101.)
        self.cruise.runner.step()
        self.assertFalse(self.cruise.motion_active)
        self.assertEqual(len(self.sent),count)
        self.assertEqual(self.sent[-1]['linear']['x'],0.)

    def test_gripper_cannot_start_chassis_actions_while_cruise_owns_it(self):
        gripper=GripperFeature(self.app.ctx)
        self.app.features.append(gripper)
        gripper.jog(1,0)
        gripper.nudge(1,0,.3)
        gripper.run_pick_sequence()
        gripper.stop_wheels()  # Unowned release/leave events must not inject a stop.
        gripper.on_deactivate()
        self.assertIsNone(gripper._motion_thread)
        self.assertFalse(gripper.motion_active)
        self.assertEqual(self.sent,[])

    def test_initial_stale_scan_cannot_start_a_new_session(self):
        runner=self.cruise.runner
        runner.running=False
        self.now=101.
        with self.assertRaisesRegex(ValueError,'新雷达数据'):
            runner.start(CruiseSettings())
        self.assertFalse(runner.running)

    def test_scan_gap_cannot_hide_real_disconnect_or_bad_scan(self):
        runner=self.cruise.runner
        self.now=101.
        runner.step()
        self.app.bridge.connected=False
        runner.step()
        self.assertFalse(runner.running)
        self.app.bridge.connected=True
        self.cruise.monitor.update(scan(2),101.)
        runner.step()
        self.assertFalse(runner.running)
        runner.running=True
        runner.cancel.clear()
        self.cruise.monitor.update({},101.)
        runner.step()
        self.assertFalse(runner.running)
        self.assertEqual(self.sent[-1]['linear']['x'],0.)

    def test_resume_reinitializes_timed_turn_and_pid_for_each_mode(self):
        for mode in ('avoid','follow','guard'):
            monitor=LaserMonitor()
            policy=CruisePolicy()
            settings=CruiseSettings(mode=mode)
            monitor.update(scan(1,.4),100.)
            policy.decide(monitor.snapshot(),settings,100.)
            old=policy.algorithm
            self.assertFalse(policy.decide(monitor.snapshot(),settings,101.)[3])
            monitor.update(scan(2,.45),101.)
            actual=policy.decide(monitor.snapshot(),settings,101.)
            expected=CruisePolicy().decide(monitor.snapshot(),settings,101.)
            self.assertEqual(actual,expected)
            self.assertIsNot(policy.algorithm,old)


if __name__=='__main__':
    unittest.main()
