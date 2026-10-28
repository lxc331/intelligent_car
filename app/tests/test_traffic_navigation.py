"""Red-light waits retain Nav2 intent without bypassing stop/lease revocation."""
import types
import unittest
from concurrent.futures import Future
from unittest.mock import Mock, patch

from carapp.core.navigation_remote import NavigationRemote
from carapp.core.navigation_remote_scripts import REMOTE_SCRIPT
from carapp.core.navigation_runtime import NavigationRuntime


class TrafficNavigationTests(unittest.TestCase):
    def runtime(self):
        runtime = NavigationRuntime.__new__(NavigationRuntime)
        runtime.goal_epoch = 7
        runtime.revoked_epoch = 6
        runtime.lease = dict(allow=True, epoch=7, expires=102., paused=True,
                             pause_reason='前方红灯')
        runtime.goal_state = 'navigating'
        runtime.goal_handle = Mock()
        runtime.cancel_future = None
        runtime._traffic_paused = False
        runtime._resume_target = (1., 2., .3)
        runtime._navigation_accept_pending = False
        runtime.command = (.06, 0., 0.)
        runtime.command_at = 100.
        runtime.last_sent = True
        runtime._ready = lambda: True
        runtime._zero = Mock()
        return runtime

    def test_adapter_reads_latest_interlock_for_each_heartbeat(self):
        interlock = dict(enabled=True, blocked=True, reason='前方红灯')
        remote = NavigationRemote('unused.invalid', 'user', motion_interlock=lambda: interlock)
        first = remote._intent()
        self.assertTrue(first['paused'])
        self.assertEqual(first['pause_reason'], '前方红灯')
        interlock['blocked'] = False
        self.assertFalse(remote._intent()['paused'])
        interlock.update(enabled=False, blocked=True)
        self.assertFalse(remote._intent()['paused'])

    def test_interlock_callback_failure_holds_without_granting_motion(self):
        remote = NavigationRemote('unused.invalid', 'user', motion_interlock=Mock(side_effect=ValueError))
        intent = remote._intent()
        self.assertTrue(intent['paused'])
        self.assertFalse(intent['allow'])

    def test_default_adapter_keeps_existing_lease_format(self):
        self.assertNotIn('paused', NavigationRemote('unused.invalid', 'user')._intent())
        self.assertIn("paused=bool(p.get('paused'))", REMOTE_SCRIPT)
        compile(REMOTE_SCRIPT, 'navigation lease RPC', 'exec')

    def test_speed_limit_is_carried_only_when_switch_is_enabled(self):
        interlock = dict(enabled=True, blocked=False, speed_limit=.1, reason='斑马线路口减速')
        remote = NavigationRemote('unused.invalid', 'user', motion_interlock=lambda: interlock)
        self.assertEqual(remote._intent()['traffic_speed_limit'], .1)
        interlock['enabled'] = False
        intent = remote._intent()
        self.assertFalse(intent['traffic_enabled'])
        self.assertFalse(intent['paused'])
        self.assertIsNone(intent['traffic_speed_limit'])

    def test_runtime_limits_only_existing_commands_and_disabled_ignores_stale_traffic_fields(self):
        runtime = self.runtime()
        runtime.lease.update(paused=False, traffic_enabled=True, traffic_speed_limit=.1)
        runtime.command = (.15, 0., .3)
        runtime.base = types.SimpleNamespace(publish=Mock())
        runtime._revoke = Mock()
        class Twist:
            def __init__(self):
                self.linear = types.SimpleNamespace(x=0., y=0., z=0.)
                self.angular = types.SimpleNamespace(x=0., y=0., z=0.)
        module = types.ModuleType('geometry_msgs.msg')
        module.Twist = Twist
        with patch.dict('sys.modules', {'geometry_msgs': types.ModuleType('geometry_msgs'),
                                       'geometry_msgs.msg': module}), \
             patch('carapp.core.navigation_runtime.time.monotonic', return_value=100.):
            runtime._drive()
            sent = runtime.base.publish.call_args.args[0]
            self.assertAlmostEqual(sent.linear.x, .1)
            self.assertAlmostEqual(sent.angular.z, .2)
            self.assertEqual(runtime.command, (.15, 0., .3))
            runtime.lease.update(traffic_enabled=False, paused=True)
            runtime._drive()
            sent = runtime.base.publish.call_args.args[0]
            self.assertAlmostEqual(sent.linear.x, .15)
            self.assertAlmostEqual(sent.angular.z, .3)
            runtime._revoke.assert_not_called()
            runtime.lease['allow'] = False
            runtime._drive()
            runtime._revoke.assert_called_once()

    def test_invalid_scene_limit_revokes_navigation(self):
        runtime = self.runtime()
        runtime.lease.update(paused=False, traffic_enabled=True, traffic_speed_limit=float('nan'))
        runtime.base = types.SimpleNamespace(publish=Mock())
        runtime._revoke = Mock()
        module = types.ModuleType('geometry_msgs.msg')
        module.Twist = type('Twist', (), {})
        with patch.dict('sys.modules', {'geometry_msgs': types.ModuleType('geometry_msgs'),
                                       'geometry_msgs.msg': module}), \
             patch('carapp.core.navigation_runtime.time.monotonic', return_value=100.):
            runtime._drive()
        runtime._revoke.assert_called_once()
        runtime.base.publish.assert_not_called()

    def test_hold_sends_zero_and_cancels_nav2_once_retaining_target(self):
        runtime = self.runtime()
        runtime._pause_for_traffic()
        runtime._pause_for_traffic()
        runtime._zero.assert_called()
        runtime.goal_handle.cancel_goal_async.assert_called_once()
        self.assertEqual(runtime.goal_state, 'paused')
        self.assertEqual(runtime._resume_target, (1., 2., .3))
        self.assertEqual(runtime.revoked_epoch, 6)
        self.assertIsNone(runtime.command)

    def test_hold_blocks_drive_without_revoking_valid_lease(self):
        runtime = self.runtime()
        runtime.base = types.SimpleNamespace(publish=Mock())
        runtime._revoke = Mock()
        module = types.ModuleType('geometry_msgs.msg')
        module.Twist = type('Twist', (), {})
        with patch.dict('sys.modules', {'geometry_msgs': types.ModuleType('geometry_msgs'),
                                       'geometry_msgs.msg': module}), \
             patch('carapp.core.navigation_runtime.time.monotonic', return_value=100.):
            runtime._drive()
        runtime.base.publish.assert_not_called()
        runtime._zero.assert_called_once()
        runtime._revoke.assert_not_called()

    def test_resume_waits_for_cancel_result_and_clear_frame(self):
        runtime = self.runtime()
        runtime._pause_for_traffic()
        runtime._navigate = Mock()
        runtime._resume_after_traffic(100.)
        runtime.lease['paused'] = False
        runtime._resume_after_traffic(100.)
        runtime._navigate.assert_not_called()
        runtime.goal_handle = None
        runtime._resume_after_traffic(100.)
        runtime._navigate.assert_called_once_with(dict(epoch=7, internal_resume=True), (1., 2., .3))

    def test_stop_revokes_waiting_goal_and_cannot_resume(self):
        runtime = self.runtime()
        runtime._pause_for_traffic()
        runtime.goal_handle = None
        runtime._revoke('user stop')
        runtime.lease['paused'] = False
        runtime._navigate = Mock()
        runtime._resume_after_traffic(100.)
        runtime._navigate.assert_not_called()
        self.assertIsNone(runtime._resume_target)
        self.assertEqual(runtime.revoked_epoch, 7)

    def test_lease_expiry_new_epoch_or_bad_sensors_prevent_resume(self):
        for condition in ('expiry', 'epoch', 'sensor', 'quitting'):
            with self.subTest(condition=condition):
                runtime = self.runtime()
                runtime._pause_for_traffic()
                runtime.goal_handle = None
                runtime.lease['paused'] = False
                if condition == 'expiry':
                    runtime.lease['expires'] = 99.
                elif condition == 'epoch':
                    runtime.lease['epoch'] = 8
                elif condition == 'quitting':
                    runtime.quitting = True
                else:
                    runtime._ready = lambda: False
                runtime._navigate = Mock()
                runtime._resume_after_traffic(100.)
                runtime._navigate.assert_not_called()
                self.assertIsNone(runtime._resume_target)

    def test_long_wait_result_is_paused_and_resumes_same_goal(self):
        runtime = self.runtime()
        runtime.goal_handle = None
        runtime._traffic_paused = False
        runtime._stamped = lambda *target: target
        runtime._reply = Mock()
        send = Future()
        runtime.navigator = types.SimpleNamespace(send_goal_async=Mock(return_value=send))
        runtime._feedback = Mock()
        module = types.ModuleType('nav2_msgs.action')
        module.NavigateToPose = types.SimpleNamespace(Goal=type('Goal', (), {}))
        result = Future()
        handle = Mock(accepted=True)
        handle.get_result_async.return_value = result
        with patch.dict('sys.modules', {'nav2_msgs': types.ModuleType('nav2_msgs'),
                                       'nav2_msgs.action': module}), \
             patch('carapp.core.navigation_runtime.time.monotonic', return_value=100.):
            runtime._navigate(dict(epoch=7, request_id='original'), (1., 2., .3))
            send.set_result(handle)
            self.assertEqual(runtime.goal_state, 'paused')
            result.set_result(types.SimpleNamespace(status=5))
            self.assertIsNone(runtime.goal_handle)
            self.assertEqual(runtime.goal_state, 'paused')
            self.assertEqual(runtime.revoked_epoch, 6)
            runtime._navigate = Mock()
            runtime.lease['paused'] = False
            runtime._resume_after_traffic(100.)
            runtime._navigate.assert_called_once()

    def _pending_action(self, runtime):
        runtime.goal_handle = None
        runtime._traffic_paused = False
        runtime._stamped = lambda *target: target
        runtime._reply = Mock()
        send, result = Future(), Future()
        runtime.navigator = types.SimpleNamespace(send_goal_async=Mock(return_value=send))
        runtime._feedback = Mock()
        module = types.ModuleType('nav2_msgs.action')
        module.NavigateToPose = types.SimpleNamespace(Goal=type('Goal', (), {}))
        handle = Mock(accepted=True)
        handle.get_result_async.return_value = result
        return module, send, result, handle

    def test_success_or_aborted_result_during_red_wait_does_not_replay(self):
        for code, state in ((4, 'succeeded'), (6, 'aborted')):
            with self.subTest(code=code):
                runtime = self.runtime()
                module, send, result, handle = self._pending_action(runtime)
                with patch.dict('sys.modules', {'nav2_msgs': types.ModuleType('nav2_msgs'),
                                               'nav2_msgs.action': module}), \
                     patch('carapp.core.navigation_runtime.time.monotonic', return_value=100.):
                    runtime._navigate(dict(epoch=7, request_id='original'), (1., 2., .3))
                    send.set_result(handle)
                    self.assertEqual(runtime.goal_state, 'paused')
                    result.set_result(types.SimpleNamespace(status=code))
                    self.assertEqual(runtime.goal_state, state)
                    self.assertIsNone(runtime._resume_target)
                    runtime._navigate = Mock()
                    runtime.lease['paused'] = False
                    runtime._resume_after_traffic(100.)
                    runtime._navigate.assert_not_called()

    def test_user_stop_before_nav2_acceptance_discards_red_wait_target(self):
        runtime = self.runtime()
        module, send, result, handle = self._pending_action(runtime)
        with patch.dict('sys.modules', {'nav2_msgs': types.ModuleType('nav2_msgs'),
                                       'nav2_msgs.action': module}), \
             patch('carapp.core.navigation_runtime.time.monotonic', return_value=100.):
            runtime._navigate(dict(epoch=7, request_id='original'), (1., 2., .3))
            self.assertTrue(runtime._navigation_accept_pending)
            runtime._pause_for_traffic()
            runtime._revoke('user stop')
            send.set_result(handle)
            self.assertIn(runtime.goal_state, ('canceled', 'canceling'))
            handle.cancel_goal_async.assert_called_once()
            result.set_result(types.SimpleNamespace(status=5))
            self.assertEqual(runtime.goal_state, 'canceled')
            self.assertIsNone(runtime._resume_target)
            runtime._navigate = Mock()
            runtime.lease['paused'] = False
            runtime._resume_after_traffic(100.)
            runtime._navigate.assert_not_called()


if __name__ == '__main__':
    unittest.main()
