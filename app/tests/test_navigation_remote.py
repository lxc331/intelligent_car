"""Offline navigation upload, lease, cancellation and Nav2 configuration checks."""
import ast
import copy
import json
import math
from pathlib import Path
import shlex
import threading
import time
import types
import unittest
from unittest.mock import Mock, patch

from carapp.core.navigation_config import LAUNCH_SOURCE, navigation_parameters
from carapp.core.navigation_remote import (NavigationRemote, NavigationRemoteError,
                                           remote_command, target_values, validate_bundle)
from carapp.core.navigation_remote_scripts import REMOTE_SCRIPT
from carapp.core.navigation_runtime import NavigationRuntime


MAP = dict(yaml_text='image: map.pgm\nmode: trinary\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\n',
           image_bytes=b'P5\n2 2\n255\n\xfe\xfe\x00\xcd')


class FakeRemote(NavigationRemote):
    def __init__(self):
        super().__init__('not-a-real-host.invalid','test',password='DO_NOT_PRINT')
        self.operations = []
        self.online = True
        self.interrupt_navigation = False
        self._clock_offset = 0.

    @property
    def connected(self):
        return self.online

    def _remote(self, operation, **values):
        self.operations.append((operation,values))
        if operation == 'prepare':
            return dict(server_monotonic=time.monotonic())
        if operation == 'upload':
            import base64
            return dict(written=len(base64.b64decode(values['chunk'])))
        if operation == 'start':
            return dict(running=True,owned=True,ready=False)
        if operation == 'status':
            return dict(running=True,owned=True,ready=True,goal_state='idle')
        if operation == 'navigate' and self.interrupt_navigation:
            self.request_cancel()
        return dict(running=True,goal_state='navigating',accepted=True)

    def _start_heartbeat(self):
        pass


class NavigationValidationTests(unittest.TestCase):
    def test_valid_bundle_and_safe_single_json_argument(self):
        files = validate_bundle(MAP)
        self.assertEqual(set(files),{'map.yaml','map.pgm'})
        target = "quote' ; $(never-execute)"
        args = shlex.split(remote_command('status',token='a'*32,comment=target))
        self.assertEqual(args[:7],['sudo','-n','docker','exec','-u','ubuntu','ClawPi'])
        self.assertEqual(json.loads(args[-1])['comment'],target)

    def test_reject_rotated_or_unsafe_map_before_upload(self):
        for replacement in ('image: /etc/passwd','origin: [0, 0, 0.2]',
                            'resolution: .nan','mode: raw','negate: 1'):
            bundle = dict(MAP)
            key = replacement.split(':')[0]
            bundle['yaml_text'] = '\n'.join(replacement if line.startswith(key+':') else line
                                             for line in MAP['yaml_text'].splitlines())
            with self.subTest(replacement=replacement), self.assertRaises(ValueError):
                validate_bundle(bundle)
        with self.assertRaises(ValueError):
            validate_bundle(dict(MAP,image_bytes=b'P5\n'+b'x'*4000256))

    def test_nonfinite_coordinates_and_speed_rejected_without_ssh(self):
        remote = FakeRemote()
        for value in (math.nan,math.inf,-math.inf):
            with self.assertRaises(ValueError):
                target_values(value,0,0)
        for speed in (.019,.151,math.nan):
            with self.assertRaises(ValueError):
                remote.start(MAP,speed=speed)
        self.assertFalse(remote.operations)

    def test_all_remote_payloads_compile_and_no_full_hardware_launch(self):
        compile(REMOTE_SCRIPT,'navigation RPC','exec')
        compile(LAUNCH_SOURCE,'navigation launch','exec')
        source = Path(__file__).parents[1]/'carapp/core/navigation_runtime.py'
        compile(source.read_text(encoding='utf8'),str(source),'exec')
        self.assertNotIn('bringup.launch',REMOTE_SCRIPT+LAUNCH_SOURCE)
        self.assertNotIn('killall',REMOTE_SCRIPT)
        self.assertNotIn('pkill',REMOTE_SCRIPT)
        self.assertIn("value['birth']",REMOTE_SCRIPT)
        self.assertIn('os.getpgid(pid)!=pid',REMOTE_SCRIPT)


    def test_plan_start_preview_sends_both_poses_and_grants_no_motion(self):
        remote = FakeRemote()
        remote.plan(.5, .5, 0., start=(-.5, -.5, 1.57))
        operation, values = remote.operations[-1]
        self.assertEqual(operation, 'plan')
        self.assertEqual(values['values'], dict(x=.5, y=.5, yaw=0.,
            start=dict(x=-.5, y=-.5, yaw=1.57)))
        self.assertFalse(any(name in values for name in ('allow', 'epochs', 'values_allowed')))
        remote.plan(.5, .5, 0.)
        self.assertNotIn('start', remote.operations[-1][1]['values'])
        for start in ((math.nan, 0., 0.), (-.5, -.5)):
            with self.assertRaises(ValueError):
                remote.plan(.5, .5, 0., start=start)


class NavigationParameterTests(unittest.TestCase):
    def test_vendor_configuration_copied_and_safe_overrides_applied(self):
        vendor = {'planner_server':{'ros__parameters':{'legacy':True}},
                  'amcl':{'ros__parameters':{'scan_topic':'wrong'}}}
        controller = {'controller_server':{'ros__parameters':{'FollowPath':{
            'plugin':'dwb_core::DWBLocalPlanner','critics':['BaseObstacle']}}}}
        original = copy.deepcopy((vendor,controller))
        result = navigation_parameters(vendor,controller,'/tmp/owner/map.yaml',.02)
        self.assertEqual((vendor,controller),original)
        planner = result['planner_server']['ros__parameters']['GridBased']
        self.assertTrue(planner['use_astar'])
        self.assertFalse(planner['allow_unknown'])
        self.assertEqual(planner['plugin'],'nav2_navfn_planner/NavfnPlanner')
        control = result['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(control['max_vel_x'],.02)
        self.assertLess(control['trans_stopped_velocity'],.02)
        self.assertIn('ObstacleFootprint',control['critics'])
        self.assertEqual(result['amcl']['ros__parameters']['scan_topic'],'/scan_raw')
        self.assertFalse(result['amcl']['ros__parameters']['set_initial_pose'])
        self.assertEqual(result['map_server']['ros__parameters']['yaml_filename'],'/tmp/owner/map.yaml')

    def test_every_nav2_velocity_publisher_remaps_to_private_gate(self):
        self.assertIn("('cmd_vel',raw)",LAUNCH_SOURCE)
        self.assertIn("('cmd_vel_smoothed',smooth)",LAUNCH_SOURCE)
        self.assertNotIn('controller/cmd_vel',LAUNCH_SOURCE)
        self.assertNotIn('scan.launch',LAUNCH_SOURCE)


class NavigationRemoteTests(unittest.TestCase):
    def test_uploads_bounded_chunks_and_hash_descriptors(self):
        remote = FakeRemote()
        result = remote.start(dict(MAP,image_bytes=MAP['image_bytes']+b'x'*100000),speed=.06)
        self.assertTrue(result['running'])
        self.assertFalse(remote._motion_allowed)
        uploads = [value for operation,value in remote.operations if operation=='upload']
        import base64
        self.assertTrue(all(len(base64.b64decode(value['chunk'])) <= 32768 for value in uploads))
        start = next(value for operation,value in remote.operations if operation=='start')
        self.assertEqual(set(start['files']),{'map.yaml','map.pgm','runtime.py','config_builder.py','navigation.launch.py'})
        self.assertTrue(all(len(value['sha256'])==64 for value in start['files'].values()))
        with self.assertRaises(NavigationRemoteError):
            remote.start(MAP)

    def test_cancel_before_dispatch_invalidates_ticket(self):
        remote = FakeRemote()
        remote._lease_exists = True
        ticket = remote.capture_intent()
        remote.request_cancel()
        with self.assertRaises(NavigationRemoteError):
            remote.navigate(1,2,0,intent_epoch=ticket)
        self.assertFalse(any(operation=='navigate' for operation,_ in remote.operations))
        self.assertFalse(remote._motion_allowed)

    def test_cancel_during_navigation_submission_cannot_return_success(self):
        remote = FakeRemote()
        remote._lease_exists = True
        remote.interrupt_navigation = True
        with self.assertRaises(NavigationRemoteError):
            remote.navigate(1,2,0,intent_epoch=remote.capture_intent())
        self.assertFalse(remote._motion_allowed)

    def test_out_of_band_cancel_bypasses_foreground_lock(self):
        remote = FakeRemote()
        remote._lease_exists = True
        entered,released = threading.Event(),threading.Event()
        def hold():
            with remote._lock:
                entered.set()
                released.wait(2)
        thread = threading.Thread(target=hold)
        thread.start()
        self.assertTrue(entered.wait(1))
        try:
            before = time.monotonic()
            remote.request_cancel()
            self.assertLess(time.monotonic()-before,.25)
            self.assertFalse(remote._motion_allowed)
        finally:
            released.set()
            thread.join()

    def test_navigation_grant_precedes_action_and_disconnected_status_is_unconfirmed(self):
        remote = FakeRemote()
        remote._lease_exists = True
        remote.navigate(1,2,0,intent_epoch=remote.capture_intent())
        operations = [operation for operation,_ in remote.operations]
        self.assertEqual(operations[:2],['heartbeat','navigate'])
        self.assertTrue(remote._motion_allowed)
        remote.online = False
        result = remote.status()
        self.assertFalse(remote._motion_allowed)
        self.assertFalse(result['motion_stopped'])
        self.assertEqual(result['motion_lease_seconds'],2.)

    def test_heartbeat_deadline_is_anchored_to_issue_not_server_receipt(self):
        remote = FakeRemote()
        remote._clock_offset = -900.
        with patch('carapp.core.navigation_remote.time.monotonic',return_value=1005.):
            intent = remote._intent()
        self.assertEqual(intent['expires'],107.)


class RuntimeBoundaryTests(unittest.TestCase):
    def runtime(self):
        runtime = NavigationRuntime.__new__(NavigationRuntime)
        runtime.pose_requested_at = 100.
        runtime.amcl_at = 101.
        runtime.pose_covariance_ok = True
        runtime._pose = lambda:(1.,2.,0.)
        return runtime

    def test_stationary_confirmed_amcl_does_not_expire_while_tf_remains_fresh(self):
        runtime = self.runtime()
        with patch('carapp.core.navigation_runtime.time.monotonic',return_value=1000.):
            self.assertTrue(runtime._localized())
        runtime._pose = lambda:None
        self.assertFalse(runtime._localized())
        runtime._pose = lambda:(1.,2.,0.)
        runtime.amcl_at = 99.
        self.assertFalse(runtime._localized())

    def test_velocity_relay_clamps_speed_and_never_reverses(self):
        runtime = self.runtime()
        runtime.speed = .06
        message = types.SimpleNamespace(linear=types.SimpleNamespace(x=.7,y=.8),
                                        angular=types.SimpleNamespace(z=.9))
        runtime._velocity(message)
        self.assertEqual(runtime.command,(.06,0.,.35))
        message.linear.x = -.4
        runtime._velocity(message)
        self.assertEqual(runtime.command,(0.,0.,.35))

    def test_invalid_velocity_revokes_instead_of_publishing(self):
        runtime = self.runtime()
        runtime.speed = .06
        runtime._revoke = Mock()
        message = types.SimpleNamespace(linear=types.SimpleNamespace(x=math.nan,y=0.),
                                        angular=types.SimpleNamespace(z=0.))
        runtime._velocity(message)
        runtime._revoke.assert_called_once()

    def test_delayed_heartbeat_cannot_extend_expired_motion_lease(self):
        runtime = self.runtime()
        runtime.goal_state = 'navigating'
        runtime.goal_handle = object()
        runtime.goal_epoch = 3
        runtime.revoked_epoch = 2
        runtime.lease = dict(at=999.9,expires=999.,allow=True,epoch=3)
        runtime.command = (.06,0.,0.)
        runtime.command_at = 999.9
        runtime.last_sent = False
        runtime._ready = lambda:True
        runtime._revoke = Mock()
        runtime.base = types.SimpleNamespace(publish=Mock())
        message_module = types.ModuleType('geometry_msgs.msg')
        message_module.Twist = type('Twist',(),{})
        with patch.dict('sys.modules',{'geometry_msgs':types.ModuleType('geometry_msgs'),
                                       'geometry_msgs.msg':message_module}), \
                patch('carapp.core.navigation_runtime.time.monotonic',return_value=1000.):
            runtime._drive()
        runtime._revoke.assert_called_once()
        runtime.base.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
