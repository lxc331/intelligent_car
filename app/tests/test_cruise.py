"""Vendor lidar adapter, transport failure and cancellation tests; no vehicle access."""
import copy
import math
import time
import types
import unittest
from carapp.config import DEFAULTS
from carapp.core.cruise import CruisePolicy,CruiseRunner,CruiseSettings,LaserMonitor
from carapp.features.base import AppContext
from carapp.model.state import CarState


def scan_message(ranges=None,sequence=1):
    return dict(header=dict(frame_id="lidar_frame",stamp=dict(sec=100,nanosec=sequence)),
                angle_min=0.,angle_increment=math.tau/400,range_min=.1,range_max=10.,
                ranges=ranges if ranges is not None else [4.]*400)


def scan(now=100.,sequence=1,left=4.,right=4.,target=None):
    message=scan_message(sequence=sequence)
    message['ranges'][10]=left;message['ranges'][390]=right
    if target:message['ranges'][target[0]]=target[1]
    monitor=LaserMonitor();monitor.update(message,now)
    return dict(monitor.snapshot(),sequence=sequence)


class LaserTests(unittest.TestCase):
    def test_raw_order_null_and_far_points_are_not_interpolated_or_projected(self):
        message=scan_message();message['ranges'][8]=None;message['ranges'][22]=.5
        monitor=LaserMonitor();monitor.update(message,20.)
        result=monitor.snapshot()
        self.assertIsInstance(result['ranges'],tuple)
        self.assertIsNone(result['ranges'][8]);self.assertEqual(result['ranges'][22],.5)
        self.assertEqual(result['left'],.5);self.assertNotIn('visibility_rays',result)
        self.assertNotIn('straight_corridor',result)

    def test_nonfinite_zero_negative_and_out_of_range_points_are_not_free(self):
        for invalid in (None,math.inf,-math.inf,math.nan,0.,-1.,15.):
            monitor=LaserMonitor();monitor.update(scan_message([invalid]*400),20.)
            self.assertTrue(all(value is None for value in monitor.snapshot()['ranges']))
            result=CruisePolicy().decide(monitor.snapshot(),CruiseSettings(),20.)
            self.assertEqual(result[:2],(0.,0.));self.assertTrue(result[3])

    def test_positive_below_minimum_remains_a_close_reading(self):
        message=scan_message();message['ranges'][10]=.05
        monitor=LaserMonitor();monitor.update(message,20.)
        self.assertEqual(monitor.snapshot()['ranges'][10],.05)

    def test_repeated_timestamp_does_not_refresh_receipt_or_sequence(self):
        monitor=LaserMonitor();message=scan_message()
        self.assertTrue(monitor.update(message,20.));self.assertFalse(monitor.update(message,22.))
        self.assertEqual(monitor.snapshot()['received_at'],20.)
        policy = CruisePolicy()
        self.assertFalse(policy.decide(monitor.snapshot(),CruiseSettings(),22.)[3])
        self.assertTrue(policy.waiting_for_scan)

    def test_partial_bad_frame_or_negative_increment_is_rejected(self):
        for message in ({},scan_message([1.]*30),dict(scan_message(),angle_increment=0.),
                        dict(scan_message(),angle_increment=-.01),
                        dict(scan_message(),header=dict(frame_id='wrong',stamp=dict(sec=1,nanosec=1)))):
            monitor=LaserMonitor();monitor.update(message,20.);self.assertTrue(monitor.snapshot()['error'])


class PolicyTests(unittest.TestCase):
    def test_single_distance_and_vendor_parameter_ranges(self):
        for distance in (.3,.6,1.5):self.assertEqual(CruiseSettings(distance).detection_distance,distance)
        for args in ({'detection_distance':.29},{'detection_distance':1.51},{'detection_distance':math.nan},
                     {'speed':0.},{'speed':.16},{'scan_angle':91.},{'mode':'invalid'}):
            with self.assertRaises(ValueError):CruiseSettings(**args)
        self.assertNotIn('side_distance',CruiseSettings().__dict__)

    def test_avoidance_uses_vendor_forward_and_turn_formula(self):
        settings=CruiseSettings(.6,.06)
        self.assertEqual(CruisePolicy().decide(scan(),settings,100.)[:2],(.06,0.))
        self.assertEqual(CruisePolicy().decide(scan(left=.4),settings,100.)[:2],(.01,-.36))
        self.assertEqual(CruisePolicy().decide(scan(right=.4),settings,100.)[:2],(.01,.36))
        self.assertEqual(CruisePolicy().decide(scan(left=.4,right=.4),settings,100.)[:2],(.01,.36))

    def test_vendor_timed_turn_is_held_until_next_sensor_frame_after_deadline(self):
        policy=CruisePolicy();settings=CruiseSettings()
        result=policy.decide(scan(left=.4),settings,100.)
        self.assertEqual(policy.decide(scan(now=100.2,sequence=2),settings,100.2)[:2],result[:2])
        deadline=policy.algorithm.timestamp
        repeated=scan(now=deadline-.1,sequence=3)
        self.assertEqual(policy.decide(repeated,settings,deadline-.1)[:2],result[:2])
        self.assertEqual(policy.decide(repeated,settings,deadline+.1)[:2],result[:2])
        self.assertEqual(policy.decide(scan(now=deadline+.2,sequence=4),settings,deadline+.2)[:2],(.06,0.))

    def test_no_custom_rear_geometry_gate_or_twenty_second_timeout(self):
        policy=CruisePolicy();settings=CruiseSettings()
        message=scan_message();message['ranges'][200]=.05
        monitor=LaserMonitor();monitor.update(message,100.)
        self.assertEqual(policy.decide(monitor.snapshot(),settings,100.)[:2],(.06,0.))
        self.assertFalse(policy.decide(scan(now=135.,sequence=2),settings,135.)[3])

    def test_follow_moves_forward_and_backward_guard_only_rotates(self):
        forward=CruisePolicy().decide(scan(target=(25,.45)),CruiseSettings(mode='follow'),100.)
        backward=CruisePolicy().decide(scan(target=(25,.1)),CruiseSettings(mode='follow'),100.)
        guard=CruisePolicy().decide(scan(target=(25,.45)),CruiseSettings(mode='guard'),100.)
        self.assertGreater(forward[0],0.);self.assertLess(backward[0],0.)
        self.assertEqual(guard[0],0.);self.assertGreater(guard[1],0.)

    def test_follow_and_guard_stop_when_nearest_target_is_outside_range(self):
        for mode in ('follow','guard'):
            self.assertEqual(CruisePolicy().decide(scan(left=.6),CruiseSettings(mode=mode),100.)[:2],(0.,0.))

    def test_follow_fixed_vendor_distance_deadband_is_preserved(self):
        result=CruisePolicy().decide(scan(target=(0,.2)),CruiseSettings(mode='follow'),100.)
        self.assertEqual(result[:2],(0.,0.))

    def test_repeated_frame_does_not_advance_pid_or_timestamp(self):
        policy=CruisePolicy();settings=CruiseSettings(mode='follow')
        sample=scan(target=(25,.45));first=policy.decide(sample,settings,100.)
        last_time=policy.algorithm.pid_dist.last_time
        self.assertEqual(policy.decide(sample,settings,100.1),first)
        self.assertEqual(policy.algorithm.pid_dist.last_time,last_time)

    def test_lost_scan_pauses_even_during_a_vendor_turn(self):
        policy=CruisePolicy();settings=CruiseSettings()
        sample=scan(left=.4);policy.decide(sample,settings,100.)
        result=policy.decide(sample,settings,101.)
        self.assertEqual(result[:2],(0.,0.));self.assertFalse(result[3])
        self.assertTrue(policy.waiting_for_scan)
        resumed=policy.decide(scan(now=101.1,sequence=2),settings,101.1)
        self.assertEqual(resumed[:2],(.06,0.));self.assertFalse(resumed[3])
        self.assertFalse(policy.waiting_for_scan)

    def test_diagnostics_identify_the_actual_decision_frame_and_mode(self):
        policy=CruisePolicy();sample=scan(sequence=7,target=(25,.45))
        policy.decide(sample,CruiseSettings(mode='guard'),100.)
        self.assertEqual(policy.diagnostics['scan_sequence'],7)
        self.assertEqual(policy.diagnostics['mode'],'guard')
        self.assertEqual(policy.diagnostics['target_distance'],.45)
        self.assertIs(policy.last_scan,sample)


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.sent=[];self.now=100.
        self.app=types.SimpleNamespace(cfg=copy.deepcopy(DEFAULTS),state=CarState(),log=lambda text:None,
            bridge=types.SimpleNamespace(connected=True,publish=lambda topic,msg:self.sent.append(msg) or True))
        self.app.state.update(ws_connected=True,cam_connected=True)
        self.monitor=LaserMonitor();self.monitor.update(scan_message(),100.)
        self.runner=CruiseRunner(AppContext(self.app),self.monitor,clock=lambda:self.now)

    def tearDown(self):self.runner.stop()

    def test_scan_gap_pauses_and_resumes_without_restarting_session(self):
        self.runner.start(CruiseSettings());time.sleep(.03)
        self.assertTrue(any(m['linear']['x']>0 for m in self.sent))
        self.now=101.;time.sleep(.15);self.assertTrue(self.runner.running)
        self.assertFalse(self.runner.cancel.is_set())
        self.assertEqual(self.sent[-1]['linear']['x'],0.)
        self.now=102.;self.monitor.update(scan_message(sequence=2),102.)
        time.sleep(.12);self.assertTrue(self.runner.running)
        self.assertGreater(self.sent[-1]['linear']['x'],0.)

    def test_camera_ros_and_publish_failures_cancel(self):
        for failure in ('ros','camera','publish'):
            self.runner.running=True;self.runner.cancel.clear()
            self.app.state.update(ws_connected=failure!='ros',cam_connected=failure!='camera')
            self.app.bridge.publish=lambda *args:failure!='publish'
            self.runner.step();self.assertFalse(self.runner.running)

    def test_all_three_modes_publish_zero_lateral_and_stop_cleanly(self):
        for mode in ('avoid','follow','guard'):
            self.runner.configure(CruiseSettings(mode=mode))
            message=scan_message(sequence=3);message['ranges'][25]=.45
            self.monitor.update(message,100.);self.runner.running=True;self.runner.cancel.clear()
            self.runner.step();self.assertEqual(self.sent[-1]['linear']['y'],0.)
            if mode=='guard':self.assertEqual(self.sent[-1]['linear']['x'],0.)
            self.runner.stop();self.assertEqual(self.app.state.get('base_cmd'),(0.,0.))

    def test_reconfigure_while_running_stops_and_replaces_old_policy(self):
        self.runner.start(CruiseSettings());old=self.runner.policy
        self.runner.configure(CruiseSettings(mode='guard'))
        self.assertIsNot(self.runner.policy,old)
        self.assertEqual(self.runner.settings.mode,'guard')
        self.assertEqual(self.app.state.get('base_cmd'),(0.,0.))
        self.runner.stop();count=len(self.sent);time.sleep(.12);self.assertEqual(len(self.sent),count)

    def test_start_refuses_unready_camera_or_invalid_scan(self):
        self.app.state.update(cam_connected=False)
        with self.assertRaises(ValueError):self.runner.start(CruiseSettings())
        self.app.state.update(cam_connected=True);self.monitor.reset()
        with self.assertRaises(ValueError):self.runner.start(CruiseSettings())

    def test_scan_that_expires_during_calculation_never_reaches_transport(self):
        def slow(*args):self.now=101.;return .04,.15,'turn',False
        self.runner.policy.decide=slow;self.runner.running=True
        self.runner.step();self.assertTrue(self.runner.running)
        self.assertEqual(self.sent[-1]['linear']['x'],0.)


if __name__=='__main__':unittest.main()
