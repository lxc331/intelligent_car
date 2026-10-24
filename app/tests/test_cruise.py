"""Cruise safety and obstacle simulation; these tests never connect to a vehicle."""
import copy
import json
import math
import tempfile
import time
import tkinter as tk
import types
import unittest
from pathlib import Path
from unittest.mock import patch

from carapp.config import DEFAULTS, load_config, save_config
from carapp.core.cruise import CruisePolicy, CruiseRunner, CruiseSettings, LaserMonitor
from carapp.features.base import AppContext
from carapp.features.cruise import CruiseFeature
from carapp.main import App
from carapp.model.state import CarState


def scan_message(ranges=None, sequence=1):
    return dict(header=dict(frame_id="lidar_frame", stamp=dict(sec=100, nanosec=sequence)),
                angle_min=0., angle_increment=math.tau/400, range_min=.1, range_max=10.,
                ranges=ranges if ranges is not None else [4.]*400)


def view(front=4., left=4., right=4., now=100., sequence=1, rear=4.):
    return dict(front=front, left=left, right=right, rear=rear,
                received_at=now, sequence=sequence, error="")


class LaserTests(unittest.TestCase):
    def test_off_axis_leg_outside_body_corridor_does_not_block_forward(self):
        msg=scan_message();msg['ranges'][44]=.6  # 39.6 degrees, outside the 44 cm corridor.
        monitor=LaserMonitor();monitor.update(msg,20.)
        self.assertGreater(monitor.snapshot()['front'],3.)
        self.assertEqual(monitor.snapshot()['left'],.6)
        msg=scan_message(sequence=2);msg['ranges'][22]=.5
        monitor.update(msg,20.)
        self.assertAlmostEqual(monitor.snapshot()['front'],.5*math.cos(math.radians(19.8)))

    def test_sectors_respect_angle_origin_and_not_array_halves(self):
        distances = [4.]*400
        distances[0], distances[100], distances[300] = .8, .5, .9
        monitor = LaserMonitor(); monitor.update(scan_message(distances), 20.)
        result = monitor.snapshot()
        self.assertEqual((result['front'],result['left'],result['right']), (.8,.5,.9))
        msg = scan_message(distances[::-1]);msg['angle_min']=math.tau-math.tau/400;msg['angle_increment']*=-1
        monitor.reset();monitor.update(msg, 20.)
        self.assertEqual(monitor.snapshot()['left'], .5)

    def test_rear_sector_wraps_across_negative_and_positive_angles(self):
        for index in (180, 200, 220):
            msg=scan_message();msg['ranges'][index]=.28
            monitor=LaserMonitor();monitor.update(msg,20.)
            self.assertEqual(monitor.snapshot()['rear'],.28)
        msg=scan_message(sequence=2)
        for index in range(161,240):msg['ranges'][index]=None
        monitor.update(msg,21.)
        self.assertIsNone(monitor.snapshot()['rear'])

    def test_side_ahead_sectors_do_not_confuse_passed_obstacles_with_new_hazards(self):
        msg=scan_message();msg['ranges'][140]=.34;msg['ranges'][260]=.35  # rear-side obstacles
        monitor=LaserMonitor();monitor.update(msg,20.)
        scan=monitor.snapshot()
        self.assertEqual(scan['left'],.34);self.assertEqual(scan['right'],.35)
        self.assertEqual(scan['left_ahead'],4.);self.assertEqual(scan['right_ahead'],4.)
        msg=scan_message(sequence=2);msg['ranges'][70]=.4;msg['ranges'][330]=.5
        monitor.update(msg,20.1);scan=monitor.snapshot()
        self.assertEqual(scan['left_ahead'],.4);self.assertEqual(scan['right_ahead'],.5)

    def test_null_nan_zero_invalid_not_free_and_sparse_sector_rejected(self):
        for invalid in (None, float('nan'), 0., -1., -math.inf, 15.):
            monitor = LaserMonitor();monitor.update(scan_message([invalid]*400), 20.)
            self.assertIsNone(monitor.snapshot()['front'])
            self.assertTrue(monitor.snapshot()['error'])

    def test_positive_infinity_is_explicit_clear_range_and_close_saturation_is_hazard(self):
        monitor = LaserMonitor();msg = scan_message([math.inf]*400)
        monitor.update(msg, 20.);self.assertEqual(monitor.snapshot()['front'], 10.)
        msg=scan_message(sequence=2);msg['ranges'][0]=.05
        monitor.update(msg, 20.1);self.assertEqual(monitor.snapshot()['front'], .05)

    def test_large_blind_gap_stops_even_with_majority_valid_samples(self):
        msg=scan_message()
        for i in range(16):msg['ranges'][i]=None
        monitor=LaserMonitor();monitor.update(msg, 20.)
        self.assertIsNone(monitor.snapshot()['front'])

    def test_repeated_sensor_stamp_does_not_refresh_receipt_time(self):
        monitor=LaserMonitor();msg=scan_message()
        self.assertTrue(monitor.update(msg,20.))
        self.assertFalse(monitor.update(msg,22.))
        self.assertEqual(monitor.snapshot()['received_at'],20.)
        self.assertTrue(CruisePolicy().decide(monitor.snapshot(),CruiseSettings(),22.)[3])

    def test_bad_frame_malformed_or_partial_scan_is_rejected(self):
        bad=[{},scan_message([1.]*30),dict(scan_message(),angle_increment=0.),
             dict(scan_message(),header=dict(frame_id='wrong',stamp=dict(sec=1,nanosec=1)))]
        for message in bad:
            monitor=LaserMonitor();monitor.update(message,20.)
            self.assertTrue(monitor.snapshot()['error'])


class PolicyTests(unittest.TestCase):
    def test_open_path_moves_forward_at_selected_speed(self):
        self.assertEqual(CruisePolicy().decide(view(),CruiseSettings(.6,.10),100.)[:2],(.10,0.))
        self.assertEqual(CruisePolicy().decide(view(.7),CruiseSettings(.6,.15),100.)[0],.06)

    def test_obstacle_keeps_forward_motion_and_turns_to_clear_side(self):
        policy=CruisePolicy();settings=CruiseSettings()
        for now,sequence in ((100.,1),(100.1,2),(100.3,3)):
            result=policy.decide(view(.5,.8,1.2,now,sequence),settings,now)
            self.assertEqual(result[:2],(.04,-.4));self.assertFalse(result[3])
        # Direction remains locked even when the other side becomes larger.
        self.assertLess(policy.decide(view(.5,2.,1.,100.4,4),settings,100.4)[1],0.)

    def test_both_sides_blocked_wait_and_timeout_stops(self):
        policy=CruisePolicy();settings=CruiseSettings()
        policy.decide(view(.5,.34,.34),settings,100.)
        self.assertEqual(policy.decide(view(.5,.34,.34,101.,2),settings,101.)[:2],(0.,0.))
        self.assertTrue(policy.decide(view(.5,.34,.34,121.,3),settings,121.)[3])

    def test_unknown_side_or_rear_hazard_stops_turning_sweep(self):
        policy=CruisePolicy();settings=CruiseSettings()
        for scan,fatal in ((view(.5,None,1.),True),
                           (view(.5,rear=.29),False),(view(.5,rear=None),False)):
            result=policy.decide(scan,settings,100.)
            self.assertEqual(result[:2],(0.,0.));self.assertEqual(result[3],fatal)

    def test_resume_requires_three_distinct_clear_scans(self):
        policy=CruisePolicy();settings=CruiseSettings()
        policy.decide(view(.5),settings,100.)
        policy.decide(view(.5,now=100.3,sequence=2),settings,100.3)
        for now in (100.4,100.5,100.6):
            policy.decide(view(1.,now=now,sequence=3),settings,now)
        self.assertEqual(policy.phase,'avoid')
        policy.decide(view(1.,now=100.7,sequence=4),settings,100.7)
        result=policy.decide(view(1.,now=100.8,sequence=5),settings,100.8)
        self.assertGreater(result[0],0.);self.assertEqual(result[1],0.);self.assertEqual(policy.phase,'forward')
        self.assertGreater(policy.decide(view(1.,now=101.1,sequence=6),settings,101.1)[0],0.)

    def test_stale_unknown_or_invalid_data_never_move(self):
        for scan in (view(None),view(now=98.),dict(view(),error='bad')):
            result=CruisePolicy().decide(scan,CruiseSettings(),100.)
            self.assertEqual(result[:2],(0.,0.));self.assertTrue(result[3])

    def test_settings_reject_nonfinite_outside_range(self):
        for distance in (.1,.2,2.):
            self.assertEqual(CruiseSettings(distance,.06).front_back_distance,distance)
            self.assertEqual(CruiseSettings(side_distance=distance).side_distance,distance)
        for distance,speed in ((math.nan,.06),(.099,.06),(2.001,.06),(.6,.3),(.6,math.inf),(.6,0.)):
            with self.assertRaises(ValueError):CruiseSettings(distance,speed)
        for distance in (math.nan,math.inf,.099,2.001):
            with self.assertRaises(ValueError):CruiseSettings(side_distance=distance)

    def test_below_point_three_uses_configured_distances_without_fixed_stop(self):
        settings=CruiseSettings(.1,.06,side_distance=.1)
        result=CruisePolicy().decide(view(.29,.29,.29,rear=.29),settings,100.)
        self.assertGreater(result[0],0.);self.assertEqual(result[1],0.);self.assertFalse(result[3])
        # The front and rear both lie below the old floor; turn using the configured thresholds.
        result=CruisePolicy().decide(view(.09,.29,.29,rear=.2),settings,100.)
        self.assertGreater(result[0],0.);self.assertNotEqual(result[1],0.);self.assertFalse(result[3])
        result=CruisePolicy().decide(view(.29,.09,.29,rear=.2),settings,100.)
        self.assertGreater(result[0],0.);self.assertLess(result[1],0.);self.assertFalse(result[3])

    def test_blocked_turn_still_waits_at_configured_low_thresholds(self):
        settings=CruiseSettings(.1,.06,side_distance=.1)
        for scan in (view(.09,.09,.09,rear=.2),view(.09,.29,.29,rear=.09)):
            result=CruisePolicy().decide(scan,settings,100.)
            self.assertEqual(result[:2],(0.,0.));self.assertFalse(result[3])

    def test_front_back_and_side_thresholds_trigger_independently(self):
        scan=view(.7,1.2,1.2)
        self.assertEqual(CruisePolicy().decide(scan,CruiseSettings(.6),100.)[1],0.)
        self.assertNotEqual(CruisePolicy().decide(scan,CruiseSettings(.8),100.)[1],0.)
        scan=view(2.,.5,1.5)
        self.assertEqual(CruisePolicy().decide(scan,CruiseSettings(side_distance=.4),100.)[1],0.)
        vx,wz,_,fatal=CruisePolicy().decide(scan,CruiseSettings(side_distance=.6),100.)
        self.assertGreater(vx,0.);self.assertLess(wz,0.);self.assertFalse(fatal)
        self.assertGreater(CruisePolicy().decide(view(2.,1.5,.5),CruiseSettings(side_distance=.6),100.)[1],0.)

    def test_side_threshold_controls_available_turn_direction(self):
        scan=view(.5,.55,.45)
        self.assertGreater(CruisePolicy().decide(scan,CruiseSettings(side_distance=.4),100.)[1],0.)
        result=CruisePolicy().decide(scan,CruiseSettings(side_distance=.6),100.)
        self.assertEqual(result[:2],(0.,0.));self.assertFalse(result[3])

    def test_rear_uses_front_back_threshold_for_turning_but_allows_driving_away(self):
        scan=view(.5,rear=.7)
        self.assertGreater(CruisePolicy().decide(scan,CruiseSettings(.6),100.)[0],0.)
        result=CruisePolicy().decide(scan,CruiseSettings(.8),100.)
        self.assertEqual(result[:2],(0.,0.));self.assertIn('后方',result[2])
        result=CruisePolicy().decide(view(2.,rear=.2),CruiseSettings(.8),100.)
        self.assertGreater(result[0],0.);self.assertEqual(result[1],0.);self.assertFalse(result[3])

    def test_rear_obstacle_ends_turn_without_stopping_in_an_open_forward_path(self):
        policy=CruisePolicy();settings=CruiseSettings()
        self.assertNotEqual(policy.decide(view(.5),settings,100.)[1],0.)
        for sequence in range(2,5):
            now=100.+sequence*.1
            result=policy.decide(view(2.,rear=.5,now=now,sequence=sequence),settings,now)
            self.assertGreater(result[0],0.);self.assertEqual(result[1],0.);self.assertFalse(result[3])
        self.assertEqual(policy.phase,'forward')

    def test_front_clear_alone_does_not_end_side_avoidance(self):
        policy=CruisePolicy();settings=CruiseSettings(side_distance=.5)
        for sequence in range(1,5):
            now=100.+sequence*.1
            result=policy.decide(view(2.,.45,1.,now,sequence),settings,now)
            self.assertGreater(result[0],0.);self.assertLess(result[1],0.)
        self.assertEqual(policy.phase,'avoid')
        for sequence in range(5,8):
            now=100.+sequence*.1
            result=policy.decide(view(2.,.6,1.,now,sequence),settings,now)
        self.assertEqual(policy.phase,'forward');self.assertGreater(result[0],0.);self.assertEqual(result[1],0.)

    def test_passed_side_obstacle_does_not_retrigger_turning(self):
        scan=dict(view(2.,left=.34,rear=.5),left_ahead=2.,right_ahead=2.)
        result=CruisePolicy().decide(scan,CruiseSettings(),100.)
        self.assertGreater(result[0],0.);self.assertEqual(result[1],0.);self.assertFalse(result[3])

    def test_arc_speed_never_exceeds_requested_speed(self):
        for speed in (.02,.03,.06,.15):
            vx,wz,_,fatal=CruisePolicy().decide(view(.4),CruiseSettings(.6,speed),100.)
            self.assertGreater(vx,0.);self.assertLessEqual(vx,speed)
            self.assertEqual(abs(wz),.4);self.assertFalse(fatal)

    def test_simulated_car_clears_box_then_continues_forward_without_collision(self):
        # Ray cast against a 20 cm box centered at (1,0), and a 4x4 m room.
        box=(.9,1.1,-.1,.1)
        def hit_rectangle(x,y,dx,dy,bounds):
            lo,hi=-math.inf,math.inf
            for position,direction,minimum,maximum in ((x,dx,bounds[0],bounds[1]),(y,dy,bounds[2],bounds[3])):
                if abs(direction)<1e-10:
                    if not minimum<=position<=maximum:return math.inf
                    continue
                a,b=sorted(((minimum-position)/direction,(maximum-position)/direction))
                lo,hi=max(lo,a),min(hi,b)
            if hi<max(0.,lo):return math.inf
            return lo if lo>0 else hi
        x=y=yaw=0.;monitor=LaserMonitor();policy=CruisePolicy();avoided=False
        for i in range(600):
            ranges=[]
            for j in range(400):
                angle=yaw+j*math.tau/400;dx,dy=math.cos(angle),math.sin(angle)
                ranges.append(min(hit_rectangle(x,y,dx,dy,box),hit_rectangle(x,y,dx,dy,(-2.,3.,-2.,2.)),10.))
            now=100.+i*.1
            monitor.update(scan_message(ranges,i+1),now)
            vx,wz,text,fatal=policy.decide(monitor.snapshot(),CruiseSettings(),now)
            self.assertFalse(fatal,(i,x,y,text))
            self.assertGreater(vx,0.,(i,text))  # no routine stop or sideways motion
            mid_yaw=yaw+wz*.05
            x+=vx*math.cos(mid_yaw)*.1;y+=vx*math.sin(mid_yaw)*.1;yaw+=wz*.1;avoided|=bool(wz)
            gap=math.hypot(max(.9-x,0.,x-1.1),max(-.1-y,0.,y-.1))
            self.assertGreater(gap,.20)  # 20 cm collision envelope must stay outside the box.
            if avoided and policy.phase=='forward' and math.hypot(x-1.,y)>1.1:break
        self.assertTrue(avoided);self.assertEqual(policy.phase,'forward')
        self.assertGreater(math.hypot(x-1.,y),1.1);self.assertGreater(abs(y),.25)


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.sent=[];self.now=100.
        self.app=types.SimpleNamespace(cfg=copy.deepcopy(DEFAULTS),state=CarState(),log=lambda text:None,
            bridge=types.SimpleNamespace(connected=True,publish=lambda topic,msg:self.sent.append(msg) or True))
        self.app.state.update(ws_connected=True,cam_connected=True)
        self.monitor=LaserMonitor();self.monitor.update(scan_message(),100.)
        self.runner=CruiseRunner(AppContext(self.app),self.monitor,clock=lambda:self.now)

    def tearDown(self):self.runner.stop()

    def test_independent_watchdog_stops_when_scan_stream_stalls(self):
        self.runner.start(CruiseSettings())
        time.sleep(.03);self.assertTrue(any(msg['linear']['x']>0 for msg in self.sent))
        self.now=101.;time.sleep(.15)
        self.assertFalse(self.runner.running)
        self.assertEqual(self.sent[-1]['linear'],dict(x=0.,y=0.,z=0.))
        count=len(self.sent);self.now=102.;self.monitor.update(scan_message(sequence=2),102.)
        time.sleep(.12);self.assertEqual(len(self.sent),count)  # recovery must not restart motion.

    def test_disconnect_camera_loss_publish_failure_cancel(self):
        for failure in ('ros','camera','publish'):
            self.runner.running=True;self.runner.cancel.clear()
            self.app.state.update(ws_connected=failure!='ros',cam_connected=failure!='camera')
            self.app.bridge.publish=lambda topic,msg:failure!='publish'
            self.runner.step();self.assertFalse(self.runner.running)

    def test_arc_publishes_forward_and_yaw_with_zero_lateral_velocity(self):
        msg=scan_message();msg['ranges'][0]=.5
        self.monitor.update(dict(msg,header=dict(frame_id='lidar_frame',stamp=dict(sec=101,nanosec=1))),100.)
        self.runner.running=True;self.runner.step()
        command=self.sent[-1]
        self.assertGreater(command['linear']['x'],0.)
        self.assertEqual(command['linear']['y'],0.)
        self.assertEqual(abs(command['angular']['z']),.4)

    def test_manual_stop_no_later_movement_and_reconfigure(self):
        self.runner.start(CruiseSettings());self.runner.configure(CruiseSettings(.8,.08))
        self.runner.stop();count=len(self.sent);time.sleep(.12)
        self.assertEqual(len(self.sent),count)
        self.assertEqual(self.app.state.get('base_cmd'),(0.,0.))

    def test_main_preserves_transport_receipt_time(self):
        app=App.__new__(App)
        app.features=[types.SimpleNamespace(on_ros_message=lambda topic,msg,when:self.monitor.update(msg,when))]
        App._on_ros_message(app,'/scan_raw',scan_message(sequence=2),88.)
        self.assertEqual(self.monitor.snapshot()['received_at'],88.)


class CruiseConfigTests(unittest.TestCase):
    def test_legacy_single_threshold_migrates_before_default_merge(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'settings.json'
            saved=dict(cruise=dict(detection_distance=.42,speed=.08),tracking_pick=dict(calibrations={'keep':{'x':.23}}))
            path.write_text(json.dumps(saved),encoding='utf-8')
            cfg=load_config(str(path))
            self.assertEqual(cfg['cruise'],dict(front_back_distance=.42,side_distance=.35,speed=.08))
            self.assertEqual(cfg['tracking_pick']['calibrations'],saved['tracking_pick']['calibrations'])
            # Reading a legacy file must not overwrite it.
            self.assertEqual(json.loads(path.read_text(encoding='utf-8')),saved)

    def test_explicit_new_thresholds_win_and_round_trip_independently(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'settings.json'
            saved=dict(cruise=dict(detection_distance=.2,front_back_distance=.9,side_distance=.45,speed=.10))
            path.write_text(json.dumps(saved),encoding='utf-8')
            cfg=load_config(str(path))
            self.assertEqual(cfg['cruise'],dict(front_back_distance=.9,side_distance=.45,speed=.10))
            save_config(cfg,str(path));reloaded=load_config(str(path))
            self.assertEqual(reloaded['cruise'],cfg['cruise'])
            self.assertNotIn('detection_distance',json.loads(path.read_text(encoding='utf-8'))['cruise'])


class CruiseWidgetTests(unittest.TestCase):
    def test_ui_settings_start_stop_recording_and_vehicle_reset(self):
        root=tk.Tk();root.withdraw();state=CarState();state.update(ws_connected=True,cam_connected=True)
        camera=types.SimpleNamespace(recording=False)
        def start_record(path):camera.recording=True;return path
        def stop_record():camera.recording=False
        camera.start_record,camera.stop_record=start_record,stop_record
        app=types.SimpleNamespace(cfg=copy.deepcopy(DEFAULTS),state=state,root=root,camera=camera,features=[],
            frame_listeners=[],log=lambda text:None,bridge=types.SimpleNamespace(connected=True,publish=lambda *a:True))
        feature=CruiseFeature(AppContext(app));feature.build(root);app.features=[feature]
        try:
            with tempfile.TemporaryDirectory() as folder,patch('carapp.features.cruise.save_config'):
                feature.ctx.save_dir=lambda sub='':folder
                feature.on_activate();feature.on_ros_message('/scan_raw',scan_message(),time.monotonic())
                feature.front_back_distance.set('.1');feature.side_distance.set('.2')
                self.assertTrue(feature.apply_settings())
                self.assertEqual(app.cfg['cruise']['front_back_distance'],.1)
                self.assertEqual(app.cfg['cruise']['side_distance'],.2)
                self.assertNotIn('保护优先',str(feature.applied.cget('text')))
                feature.front_back_distance.set('.8');feature.side_distance.set('.45');feature.speed.set('.09')
                expected=CruiseSettings(.8,.09,side_distance=.45)
                self.assertTrue(feature.apply_settings());self.assertEqual(feature.runner.settings,expected)
                self.assertEqual(app.cfg['cruise'],expected.__dict__)
                feature.front_back_distance.set('nan');self.assertFalse(feature.apply_settings())
                self.assertEqual(feature.runner.settings,expected)
                feature.front_back_distance.set('.8');feature.side_distance.set('.099')
                self.assertFalse(feature.apply_settings());self.assertEqual(feature.runner.settings,expected)
                feature.side_distance.set('.45')
                with patch('carapp.features.cruise.save_config',side_effect=OSError('disk unavailable')):
                    feature.side_distance.set('.6');self.assertFalse(feature.apply_settings())
                self.assertEqual(feature.runner.settings,expected);self.assertEqual(app.cfg['cruise'],expected.__dict__)
                feature.side_distance.set('.45');feature.cam._last_frame=(0,b'frame-bytes','jpeg')
                with patch('builtins.open',side_effect=AssertionError('motor notification must not write disk')):
                    feature.start()
                self.assertTrue(feature._busy)
                self.assertTrue(camera.recording)
                feature._flush_frames()
                self.assertTrue(list(Path(folder).glob('阶段_*.jpg')))
                feature.on_deactivate();self.assertFalse(feature._busy);self.assertFalse(camera.recording)
                self.assertTrue((Path(folder)/'运行记录.json').exists())
                feature.on_vehicle_change();self.assertIsNone(feature.monitor.snapshot()['front'])
        finally:
            feature.on_close();root.destroy()


if __name__=='__main__':unittest.main()
