"""Visual approach regressions, including the user's far-target miscalibration."""
import copy
import io
import threading
import time
import unittest

from PIL import Image, ImageDraw
from carapp.config import DEFAULTS
from carapp.core.color_tracker import ColorTracker, detect_color
from carapp.core.visual_pick import VisualPickRunner, VisualServoPolicy, validate_calibration
from carapp.core.pick_approach import approach_config, ApproachProgress


def observation(x=.45, y=.35, width=.12, frame_id=1, now=None):
    return {"seen_at": time.monotonic() if now is None else now, "stable": 3, "frame_id": frame_id,
            "target": {"x": x, "y": y, "width": width, "area": width*.12,
                       "bbox": [x-width/2,y-.06,x+width/2,y+.06]}}


class TrackingTests(unittest.TestCase):
    def setUp(self):
        self.cfg = copy.deepcopy(DEFAULTS['tracking_pick'])
        self.cal = {'x': .45, 'y': .76, 'width': .26, 'host': 'car', 'color': 'red',
                    'pose': {'pitch':1300,'yaw':1500,'arm':1700}, 'version':2,'contact_confirmed':True}
        self.policy = VisualServoPolicy(self.cal,self.cfg)

    def test_rejects_real_failed_far_calibration(self):
        bad = dict(self.cal,y=.2911279069767442,width=.1)
        with self.assertRaisesRegex(ValueError,'远端'):
            validate_calibration(bad,'car','red',self.cal['pose'])

    def test_requires_contact_confirmation_and_same_pose(self):
        validate_calibration(self.cal,'car','red',self.cal['pose'])
        with self.assertRaises(ValueError):
            validate_calibration(dict(self.cal,version=1),'car','red',self.cal['pose'])
        with self.assertRaises(ValueError):
            validate_calibration(self.cal,'car','red',{'pitch':1500})

    def test_far_target_generates_forward_motion_not_close(self):
        phase,vx,wz = self.policy.decide(observation(y=.29),time.monotonic())
        self.assertEqual(phase,'接近目标')
        self.assertGreater(vx,0)
        self.assertLessEqual(vx,.06)
        self.assertEqual(wz,0)

    def test_align_before_approaching(self):
        phase,vx,wz = self.policy.decide(observation(x=.65),time.monotonic())
        self.assertEqual(phase,'左右对准')
        self.assertEqual(vx,0)
        self.assertLess(wz,0)

    def test_no_forward_on_lost_or_stale_frame(self):
        for obs in ({'seen_at':time.monotonic(),'target':None},observation(now=0)):
            _,vx,wz = self.policy.decide(obs,time.monotonic())
            self.assertEqual((vx,wz),(0,0))

    def test_ready_requires_distinct_frames_and_correct_size(self):
        obs=observation(y=.75,width=.25)
        for _ in range(5):
            self.assertNotEqual(self.policy.decide(obs,time.monotonic())[0],'到达夹取位置')
        obs['frame_id']=2
        self.assertNotEqual(self.policy.decide(obs,time.monotonic())[0],'到达夹取位置')
        obs['frame_id']=3
        self.assertEqual(self.policy.decide(obs,time.monotonic())[0],'到达夹取位置')
        with self.assertRaises(RuntimeError):
            self.policy.decide(observation(y=.75,width=.10),time.monotonic())

    def test_cancel_stops_without_later_gripper_commands(self):
        commands,servos=[],[]
        runner=VisualPickRunner(self.cfg,self.cal,DEFAULTS['servo']['actions'],
            lambda: observation(),lambda:{'pose':(0,0,0),'seen_at':time.monotonic()},
            lambda vx,wz:commands.append((vx,wz)) or True,
            lambda p,d:servos.append(p) or True)
        thread=threading.Thread(target=runner.run)
        thread.start();time.sleep(.12);runner.cancel();thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(commands[-1],(0,0))
        self.assertEqual(servos,[])

    def test_missing_odometry_never_moves(self):
        commands=[]
        runner=VisualPickRunner(self.cfg,self.cal,DEFAULTS['servo']['actions'],lambda:observation(),
            lambda:None,lambda vx,wz:commands.append((vx,wz)) or True,lambda p,d:True)
        runner.run()
        self.assertTrue(all(c==(0,0) for c in commands))
        self.assertIn('里程计',runner.result)

    def test_detects_near_reference_in_synthetic_image(self):
        image=Image.new('RGB',(640,480),'#777777')
        ImageDraw.Draw(image).rectangle((240,300,350,440),fill='#ff1010')
        buf=io.BytesIO();image.save(buf,format='JPEG')
        target=detect_color(buf.getvalue(),'red')
        self.assertIsNotNone(target)
        self.assertGreater(target['y'],.6)
        self.assertIsNone(detect_color(buf.getvalue(),'blue'))

    def test_tracking_does_not_jump_to_distant_same_colour_object(self):
        image=Image.new('RGB',(320,240),'#777777')
        ImageDraw.Draw(image).rectangle((20,20,60,60),fill='red')
        buf=io.BytesIO();image.save(buf,format='PNG')
        previous={'x':.8,'y':.8,'area':.02}
        self.assertIsNone(detect_color(buf.getvalue(),'red',previous))

    def test_small_far_target_survives_processing(self):
        image=Image.new('RGB',(640,480),'#777777')
        ImageDraw.Draw(image).rectangle((310,150,319,159),fill='red')
        buf=io.BytesIO();image.save(buf,format='PNG')
        target=detect_color(buf.getvalue(),'red')
        self.assertIsNotNone(target)
        self.assertLess(target['width'],.02)

    def test_extended_limits_do_not_mutate_existing_settings(self):
        before=copy.deepcopy(self.cfg)
        run=approach_config(self.cfg,1.5)
        self.assertEqual(self.cfg,before)
        self.assertEqual(run['max_distance'],1.5)
        self.assertGreaterEqual(run['max_seconds'],105.)
        self.assertEqual(run['max_speed'],self.cfg['max_speed'])
        for value in (float('nan'),0,4):
            with self.assertRaises(ValueError):approach_config(self.cfg,value)

    def test_turn_hysteresis_and_minimum_command(self):
        for offset in (.04,.03):
            phase,vx,wz=self.policy.decide(observation(x=self.cal['x']+offset),time.monotonic())
            self.assertEqual(phase,'左右对准')
            self.assertEqual(vx,0.)
            self.assertGreaterEqual(abs(wz),.09)
        self.assertEqual(self.policy.decide(observation(x=self.cal['x']+.015),time.monotonic())[0],'接近目标')

    def test_no_progress_stops_but_turning_does_not_consume_forward_budget(self):
        monitor=ApproachProgress(timeout=1.)
        target=observation()['target']
        for i in range(20):monitor.update(target,0,i*.1)
        with self.assertRaisesRegex(RuntimeError,'未接近'):
            for i in range(30):monitor.update(target,.04,2+i*.1)
        monitor=ApproachProgress(timeout=1.)
        for i in range(30):monitor.update(dict(target,y=.3+i*.01),.04,i*.1)

    def test_one_meter_simulated_closed_loop_reaches_pick_then_lifts(self):
        cfg=approach_config(self.cfg,1.5)
        sim={'time':100.,'x':0.,'vx':0.,'frame':0};servos=[]
        class Clock:
            def is_set(self):return False
            def wait(self,seconds):
                sim['x']+=sim['vx']*seconds;sim['time']+=seconds
                return False
        def observe():
            sim['frame']+=1
            progress=min(1.,sim['x'])
            return observation(y=.25+.51*progress,width=.015+.245*progress,
                               frame_id=sim['frame'],now=sim['time'])
        def drive(vx,wz):sim['vx']=vx;return True
        runner=VisualPickRunner(cfg,self.cal,DEFAULTS['servo']['actions'],observe,
            lambda:dict(pose=(sim['x'],0,0),seen_at=sim['time']),drive,
            lambda p,d:servos.append((sim['vx'],p)) or True)
        runner.cancelled=Clock()
        from unittest.mock import patch
        with patch('carapp.core.visual_pick.time.monotonic',lambda:sim['time']):runner.run()
        self.assertIn('动作完成',runner.result)
        self.assertGreater(sim['x'],.9)
        self.assertLess(sim['x'],1.1)
        self.assertEqual([list(p)[0] for _,p in servos],['gripper','arm'])
        self.assertTrue(all(v==0 for v,p in servos))


if __name__=='__main__':
    unittest.main()
