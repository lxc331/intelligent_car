"""Metric registration, project integrity and road motion boundary tests."""
import json
import math
from pathlib import Path
import tempfile
import unittest
import types
import tkinter as tk
from unittest.mock import Mock, patch
from carapp.core.sandbox_geometry import fit_registration, cropped_origin
from carapp.core.sandbox_project import build_project, load_project
from carapp.core.sandbox_route import plan_route
from carapp.core.navigation_path import pose_clear, checked_path
from carapp.core.sandbox_navconfig import road_parameters
from carapp.core.navigation_config import navigation_parameters
from test_navigation_remote import FakeRemote, MAP


class SandboxTests(unittest.TestCase):
    def test_actual_action_contains_all_path_points(self):
        from carapp.core.navigation_runtime import NavigationRuntime
        runtime=NavigationRuntime.__new__(NavigationRuntime)
        runtime.follower=Mock();runtime.navigator=Mock();runtime._stamped=lambda *p:types.SimpleNamespace(header='map',point=p)
        action=types.ModuleType('nav2_msgs.action')
        action.NavigateToPose=types.SimpleNamespace(Goal=types.SimpleNamespace)
        action.FollowPath=types.SimpleNamespace(Goal=types.SimpleNamespace)
        messages=types.ModuleType('nav_msgs.msg');messages.Path=types.SimpleNamespace
        path=[(0,0,0),(.1,0,0),(.2,0,0)]
        with patch.dict('sys.modules',{'nav2_msgs.action':action,'nav_msgs.msg':messages}):
            runtime._navigate({'epoch':3},path[-1],path)
        goal=runtime.follower.send_goal_async.call_args.args[0]
        self.assertEqual([p.point for p in goal.path.poses],path)
        self.assertEqual(goal.goal_checker_id,'general_goal_checker')
        runtime.navigator.send_goal_async.assert_not_called()
        runtime._feedback(types.SimpleNamespace(feedback=types.SimpleNamespace(distance_to_goal=.2)))
        self.assertEqual(runtime.distance_remaining,.2)

    def test_correspondence_wizard_pair_order(self):
        from carapp.ui.sandbox_registration import SandboxRegistration
        root=tk.Tk();root.withdraw()
        try:
            with patch.object(SandboxRegistration,'load'):
                wizard=SandboxRegistration(root,Mock())
            wizard.paths={0:'road',1:'lidar'}
            wizard.add(1,(1,1));self.assertEqual(wizard.points,[[],[]])
            wizard.add(0,(.5,.5));wizard.add(1,(1,1))
            self.assertEqual([len(p) for p in wizard.points],[1,1])
            wizard.undo();self.assertEqual(wizard.points,[[],[]])
            wizard.close()
        finally:root.destroy()

    def test_metric_transform_and_invalid_correspondence(self):
        source=[(0,0),(2,0),(0,3),(2,3)]
        target=[(4-y,5+x) for x,y in source]
        reg=fit_registration(source,target)
        self.assertAlmostEqual(reg.yaw,math.pi/2)
        self.assertAlmostEqual(reg.rms,0)
        for a,b in zip(source,target):
            self.assertLess(math.dist(reg.forward(*a),b),1e-8)
            self.assertLess(math.dist(reg.inverse(*b),a),1e-8)
        for target in ([(0,0),(1,0),(2,0)],[(x*2,y*2) for x,y in source]):
            with self.assertRaises(ValueError):fit_registration(source,target)
        self.assertEqual(cropped_origin((1,2,0),.05,100,10,20,30,40),(1.5,4.,0))

    def test_claw_unknown_and_path_continuity(self):
        grid=dict(width=100,height=100,resolution=.01,origin=(0,0,0),data=[0]*10000)
        self.assertTrue(pose_clear(grid,(.5,.5,0)))
        grid['data'][50*100+72]=100
        self.assertFalse(pose_clear(grid,(.5,.5,0)))
        grid['data'][50*100+72]=-1
        self.assertFalse(pose_clear(grid,(.5,.5,0)))
        with self.assertRaises(ValueError):checked_path([(0,0,0),(1,0,0)])

    def test_project_roundtrip_narrow_road_and_tamper(self):
        raw=Path(__file__).parents[1]/'savedMap/manual_map/rawMap.yaml'
        points=[(.5,.52),(1.39,2.87),(2.72,4.88)]
        with tempfile.TemporaryDirectory() as directory:
            project=build_project(raw,raw,points,points,Path(directory)/'project')
            self.assertEqual(len(tuple(project.network)),9)
            self.assertIn('image: keepout.pgm',project.upload()['keepout_yaml'])
            route=plan_route(project,(.25,1.,math.pi/2),6)
            self.assertFalse(set(route.nodes)&{2,5,8})
            self.assertAlmostEqual(route.points[0][1],1.)
            with self.assertRaises(ValueError):plan_route(project,(.25,1.,math.pi/2),8)
            meta=json.loads(project.path.read_text(encoding='utf8'));meta['sha256']={}
            project.path.write_text(json.dumps(meta),encoding='utf8')
            with self.assertRaises(ValueError):load_project(project.path)

    def test_follow_path_upload_and_cancel_epoch(self):
        remote=FakeRemote();remote.start(MAP)
        ticket=remote.capture_intent()
        remote.follow_path([(0,0,0),(.1,0,0)],project_id='test',intent_epoch=ticket)
        self.assertEqual(remote.operations[-1][0],'follow_path')
        self.assertNotIn('x',remote.operations[-1][1]['values'])
        remote.request_cancel()
        with self.assertRaises(Exception):remote.follow_path([(0,0,0),(.1,0,0)],project_id='test',intent_epoch=ticket)
        remote.close()

    def test_road_profile_keeps_separate_localization_map(self):
        params=navigation_parameters({}, {}, '/tmp/test/map.yaml', .04)
        road_parameters(params,'/tmp/test',.04)
        self.assertEqual(params['map_server']['ros__parameters']['yaml_filename'],'/tmp/test/map.yaml')
        self.assertEqual(params['sandbox_mask_server']['ros__parameters']['yaml_filename'],'/tmp/test/keepout.yaml')
        self.assertIn('RegulatedPurePursuit',params['controller_server']['ros__parameters']['FollowPath']['plugin'])
        for name in ('local_costmap','global_costmap'):
            self.assertIn('sandbox_keepout',params[name][name]['ros__parameters']['filters'])
