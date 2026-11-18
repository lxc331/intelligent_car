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

    def _blocked_feature(self):
        import json
        from carapp.config import DEFAULTS
        notices, stops = [], []

        class Feature:
            def __init__(self):
                self.goal_node = types.SimpleNamespace(id=3)
                self._stop_requested = False
                self.goal = None
                self.model = types.SimpleNamespace(snapshot=lambda: dict(pose=(0., 0., 0.), pose_stale=False))
                self.ctx = types.SimpleNamespace(cfg=json.loads(json.dumps(DEFAULTS)))
                self.map_panel = types.SimpleNamespace(set_route=Mock(), set_plan=Mock())
            def _notice(self, text, error=False):
                notices.append(text)
            def _ready(self):
                return True
            def _route_goal_pose(self, ids):
                return (0., 0., 0.)
            def stop_all(self):
                stops.append(True)
                self._stop_requested = True

        return Feature(), notices, stops

    def test_blocked_road_reroutes_then_keeps_waiting_without_cancelling(self):
        """Moving the obstacle away must still let the retained goal continue."""
        import time as _time
        from carapp.core.navigation_avoidance import AvoidancePolicy, AvoidanceSettings
        from carapp.features.sandbox_navigation import SandboxNavigation

        feature, notices, stops = self._blocked_feature()
        box = SandboxNavigation(feature)
        box.policy = AvoidancePolicy(AvoidanceSettings(reroute_after=1., wait_seconds=10.))
        box.project = types.SimpleNamespace(network=None, metadata={'id': 'x'}, path=Path('offline'))
        box.route = types.SimpleNamespace(edges=((0, 1),), nodes=(0, 1),
                                          points=((0., 0., 0.), (.1, 0., 0.)), length=.1)
        box._running = True
        clock = [0.]
        with patch('carapp.features.sandbox_navigation.time.monotonic', side_effect=lambda: clock[0]), \
                patch('carapp.features.sandbox_navigation.plan_route', side_effect=ValueError('没有可通行的道路路径')):
            blocked = dict(goal_state='paused', avoid=dict(blocked=True, distance=.2))
            box.on_status(blocked)
            self.assertEqual(box.policy.state, 'blocked')
            clock[0] = 1.1
            box.on_status(blocked)
            self.assertEqual(box.policy.state, 'rerouting')
            for _ in range(200):
                box.on_background_tick()
                if box.policy.state == 'waiting':
                    break
                _time.sleep(.01)
            self.assertEqual(box.policy.state, 'waiting')
            for step in range(2, 40):
                clock[0] = float(step)
                box.on_status(blocked)
            self.assertEqual(box.policy.state, 'waiting')
            self.assertEqual(stops, [])
            self.assertEqual(sum(1 for text in notices if '自动继续' in text), 1)
            self.assertLessEqual(len([text for text in notices if '等待' in text]), 2)
            # 障碍移开：车端回到 navigating，策略只提示一次并复位
            clock[0] = 41.
            box.on_status(dict(goal_state='navigating', avoid=dict(blocked=False)))
        self.assertEqual(box.policy.state, 'idle')
        self.assertTrue(any('障碍已让开' in text for text in notices))

    def test_explicit_give_up_still_stops_and_never_restarts_by_itself(self):
        import time as _time
        from carapp.core.navigation_avoidance import AvoidancePolicy, AvoidanceSettings
        from carapp.features.sandbox_navigation import SandboxNavigation

        feature, notices, stops = self._blocked_feature()
        box = SandboxNavigation(feature)
        box.policy = AvoidancePolicy(AvoidanceSettings(reroute_after=1., wait_seconds=2., give_up_seconds=10.))
        box.project = types.SimpleNamespace(network=None, metadata={'id': 'x'}, path=Path('offline'))
        box.route = types.SimpleNamespace(edges=((0, 1),), nodes=(0, 1),
                                          points=((0., 0., 0.), (.1, 0., 0.)), length=.1)
        box._running = True
        clock = [0.]
        with patch('carapp.features.sandbox_navigation.time.monotonic', side_effect=lambda: clock[0]), \
                patch('carapp.features.sandbox_navigation.plan_route', side_effect=ValueError('没有可通行的道路路径')):
            blocked = dict(goal_state='paused', avoid=dict(blocked=True, distance=.2))
            box.on_status(blocked)
            clock[0] = 1.1
            box.on_status(blocked)
            for _ in range(200):
                box.on_background_tick()
                if box.policy.state == 'waiting':
                    break
                _time.sleep(.01)
            clock[0] = 12.
            box.on_status(blocked)
            self.assertTrue(stops)
            # 停车之后车端可能还在报 paused；不能因此重新开始一轮并被再次取消
            clock[0] = 13.
            box.on_status(blocked)
            box.on_status(blocked)
        self.assertEqual(stops, [True])

    def test_clear_corridor_restarts_the_route_after_a_blocked_abort(self):
        """The car can drop its goal on an abort; once clear it needs a new one."""
        import time as _time
        from carapp.features.sandbox_navigation import SandboxNavigation

        feature, notices, stops = self._blocked_feature()
        feature.session = types.SimpleNamespace(submit=Mock(return_value=True))
        box = SandboxNavigation(feature)
        box.project = types.SimpleNamespace(network=None, metadata={'id': 'x'}, path=Path('offline'))
        box.route = types.SimpleNamespace(edges=((0, 1),), nodes=(0, 1),
                                          points=((0., 0., 0.), (.1, 0., 0.)), length=.1)
        route = types.SimpleNamespace(points=((0., 0., 0.), (.1, 0., 0.)), nodes=(0, 1), length=.1)
        clock = [0.]
        with patch('carapp.features.sandbox_navigation.time.monotonic', side_effect=lambda: clock[0]), \
                patch('carapp.features.sandbox_navigation.plan_route', return_value=route):
            box.on_status(dict(goal_state='blocked', avoid=dict(blocked=False)))
            self.assertTrue(any('重新下发' in text for text in notices))
            for _ in range(300):
                box.on_background_tick()
                if feature.session.submit.called:
                    break
                _time.sleep(.01)
        self.assertTrue(feature.session.submit.called)
        self.assertEqual(feature.session.submit.call_args[0][0], 'follow_path')
        self.assertEqual(stops, [])

    def test_resend_gives_up_after_three_failed_attempts(self):
        import time as _time
        from carapp.features.sandbox_navigation import SandboxNavigation

        feature, notices, stops = self._blocked_feature()
        box = SandboxNavigation(feature)
        box.project = types.SimpleNamespace(network=None, metadata={'id': 'x'}, path=Path('offline'))
        box.route = types.SimpleNamespace(edges=((0, 1),), nodes=(0, 1),
                                          points=((0., 0., 0.), (.1, 0., 0.)), length=.1)
        clock = [0.]
        with patch('carapp.features.sandbox_navigation.time.monotonic', side_effect=lambda: clock[0]), \
                patch('carapp.features.sandbox_navigation.plan_route', side_effect=ValueError('没有可通行路径')):
            for step in range(5):
                clock[0] = step*3.
                box.on_status(dict(goal_state='blocked', avoid=dict(blocked=False)))
                for _ in range(100):
                    box.on_background_tick()
                    if not box.busy:
                        break
                    _time.sleep(.01)
        self.assertEqual(box._resend_count, 3)
        self.assertTrue(any('连续 3 次' in text for text in notices))

    def test_route_that_would_need_a_uturn_is_never_submitted(self):
        from carapp.config import DEFAULTS
        from carapp.features.sandbox_navigation import SandboxNavigation

        notices, calls = [], []

        class Session:
            def submit(self, *args, **kwargs):
                calls.append(args)
                return True

        class Feature:
            def __init__(self):
                self.goal_node = types.SimpleNamespace(id=3)
                self._stop_requested = False
                self.goal = None
                self.session = Session()
                self.model = types.SimpleNamespace(snapshot=lambda: dict(pose=(0., 0., 0.), pose_stale=False))
                self.ctx = types.SimpleNamespace(cfg=json.loads(json.dumps(DEFAULTS)))
                self.map_panel = types.SimpleNamespace(set_route=Mock(), set_plan=Mock())
            def _notice(self, text, error=False):
                notices.append(text)
            def _ready(self):
                return True
            def _route_goal_pose(self, ids):
                return (0., 0., 0.)

        box = SandboxNavigation(Feature())
        box.project = types.SimpleNamespace(network=None, metadata={'id': 'x'}, path=Path('offline'))
        route = types.SimpleNamespace(points=((0., 0., math.pi), (.1, 0., math.pi)),
                                      nodes=(0, 1), length=.1)
        self.assertTrue(box._requires_turn(route, (0., 0., 0.)))
        self.assertFalse(box._requires_turn(types.SimpleNamespace(points=((0., 0., .3),)),
                                           (0., 0., 0.)))
        box.events.put((box.epoch, 'route', (route, (0., 0., 0.)), None))
        box._launch = True
        box.on_background_tick()
        self.assertEqual(calls, [])
        self.assertTrue(any('掉头' in text for text in notices))

    def test_plan_notices_stay_on_the_page_thread(self):
        """The road planner runs in a worker thread; only the page thread may
        write the status bar and the shared 操作记录.json."""
        import threading
        from carapp.config import DEFAULTS
        from carapp.features.sandbox_navigation import SandboxNavigation

        notices = []

        class Feature:
            def __init__(self):
                self.goal_node = types.SimpleNamespace(id=7)
                self._stop_requested = False
                self.model = types.SimpleNamespace(snapshot=lambda: dict(pose=(0., 0., 0.), pose_stale=False))
                self.ctx = types.SimpleNamespace(cfg=json.loads(json.dumps(DEFAULTS)))
                self.map_panel = types.SimpleNamespace(set_route=Mock(), set_plan=Mock())
            def _notice(self, text, error=False):
                notices.append((threading.current_thread().name, text))
            def _ready(self):
                return True
            def _route_goal_pose(self, ids):
                return (0., 0., 0.)

        feature = Feature()
        box = SandboxNavigation(feature)
        box.project = types.SimpleNamespace(network=None, metadata={}, path=Path("offline"))
        route = types.SimpleNamespace(points=(), nodes=(7,), length=1.)
        with patch("carapp.features.sandbox_navigation.plan_route", return_value=route):
            self.assertTrue(box.plan(preview_start=(0., 0., 0.)))
            for _ in range(100):
                box.on_background_tick()
                if box.route is route:
                    break
                threading.Event().wait(.02)
        self.assertIs(box.route, route)
        self.assertEqual(len(notices), 3)
        self.assertEqual({name for name, _ in notices}, {"MainThread"})

    def test_project_checked_out_with_crlf_still_loads(self):
        """git on Windows rewrites YAML assets to CRLF; the manifest must survive it."""
        raw=Path(__file__).parents[1]/'savedMap/manual_map/rawMap.yaml'
        points=[(.5,.52),(1.39,2.87),(2.72,4.88)]
        with tempfile.TemporaryDirectory() as directory:
            project=build_project(raw,raw,points,points,Path(directory)/'project')
            for name in ('map.yaml','roads.yaml','keepout.yaml','road_network.yaml'):
                path=project.path.parent/name
                path.write_bytes(path.read_bytes().replace(b'\r\n',b'\n').replace(b'\n',b'\r\n'))
            load_project(project.path)
            target=project.path.parent/'map.yaml'
            target.write_bytes(target.read_bytes()+b'# edited\r\n')
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

    def test_follow_path_carries_the_requested_margin_setting(self):
        """The car validates against its live costmap; the page may allow zero
        margin on purpose, and the uploaded route must say so."""
        import base64

        def route_payload(remote):
            chunks=[values for operation,values in remote.operations
                    if operation=='upload' and values.get('name','').startswith('route_')]
            return json.loads(b''.join(base64.b64decode(v['chunk']) for v in chunks))

        remote=FakeRemote();remote.start(MAP);remote.operations.clear()
        remote.follow_path([(0,0,0),(.1,0,0)],project_id='test',pass_cost=254)
        self.assertEqual(route_payload(remote)['pass_cost'],254)
        remote.operations.clear()
        remote.follow_path([(0,0,0),(.1,0,0)],project_id='test',pass_cost=300)
        self.assertEqual(route_payload(remote)['pass_cost'],254)
        remote.close()
        remote=FakeRemote();remote.start(MAP);remote.operations.clear()
        remote.follow_path([(0,0,0),(.1,0,0)],project_id='test',pass_cost=100)
        self.assertEqual(route_payload(remote)['pass_cost'],253)
        remote.close()

    def test_road_profile_keeps_separate_localization_map(self):
        params=navigation_parameters({}, {}, '/tmp/test/map.yaml', .04)
        road_parameters(params,'/tmp/test',.04)
        self.assertEqual(params['map_server']['ros__parameters']['yaml_filename'],'/tmp/test/map.yaml')
        self.assertEqual(params['sandbox_mask_server']['ros__parameters']['yaml_filename'],'/tmp/test/keepout.yaml')
        self.assertIn('RegulatedPurePursuit',params['controller_server']['ros__parameters']['FollowPath']['plugin'])
        self.assertIn('sandbox_keepout',params['global_costmap']['global_costmap']['ros__parameters']['filters'])
        # The mask marks everything off the narrow sandbox road lethal; applying it
        # to the local costmap aborts the controller on its own footprint, so it
        # belongs to planning only.
        self.assertNotIn('filters',params['local_costmap']['local_costmap']['ros__parameters'])

    def test_local_costmap_keeps_room_for_tracking_error(self):
        params=navigation_parameters({}, {}, '/tmp/test/map.yaml', .04)
        road_parameters(params,'/tmp/test',.04)
        local=params['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']
        global_=params['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']
        self.assertLessEqual(local['inflation_radius'],.15)
        self.assertGreaterEqual(global_['inflation_radius'],.20)

    def test_route_paths_round_corners_so_the_controller_does_not_stall(self):
        """A raw waypoint polyline jumps in heading, which makes the pure-pursuit
        controller crawl or wobble at every node; the submitted path must turn
        smoothly and stay inside the corridor."""
        from carapp.core.navigation_path import checked_path, rounded_path
        vertices=[(0.,0.,0.),(1.,0.,0.),(1.,1.,0.),(2.,1.,0.)]
        path=checked_path(rounded_path(vertices,radius=.2))
        self.assertEqual(path[0][:2],(0.,0.))
        self.assertEqual(path[-1][:2],(2.,1.))
        self.assertTrue(all(math.dist(a[:2],b[:2])<=.15 for a,b in zip(path,path[1:])))
        # The right-angle corner is cut: no sample sits on the original vertex.
        self.assertGreater(min(math.dist(point[:2],(1.,0.)) for point in path),.04)
        # Heading changes stay small between neighbouring samples.
        turns=[abs(math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2])))
               for a,b in zip(path,path[1:]) if math.dist(a[:2],b[:2])>.004]
        self.assertLess(max(turns),.6)

    def test_controller_keeps_a_long_enough_lookahead_and_fast_corner_turns(self):
        """Short lookahead + a wide rotate-in-place dead band is what made the car
        weave on straights and idle at every corner."""
        params=navigation_parameters({}, {}, '/tmp/test/map.yaml', .04)
        road_parameters(params,'/tmp/test',.04)
        follow=params['controller_server']['ros__parameters']['FollowPath']
        self.assertGreaterEqual(follow['lookahead_dist'],.15)
        self.assertLessEqual(follow['rotate_to_heading_min_angle'],.2)
        self.assertGreaterEqual(follow['rotate_to_heading_angular_vel'],.4)
        # RPP's costmap collision check cannot pass in a 25–27 cm corridor: the
        # footprint half-width is 0.105 m, inscribed inflation starts 0.115 m
        # from a wall and the corridor half-width is only ~0.125 m, so the
        # projection trips "detected collision ahead" and aborts every goal.
        self.assertFalse(follow['use_collision_detection'])
