"""Measured footprint, visible swept area and bounded forward-arc selection."""
import json
import math
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from carapp.core.cruise import CruisePolicy, CruiseSettings
from carapp.core.cruise_geometry import VehicleGeometry, check_turns, evaluate_motion, load_vehicle_geometry
from carapp.core.laser_scan import LaserMonitor
from test_cruise import scan_message


def geometry_scan(*obstacles, unknown=(), sequence=1, now=100.):
    message=scan_message(sequence=sequence)
    for angle,distance in obstacles:
        message['ranges'][round((angle%360)/.9)%400]=distance
    for angle in unknown:
        message['ranges'][round((angle%360)/.9)%400]=None
    monitor=LaserMonitor();monitor.update(message,now)
    return monitor.snapshot()


class FootprintTests(unittest.TestCase):
    def test_user_dimensions_and_lidar_offset_are_consistent(self):
        geometry=load_vehicle_geometry()
        self.assertAlmostEqual(geometry.half_width*2,.17)
        self.assertAlmostEqual(geometry.front-geometry.lidar_x,.22)
        self.assertAlmostEqual(geometry.rear+geometry.lidar_x,.09)
        self.assertAlmostEqual(geometry.front+geometry.rear,.31)

    def test_invalid_dimensions_and_config_fail_without_silent_fallback(self):
        for fields in ({'front':math.nan},{'half_width':0.},{'lidar_x':-.5},
                       {'linear_deceleration':0.},{'prediction_seconds':.2},{'margin':True}):
            with self.subTest(fields=fields),self.assertRaises(ValueError):VehicleGeometry(**fields)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'geometry.json';path.write_text('{"geometry":{"front":0}}',encoding='utf-8')
            with self.assertRaises(ValueError):load_vehicle_geometry(path)
            path.write_text('{bad json',encoding='utf-8')
            with self.assertRaises(ValueError):load_vehicle_geometry(path)

    def test_scan_preserves_angles_unknowns_and_explicit_free_rays(self):
        message=scan_message();message['ranges'][80]=None;message['ranges'][100]=math.inf
        message['ranges'][300]=.05
        monitor=LaserMonitor();monitor.update(message,100.);rays=monitor.snapshot()['rays']
        self.assertIsInstance(rays,tuple)
        self.assertEqual(rays[80][1:],(None,False))
        self.assertEqual(rays[100][1:],(10.,False))
        self.assertEqual(rays[300][1:],(.05,True))
        self.assertAlmostEqual(rays[300][0],-math.pi/2)
        json.dumps(monitor.snapshot(),allow_nan=False)

    def test_measured_corridor_excludes_side_obstacle_but_includes_body_obstacle(self):
        monitor=LaserMonitor();message=scan_message();message['ranges'][22]=.5
        monitor.update(message,100.);self.assertGreater(monitor.snapshot()['front'],1.)
        message=scan_message(sequence=2);message['ranges'][11]=.5
        monitor.update(message,100.);self.assertLess(monitor.snapshot()['front'],.5)


class SweptMotionTests(unittest.TestCase):
    def test_rear_near_point_outside_sweep_does_not_block_either_turn(self):
        scan=geometry_scan((126.,.288),(234.,.293),(0.,.5))
        self.assertLess(scan['left_rear'],.3);self.assertLess(scan['right_rear'],.3)
        for options in check_turns(scan,.04,extra_margin=.01).values():
            self.assertTrue(any(option.clear for option in options))
        result=CruisePolicy().decide(scan,CruiseSettings(.6,.1,.2),100.)
        self.assertGreater(result[0],0.);self.assertNotEqual(result[1],0.)

    def test_true_tail_sweep_blocks_only_the_turn_that_moves_toward_it(self):
        for angle,blocked,clear in ((112.5,'right','left'),(247.5,'left','right')):
            with self.subTest(angle=angle):
                options=check_turns(geometry_scan((angle,.14)),.04,extra_margin=.01)
                self.assertFalse(any(item.clear for item in options[blocked]))
                self.assertTrue(any(item.clear for item in options[clear]))
                self.assertTrue(any(item.collision_time and item.collision_time>0 for item in options[blocked]))

    def test_a_stronger_arc_can_clear_an_obstacle_that_blocks_gentle_steering(self):
        options=check_turns(geometry_scan((-10.,.32)),.04,extra_margin=.01)['left']
        self.assertFalse(options[0].clear);self.assertTrue(options[1].clear)
        self.assertTrue(options[2].clear)

    def test_gentle_arc_avoids_tail_swing_that_blocks_sharper_steering(self):
        options=check_turns(geometry_scan((-130.,.16)),.04,extra_margin=.01)['left']
        self.assertTrue(options[0].clear);self.assertFalse(options[1].clear);self.assertFalse(options[2].clear)

    def test_point_already_inside_claw_footprint_cannot_be_ignored(self):
        options=check_turns(geometry_scan((0.,.19)),.02)
        for items in options.values():
            self.assertTrue(all(not item.clear and item.collision_time==0. for item in items))

    def test_unknown_new_tail_area_blocks_only_affected_direction(self):
        options=check_turns(geometry_scan(unknown=(111.6,112.5,113.4,114.3)),.04,extra_margin=.01)
        self.assertFalse(any(item.clear for item in options['right']))
        self.assertTrue(any(item.clear for item in options['left']))
        self.assertTrue(all('数据不足' in item.reason for item in options['right']))

    def test_unknown_directly_behind_does_not_block_a_forward_escape(self):
        self.assertTrue(evaluate_motion(geometry_scan(unknown=(180.,)),.04,0.).clear)

    def test_unknown_front_area_is_never_treated_as_free(self):
        self.assertFalse(evaluate_motion(geometry_scan(unknown=(-1.8,-.9,0.,.9,1.8)),.04,0.).clear)

    def test_isolated_hole_has_bounded_support_without_altering_raw_data(self):
        scan=geometry_scan((-.9,.8),(.9,1.2),unknown=(0.,))
        self.assertIsNone(scan['rays'][0][1])
        self.assertAlmostEqual(scan['visibility_rays'][0][1],.77)
        self.assertFalse(scan['visibility_rays'][0][2])
        self.assertIn(0,scan['supported_gap_indices'])
        self.assertTrue(evaluate_motion(scan,.04,0.).clear)

    def test_large_hole_is_not_filled_through_other_inferred_beams(self):
        scan=geometry_scan(unknown=(-1.8,-.9,0.,.9,1.8))
        self.assertIsNone(scan['visibility_rays'][0][1])
        self.assertNotIn(0,scan['supported_gap_indices'])

    def test_recovered_visibility_in_an_open_path_does_not_trigger_an_arbitrary_turn(self):
        policy=CruisePolicy();settings=CruiseSettings(.6,.03,.2)
        blocked=geometry_scan(unknown=(-1.8,-.9,0.,.9,1.8))
        self.assertEqual(policy.decide(blocked,settings,100.)[:2],(0.,0.))
        clear=geometry_scan(sequence=2,now=100.1)
        result=policy.decide(clear,settings,100.1)
        self.assertGreater(result[0],0.);self.assertEqual(result[1],0.)
        self.assertIsNone(policy.direction)

    def test_near_neighbor_cannot_be_averaged_away_by_a_distant_neighbor(self):
        scan=geometry_scan((-.9,.15),(.9,4.),unknown=(0.,))
        self.assertAlmostEqual(scan['visibility_rays'][0][1],.12)
        self.assertFalse(evaluate_motion(scan,.04,0.).clear)

    def test_lack_of_raw_rays_cannot_fall_back_to_sector_minima(self):
        scan=geometry_scan();scan.pop('rays')
        self.assertFalse(evaluate_motion(scan,.04,.15).clear)
        result=CruisePolicy().decide(scan,CruiseSettings(),100.)
        self.assertEqual(result[:2],(0.,0.))

    def test_full_claw_length_matters_even_when_shorter_body_would_pass(self):
        scan=geometry_scan((0.,.28))
        self.assertFalse(evaluate_motion(scan,.04,0.).clear)
        short=replace(VehicleGeometry(),front=.10)
        self.assertTrue(evaluate_motion(scan,.04,0.,short).clear)

    def test_control_delay_and_braking_are_included_after_prediction(self):
        geometry=replace(VehicleGeometry(),prediction_seconds=1.)
        # Without delay/braking, the inflated front would stop before this point.
        distance=.22+geometry.margin+.04*1.1
        result=evaluate_motion(geometry_scan((0.,distance)),.04,0.,geometry)
        self.assertFalse(result.clear);self.assertGreater(result.collision_time,1.)
        self.assertGreater(result.horizon,1.+geometry.control_delay)

    def test_front_threshold_clear_does_not_override_a_claw_collision(self):
        scan=geometry_scan((0.,.19))
        result=CruisePolicy().decide(scan,CruiseSettings(.1,.06,.1),100.)
        self.assertEqual(result[:2],(0.,0.))

    def test_selected_command_is_one_of_the_checked_forward_trajectories(self):
        policy=CruisePolicy();scan=geometry_scan((0.,.5),(-10.,.32))
        result=policy.decide(scan,CruiseSettings(.6,.1,.2),100.)
        self.assertGreater(result[0],0.)
        self.assertTrue(evaluate_motion(scan,result[0],result[1],extra_margin=.01).clear)
        self.assertEqual((policy.turning.motion.speed,policy.turning.motion.yaw),result[:2])

    def test_geometry_diagnostics_are_serializable_and_keep_candidate_reasons(self):
        policy=CruisePolicy();scan=geometry_scan((0.,.5),(112.5,.14))
        policy.decide(scan,CruiseSettings(.6,.06,.2),100.)
        detail=policy.diagnostics
        self.assertEqual(len(detail['options']['right']['candidates']),3)
        self.assertTrue(any(item['collision_point'] for item in detail['options']['right']['candidates']))
        json.dumps(detail,allow_nan=False)


if __name__=='__main__':unittest.main()
