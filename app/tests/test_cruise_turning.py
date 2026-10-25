"""Directional geometry and stop-confirm-switch behavior, with no vehicle connection."""
import math
import unittest

from carapp.core.cruise import CruisePolicy, CruiseSettings
from carapp.core.cruise_turning import assess_turns
from carapp.core.laser_scan import LaserMonitor
from test_cruise import scan_message, view


class DirectionalScanTests(unittest.TestCase):
    def test_initial_turn_uses_front_clearance_instead_of_side_width(self):
        settings=CruiseSettings(.6,.06,side_distance=.15)
        for obstacle_index,left_side,right_side,expected in ((20,1.2,.5,'right'),(380,.5,1.2,'left')):
            with self.subTest(expected=expected):
                msg=scan_message();msg['ranges'][0]=.5
                msg['ranges'][obstacle_index]=.42
                msg['ranges'][100]=left_side;msg['ranges'][300]=right_side
                monitor=LaserMonitor();monitor.update(msg,100.)
                policy=CruisePolicy();result=policy.decide(monitor.snapshot(),settings,100.)
                self.assertGreater(result[0],0.);self.assertFalse(result[3])
                self.assertEqual(policy.direction,expected)
                if expected=='left':self.assertGreater(result[1],0.)
                else:self.assertLess(result[1],0.)

    def test_scan_stream_drives_stop_confirm_and_opposite_turn(self):
        policy=CruisePolicy();monitor=LaserMonitor();settings=CruiseSettings(.6,.06,side_distance=.15)
        def sample(sequence,blocked):
            message=scan_message(sequence=sequence)
            message['ranges'][0]=.5
            message['ranges'][100]=.5
            message['ranges'][300]=.7
            if blocked:message['ranges'][125]=.13
            return message
        monitor.update(sample(1,False),100.)
        self.assertLess(policy.decide(monitor.snapshot(),settings,100.)[1],0.)
        for sequence,now in ((2,101.),(3,101.1),(4,101.4)):
            monitor.update(sample(sequence,True),now)
            result=policy.decide(monitor.snapshot(),settings,now)
            if sequence<4:self.assertEqual(result[:2],(0.,0.))
        self.assertGreater(result[0],0.);self.assertGreater(result[1],0.)
        self.assertEqual(policy.direction,'left')

    def test_rear_quadrants_and_front_corners_are_independent(self):
        msg=scan_message();msg['ranges'][185]=.28  # 166.5 degrees: left rear
        msg['ranges'][250]=.7                    # 225 degrees: right rear
        msg['ranges'][20]=.12                    # 18 degrees: left front, outside old side sector
        monitor=LaserMonitor();monitor.update(msg,100.)
        scan=monitor.snapshot()
        self.assertEqual(scan['left_rear'],.28);self.assertEqual(scan['right_rear'],.7)
        self.assertEqual(scan['left_front'],.12);self.assertGreater(scan['right_front'],1.)

    def test_direct_rear_boundary_is_checked_in_both_tail_quadrants(self):
        msg=scan_message();msg['ranges'][200]=.2
        monitor=LaserMonitor();monitor.update(msg,100.)
        scan=monitor.snapshot()
        self.assertEqual(scan['left_rear'],.2);self.assertEqual(scan['right_rear'],.2)

    def test_extended_straight_corridor_checks_close_diagonal_obstacles(self):
        msg=scan_message();msg['ranges'][67]=.1  # 60.3 degrees, within the measured 21 cm corridor
        monitor=LaserMonitor();monitor.update(msg,100.)
        scan=monitor.snapshot()
        self.assertGreater(scan['front'],1.)
        self.assertLess(scan['straight_corridor'],.11)
        msg=scan_message(sequence=2);msg['ranges'][100]=.34  # outside corridor at the side
        monitor.update(msg,100.1)
        self.assertGreater(monitor.snapshot()['straight_corridor'],1.)


class TurnAssessmentTests(unittest.TestCase):
    def setUp(self):self.settings=CruiseSettings(.6,.06,side_distance=.15)

    def test_left_turn_checks_right_tail_and_right_turn_checks_left_tail(self):
        options=assess_turns(view(.5,.51,.41,left_rear=.13,right_rear=.7),self.settings,.01)
        self.assertTrue(options['left'].clear)
        self.assertFalse(options['right'].clear);self.assertIn('左后',options['right'].reason)
        options=assess_turns(view(.5,.51,.41,left_rear=.7,right_rear=.13),self.settings,.01)
        self.assertFalse(options['left'].clear);self.assertTrue(options['right'].clear)

    def test_missing_tail_blocks_only_the_direction_that_needs_it(self):
        for missing in (None,math.nan):
            options=assess_turns(view(.5,left_rear=missing,right_rear=.7),self.settings)
            self.assertTrue(options['left'].clear);self.assertFalse(options['right'].clear)

    def test_front_corner_can_block_turn_even_when_side_is_wide(self):
        options=assess_turns(view(.5,1.,1.,left_front=.12),self.settings)
        self.assertFalse(options['left'].clear);self.assertIn('左前',options['left'].reason)
        self.assertFalse(options['right'].clear)  # this point lies inside the present body


class InitialTurnChoiceTests(unittest.TestCase):
    def setUp(self):self.settings=CruiseSettings(.6,.06,side_distance=.15)

    def test_more_open_front_wins_even_when_other_side_is_wider(self):
        for left_front,right_front,left_side,right_side,expected in (
            (.42,.5,1.2,.5,'right'),(.5,.42,.5,1.2,'left')):
            with self.subTest(expected=expected):
                policy=CruisePolicy()
                result=policy.decide(view(.42,left_side,right_side,left_front=left_front,
                                          right_front=right_front),self.settings,100.)
                self.assertGreater(result[0],0.);self.assertFalse(result[3])
                self.assertEqual(policy.direction,expected)

    def test_equal_front_clearance_preserves_wider_side_choice(self):
        for left,right,expected in ((.8,.5,'left'),(.5,.8,'right'),(.5,.5,'right')):
            with self.subTest(expected=expected):
                policy=CruisePolicy()
                policy.decide(view(.5,left,right),self.settings,100.)
                self.assertEqual(policy.direction,expected)

    def test_more_open_front_cannot_override_blocked_tail(self):
        policy=CruisePolicy()
        result=policy.decide(view(.42,.5,.7,left_front=.42,right_front=.5,left_rear=.13),
                             self.settings,100.)
        self.assertGreater(result[0],0.);self.assertGreater(result[1],0.)
        self.assertEqual(policy.direction,'left')

    def test_front_preference_does_not_change_an_available_current_turn(self):
        policy=CruisePolicy()
        policy.decide(view(.5,.5,.7),self.settings,100.)
        result=policy.decide(view(.42,1.2,.7,100.1,2,left_front=.6,right_front=.42),
                             self.settings,100.1)
        self.assertGreater(result[0],0.);self.assertLess(result[1],0.)
        self.assertEqual(policy.direction,'right')


class TurnSwitchTests(unittest.TestCase):
    def setUp(self):
        self.settings=CruiseSettings(.6,.06,side_distance=.15)
        self.policy=CruisePolicy()
        result=self.step(100.,1)
        self.assertLess(result[1],0.)  # right is initially wider

    def step(self,now,sequence,**regions):
        return self.policy.decide(view(.5,.5,.7,now,sequence,**regions),self.settings,now)

    def blocked_right(self,now,sequence,**regions):
        return self.step(now,sequence,**dict(dict(left_rear=.13,right_rear=.8),**regions))

    def test_block_then_three_new_frames_switches_to_left(self):
        for now,sequence in ((101.,2),(101.1,3)):
            self.assertEqual(self.blocked_right(now,sequence)[:2],(0.,0.))
        result=self.blocked_right(101.4,4)
        self.assertGreater(result[0],0.);self.assertGreater(result[1],0.)
        self.assertEqual(self.policy.direction,'left');self.assertEqual(self.policy.avoid_started,100.)

    def test_repeated_frames_never_satisfy_switch_confirmation(self):
        self.blocked_right(101.,2)
        scan=view(.5,.5,.7,101.,2,left_rear=.13,right_rear=.8)
        for now in (101.1,101.2,101.4):
            self.assertEqual(self.policy.decide(scan,self.settings,now)[:2],(0.,0.))
        self.assertEqual(self.policy.turning.confirmations,1)
        self.assertEqual(self.blocked_right(101.45,3)[:2],(0.,0.))
        self.assertGreater(self.blocked_right(101.5,4)[1],0.)

    def test_pause_and_change_interval_prevent_fast_oscillation(self):
        for now,sequence in ((100.1,2),(100.2,3),(100.4,4),(100.9,5)):
            self.assertEqual(self.blocked_right(now,sequence)[:2],(0.,0.))
        self.assertGreater(self.blocked_right(101.1,6)[1],0.)
        for now,sequence in ((101.2,7),(101.3,8),(101.6,9)):
            result=self.step(now,sequence,left_rear=.8,right_rear=.13)
            self.assertEqual(result[:2],(0.,0.))
        self.assertLess(self.step(102.2,10,left_rear=.8,right_rear=.13)[1],0.)

    def test_alternative_becoming_blocked_resets_consecutive_count(self):
        self.blocked_right(101.,2);self.blocked_right(101.1,3)
        self.assertEqual(self.blocked_right(101.2,4,right_rear=.13)[:2],(0.,0.))
        for now,sequence in ((101.3,5),(101.4,6)):
            self.assertEqual(self.blocked_right(now,sequence)[:2],(0.,0.))
        self.assertGreater(self.blocked_right(101.5,7)[1],0.)

    def test_current_direction_recovery_resumes_it_without_unnecessary_switch(self):
        self.blocked_right(101.,2)
        for now,sequence in ((101.1,3),(101.2,4)):
            self.assertEqual(self.step(now,sequence)[:2],(0.,0.))
        self.assertLess(self.step(101.4,5)[1],0.)
        self.assertEqual(self.policy.direction,'right')

    def test_wider_other_side_does_not_interrupt_clear_current_turn(self):
        for now,sequence in ((101.,2),(101.1,3),(101.2,4)):
            result=self.policy.decide(view(.5,1.2,.7,now,sequence),self.settings,now)
            self.assertGreater(result[0],0.);self.assertLess(result[1],0.)

    def test_both_tails_blocked_waits_with_direction_reasons(self):
        result=self.step(101.,2,left_rear=.13,right_rear=.13)
        self.assertEqual(result[:2],(0.,0.));self.assertFalse(result[3])
        self.assertIn('左后',result[2]);self.assertIn('右后',result[2])

    def test_switch_does_not_reset_twenty_second_budget(self):
        self.blocked_right(101.,2);self.blocked_right(101.1,3);self.blocked_right(101.4,4)
        self.assertEqual(self.policy.direction,'left')
        result=self.step(120.,5,left_rear=.8,right_rear=.8)
        self.assertTrue(result[3]);self.assertEqual(result[:2],(0.,0.));self.assertIn('20 秒',result[2])

    def test_stale_scan_cancels_during_confirmation(self):
        self.blocked_right(101.,2)
        result=self.policy.decide(view(.5,.5,.7,101.,2,left_rear=.13,right_rear=.8),self.settings,102.)
        self.assertTrue(result[3]);self.assertEqual(result[:2],(0.,0.))

    def test_blocked_forward_corridor_cannot_be_used_as_a_straight_escape(self):
        result=self.policy.decide(view(.8,.5,.7,101.,2,left_rear=.13,right_rear=.13,
                                       straight_corridor=.1),self.settings,101.)
        self.assertEqual(result[:2],(0.,0.))

    def test_diagnostics_show_both_options_and_confirmation(self):
        self.blocked_right(101.,2)
        detail=self.policy.diagnostics
        self.assertTrue(detail['options']['left']['can_start'])
        self.assertFalse(detail['options']['right']['clear'])
        self.assertEqual(detail['candidate'],'left');self.assertEqual(detail['confirmations'],1)
        self.assertEqual(detail['avoid_elapsed'],1.)


if __name__=='__main__':unittest.main()
