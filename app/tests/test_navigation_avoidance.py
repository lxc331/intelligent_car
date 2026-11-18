"""Corridor clearance and the reroute-first obstacle policy, both offline."""
import math
import unittest

from carapp.core.navigation_avoidance import AvoidancePolicy, AvoidanceSettings
from carapp.core.navigation_path import corridor_obstacle, project_polyline
from carapp.core.sandbox_runtime import ForwardGuard


def straight(length=1.0, step=.05):
    count = int(round(length/step))
    return [(index*step, 0., 0.) for index in range(count+1)]


class CorridorGeometryTests(unittest.TestCase):
    def test_project_polyline_returns_arc_length_and_lateral_gap(self):
        path = straight(1.)
        s, gap = project_polyline(path, .30, .02)
        self.assertAlmostEqual(s, .30, places=6)
        self.assertAlmostEqual(gap, .02, places=6)
        s, gap = project_polyline(path, .50, -.20)
        self.assertAlmostEqual(gap, .20, places=6)

    def test_object_on_the_centre_line_is_measured_along_the_path(self):
        distance, point = corridor_obstacle([(.30, .01)], straight(1.))
        self.assertAlmostEqual(distance, .30, places=6)
        self.assertEqual(point, (.30, .01))

    def test_corridor_walls_and_side_objects_are_not_obstacles(self):
        walls = [(x, .11) for x in (0., .1, .2, .3, .4)] + [(x, -.11) for x in (0., .1, .2, .3)]
        self.assertEqual(corridor_obstacle(walls, straight(1.), half_width=.06), (None, None))

    def test_objects_behind_or_beyond_the_lookahead_are_ignored(self):
        self.assertEqual(corridor_obstacle([(-.40, 0.)], straight(1.)), (None, None))
        self.assertEqual(corridor_obstacle([(.90, 0.)], straight(1.), lookahead=.55), (None, None))

    def test_nearest_object_wins_and_curve_is_followed(self):
        distance, point = corridor_obstacle([(.40, 0.), (.20, .01)], straight(1.))
        self.assertAlmostEqual(distance, .20, places=6)
        self.assertEqual(point, (.20, .01))
        corner = [(0., 0., 0.), (.2, 0., 0.), (.2, .2, 1.57), (.2, .4, 1.57)]
        self.assertAlmostEqual(corridor_obstacle([(.21, .30)], corner)[0], .50, places=6)
        self.assertAlmostEqual(corridor_obstacle([(.35, .30)], corner, half_width=.06)[0] or 0., 0.)


class ForwardGuardTests(unittest.TestCase):
    def test_block_latches_only_after_the_hold_time_and_releases_with_hysteresis(self):
        guard = ForwardGuard(hold=.25, release=.5)
        self.assertFalse(guard.update(.10, 0.))
        self.assertFalse(guard.update(.10, .20))
        self.assertTrue(guard.update(.10, .30))
        self.assertEqual(guard.speed_scale, 0.)
        # Inside the hysteresis band (stop .25 / clear .32) the stop stays latched.
        self.assertTrue(guard.update(.30, .60))
        self.assertTrue(guard.update(.31, 1.10))
        # Clearance must persist for the release time before the latch drops.
        self.assertTrue(guard.update(None, 1.20))
        self.assertTrue(guard.blocked)
        self.assertFalse(guard.update(None, 1.80))
        self.assertFalse(guard.blocked)

    def test_speed_ramps_between_stop_and_slow_distance(self):
        guard = ForwardGuard(stop_distance=.25, slow_distance=.45)
        guard.update(.45, 0.)
        self.assertAlmostEqual(guard.speed_scale, 1.)
        guard.update(.35, .1)
        self.assertAlmostEqual(guard.speed_scale, .5, places=6)
        guard.update(.25, .2)
        self.assertAlmostEqual(guard.speed_scale, 0., places=6)

    def test_invalid_thresholds_are_refused(self):
        for values in (dict(stop_distance=.05), dict(slow_distance=.26),
                       dict(clear_distance=.50), dict(half_width=.5),
                       dict(lookahead=.1), dict(hold=-1.), dict(release=5.),
                       dict(stop_distance=math.nan)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                ForwardGuard(**values)


class AvoidancePolicyTests(unittest.TestCase):
    def blocked(self, policy, now, distance=.20):
        return policy.update(now, phase='paused', avoid=dict(blocked=True, distance=distance))

    def test_reroutes_first_then_waits_without_cancelling_the_goal(self):
        policy = AvoidancePolicy(AvoidanceSettings(reroute_after=1., wait_seconds=10.))
        self.assertEqual(self.blocked(policy, 0.).action, 'hold')
        self.assertEqual(policy.update(.5, phase='paused', avoid=dict(blocked=True)).action, 'hold')
        self.assertEqual(policy.update(1.1, phase='paused', avoid=dict(blocked=True)).action, 'reroute')
        policy.rerouted(False, 1.2)
        self.assertEqual(policy.state, 'waiting')
        actions = [policy.update(float(step), phase='paused', avoid=dict(blocked=True))
                   for step in range(3, 60)]
        self.assertNotIn('stop', [decision.action for decision in actions])
        escalated = [decision.reason for decision in actions if decision.action == 'escalate']
        self.assertEqual(len(escalated), 1)
        self.assertIn('自动继续', escalated[0])
        # Waiting must not become a 2 Hz notice stream.
        self.assertEqual([d.reason for d in actions if d.action == 'hold'], ['']*len([d for d in actions if d.action == 'hold']))
        self.assertEqual(policy.update(60., phase='navigating', avoid=dict(blocked=False)).action, 'recovered')

    def test_explicit_give_up_still_stops_when_asked(self):
        policy = AvoidancePolicy(AvoidanceSettings(reroute_after=1., wait_seconds=2., give_up_seconds=10.))
        self.blocked(policy, 0.)
        self.assertEqual(policy.update(1.5, phase='paused', avoid=dict(blocked=True)).action, 'reroute')
        policy.rerouted(False, 1.5)
        self.assertEqual(policy.update(5., phase='paused', avoid=dict(blocked=True)).action, 'escalate')
        decision = policy.update(12., phase='paused', avoid=dict(blocked=True))
        self.assertEqual(decision.action, 'stop')
        self.assertIn('障碍', decision.reason)
        self.assertEqual(policy.update(99., phase='paused', avoid=dict(blocked=True)).action, 'stop')

    def test_successful_reroute_or_cleared_road_returns_to_idle(self):
        policy = AvoidancePolicy()
        self.blocked(policy, 0.)
        policy.update(2., phase='paused', avoid=dict(blocked=True))
        policy.rerouted(True, 2.1)
        self.assertEqual(policy.state, 'idle')
        self.assertEqual(policy.update(3., phase='navigating', avoid=dict(blocked=False)).action, 'hold')

        self.blocked(policy, 4.)
        decision = policy.update(8., phase='navigating', avoid=dict(blocked=False))
        self.assertEqual(decision.action, 'recovered')
        self.assertEqual(policy.state, 'idle')

    def test_car_reported_blockage_counts_without_a_guard_flag(self):
        policy = AvoidancePolicy(AvoidanceSettings(reroute_after=.5))
        self.assertEqual(policy.update(0., phase='blocked', avoid=None).action, 'hold')
        self.assertEqual(policy.update(1., phase='blocked', avoid=None).action, 'reroute')

    def test_disabled_avoidance_never_intervenes(self):
        policy = AvoidancePolicy(AvoidanceSettings(enabled=False))
        for now in range(5):
            self.assertEqual(self.blocked(policy, float(now)).action, 'hold')

    def test_settings_are_validated_and_built_from_config(self):
        for values in (dict(reroute_after=0.), dict(wait_seconds=1.), dict(give_up_seconds=5.),
                       dict(max_turn=math.radians(120.)), dict(max_turn=math.nan)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                AvoidanceSettings(**values)
        built = AvoidanceSettings.from_config(dict(enabled=False, reroute_after=2., wait_seconds=20.,
                                                   give_up_seconds=30., max_turn=math.radians(60.)))
        self.assertEqual((built.enabled, built.reroute_after, built.wait_seconds), (False, 2., 20.))
        self.assertEqual(built.give_up_seconds, 30.)
        self.assertAlmostEqual(built.max_turn, math.radians(60.))


if __name__ == "__main__":
    unittest.main()
