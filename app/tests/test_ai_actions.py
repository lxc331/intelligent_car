"""AI plans are untrusted JSON; execution tests never connect to a vehicle."""
import copy
from dataclasses import FrozenInstanceError
import json
import threading
import time
import types
import unittest

from carapp.config import DEFAULTS
from carapp.core.ai_actions import AIActionRunner, AIPlan, AIStep, build_system_prompt, parse_plan
from carapp.model.state import CarState


def wait_until(predicate, seconds=1.5):
    deadline = time.monotonic() + seconds
    while not predicate() and time.monotonic() < deadline:
        time.sleep(.005)
    if not predicate():
        raise AssertionError("AI worker did not reach expected state")


class AIPlanTests(unittest.TestCase):
    def setUp(self):
        self.cfg = copy.deepcopy(DEFAULTS)

    def plan(self, actions):
        return parse_plan(dict(actions=actions, response="完成"), self.cfg)

    def move(self, **values):
        return dict(type="move", vx=.06, vy=0., wz=0., seconds=1., **values)

    def test_current_schema_is_immutable_and_describes_entire_sequence(self):
        plan = self.plan([dict(type="move", vx=.06, seconds=1),
                          dict(type="servo", targets={"gripper": 2000}, duration=.8),
                          dict(type="wait", seconds=1), dict(type="stop")])
        self.assertEqual([step.type for step in plan.actions], ["move", "servo", "wait", "stop"])
        self.assertAlmostEqual(plan.seconds, 2.8)
        self.assertIn("gripper=2000", plan.summary)
        with self.assertRaises(FrozenInstanceError):
            plan.actions[0].vx = .2
        with self.assertRaises(FrozenInstanceError):
            plan.actions = ()
        self.assertEqual(parse_plan(plan.payload(), self.cfg), plan)

    def test_original_x_y_z_t_arrays_are_supported_without_eval(self):
        plan = parse_plan('{"action":[[0.06,0,0,1],[0,0,-0.25,1],[0,0,0,0]],"response":"出发"}', self.cfg)
        self.assertEqual(plan.actions[1].wz, -.25)
        self.assertEqual(plan.actions[-1].type, "stop")

    def test_only_an_entire_json_document_is_accepted(self):
        valid = json.dumps(dict(actions=[dict(type="stop")]))
        self.assertTrue(parse_plan("```json\n" + valid + "\n```", self.cfg).actions)
        for raw in ("please execute " + valid, "__import__('os').system('whoami')", "[]",
                    '{"actions":[],"actions":[{"type":"stop"}]}',
                    '{"actions":[{"type":"move","seconds":NaN}]}'):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                parse_plan(raw, self.cfg)

    def test_code_topics_and_unrecognised_parameters_are_rejected(self):
        for action in (dict(type="shell", command="whoami"), dict(type="stop", topic="/controller/cmd_vel"),
                       dict(type="move", seconds=1, python="print(1)"),
                       dict(type="servo", targets={"arbitrary_motor": 1500})):
            with self.subTest(action=action), self.assertRaises(ValueError):
                self.plan([action])

    def test_velocity_is_finite_bounded_and_checks_diagonal_norm(self):
        for action in (dict(type="move", vx=.151, seconds=1),
                       dict(type="move", vx=.12, vy=.12, seconds=1),
                       dict(type="move", wz=.401, seconds=1),
                       dict(type="move", vx=float("inf"), seconds=1),
                       dict(type="move", vx=float("nan"), seconds=1),
                       dict(type="move", vx=True, seconds=1),
                       dict(type="move", vx="0.06", seconds=1)):
            with self.subTest(action=action), self.assertRaises(ValueError):
                self.plan([action])

    def test_duration_count_and_total_budget_are_checked_before_motion(self):
        for actions in ([dict(type="move", vx=.06, seconds=0)],
                        [dict(type="move", seconds=-1)], [dict(type="wait", seconds=31)],
                        [dict(type="servo", targets={"gripper": 2000}, duration=11)],
                        [dict(type="wait", seconds=30)] * 5, [dict(type="stop")] * 17):
            with self.subTest(actions=actions), self.assertRaises(ValueError):
                self.plan(actions)

    def test_servo_mixed_or_out_of_car_calibration_targets_are_rejected(self):
        self.cfg["servo"]["limits"]["pitch"] = [1150, 1800]
        for targets in ({"pitch": 2000}, {"pitch": 1500, "gripper": 2200},
                        {"gripper": 1500.5}, {"gripper": "2000"}, {"gripper": True}, {}):
            with self.subTest(targets=targets), self.assertRaises(ValueError):
                self.plan([dict(type="servo", targets=targets)])

    def test_prompt_uses_selected_car_calibration_and_never_whole_config(self):
        self.cfg["car"]["profile_id"] = "car-two"
        self.cfg["servo"]["limits"]["pitch"] = [1150, 1800]
        self.cfg["private_api_key"] = "secret-not-for-model"
        prompt = build_system_prompt(self.cfg)
        self.assertIn("car-two", prompt)
        self.assertIn("1800", prompt)
        self.assertNotIn("secret-not-for-model", prompt)

    def test_conversational_empty_plan_never_invents_motion(self):
        self.assertEqual(self.plan([]).summary, "无可执行动作")
        for raw in ({"actions": [], "action": []}, {"response": "你好"}, {"actions": [], "code": "print(1)"}):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                parse_plan(raw, self.cfg)


class Context:
    def __init__(self):
        self.cfg = copy.deepcopy(DEFAULTS)
        self.cfg["base"]["publish_hz"] = 20
        self.state = CarState()
        self.state.update(ws_connected=True, cam_connected=True)
        self.bridge = types.SimpleNamespace(connected=True)
        self.app = types.SimpleNamespace(camera=types.SimpleNamespace(connected=True, last_frame_at=time.monotonic()))
        self.sent = []
        self.hold = False
        self.publish_ok = True

    def publish_twist(self, vx=0., wz=0., quiet=False, *, vy=0.):
        self.sent.append((time.monotonic(), "move", (vx, vy, wz)))
        return self.publish_ok

    def publish_servo(self, targets, duration):
        self.sent.append((time.monotonic(), "servo", dict(targets)))
        return self.publish_ok

    def stop_base(self):
        self.sent.append((time.monotonic(), "move", (0., 0., 0.)))

    def motion_interlock(self):
        return dict(enabled=self.hold, blocked=self.hold)


class AIActionRunnerTests(unittest.TestCase):
    def setUp(self):
        self.ctx = Context()
        self.permit = True
        self.notifications = []
        self.runner = AIActionRunner(self.ctx, allowed=lambda: self.permit, notify=self.notifications.append)
        self.addCleanup(self.runner.close)

    def move(self, seconds=.12, vx=.06):
        return parse_plan(dict(actions=[dict(type="move", vx=vx, seconds=seconds)]), self.ctx.cfg)

    def nonzero(self):
        return [item for item in self.ctx.sent if item[1] == "move" and any(item[2])]

    def finished(self):
        wait_until(lambda: not self.runner.motion_active)

    def test_idle_stop_and_empty_plan_do_not_interrupt_another_feature(self):
        self.assertFalse(self.runner.stop())
        self.assertFalse(self.runner.start(parse_plan(dict(actions=[]), self.ctx.cfg)))
        self.assertEqual(self.ctx.sent, [])

    def test_offline_or_other_motion_owner_prevents_start_without_zero_injection(self):
        self.permit = False
        self.assertFalse(self.runner.start(self.move()))
        self.permit = True
        self.ctx.state.update(ws_connected=False)
        self.assertFalse(self.runner.start(self.move()))
        self.ctx.state.update(ws_connected=True, cam_connected=False)
        self.assertFalse(self.runner.start(self.move()))
        self.assertEqual(self.ctx.sent, [])

    def test_manually_constructed_bad_plan_and_changed_calibration_are_revalidated(self):
        with self.assertRaises(ValueError):
            self.runner.start(AIPlan((AIStep("move", vx=.5, seconds=1),)))
        plan = parse_plan(dict(actions=[dict(type="servo", targets={"pitch": 2000})]), self.ctx.cfg)
        self.ctx.cfg["servo"]["limits"]["pitch"] = [1150, 1800]
        with self.assertRaises(ValueError):
            self.runner.start(plan)
        self.assertEqual(self.ctx.sent, [])

    def test_sequence_reaches_servos_and_final_stop_even_without_model_stop(self):
        plan = parse_plan(dict(actions=[dict(type="move", vx=.06, seconds=.04),
                        dict(type="servo", targets={"gripper": 2000}, duration=.1),
                        dict(type="move", vx=-.06, seconds=.04)]), self.ctx.cfg)
        self.assertTrue(self.runner.start(plan))
        self.finished()
        self.assertTrue(any(item[1] == "servo" for item in self.ctx.sent))
        self.assertTrue(any(item[2] == (-.06, 0., 0.) for item in self.ctx.sent))
        self.assertEqual(self.ctx.sent[-1][2], (0., 0., 0.))
        self.assertIn("完成", self.runner.phase)

    def test_traffic_hold_pauses_duration_and_resumes_only_remaining_task(self):
        self.ctx.hold = True
        self.assertTrue(self.runner.start(self.move(.12)))
        time.sleep(.14)
        self.assertTrue(self.runner.running)
        self.assertEqual(self.nonzero(), [])
        self.ctx.hold = False
        wait_until(lambda: bool(self.nonzero()))
        started = self.nonzero()[0][0]
        self.finished()
        self.assertGreaterEqual(self.ctx.sent[-1][0] - started, .09)

    def test_explicit_stop_cancels_all_later_steps_and_never_restarts_on_clear(self):
        plan = parse_plan(dict(actions=[dict(type="move", vx=.06, seconds=2),
                            dict(type="servo", targets={"gripper": 1500}, duration=.1)]), self.ctx.cfg)
        self.assertTrue(self.runner.start(plan))
        wait_until(lambda: bool(self.nonzero()))
        self.assertTrue(self.runner.stop())
        self.finished()
        count = len(self.ctx.sent)
        self.ctx.hold = False
        time.sleep(.12)
        self.assertEqual(len(self.ctx.sent), count)
        self.assertFalse(any(item[1] == "servo" for item in self.ctx.sent))
        self.assertEqual(self.ctx.sent[-1][2], (0., 0., 0.))
        self.assertFalse(self.runner.stop())
        self.assertEqual(len(self.ctx.sent), count)

    def test_stop_during_servo_prevents_following_move_or_gripper_command(self):
        plan = parse_plan(dict(actions=[dict(type="servo", targets={"gripper": 2000}, duration=.5),
                            dict(type="move", vx=.06, seconds=.1),
                            dict(type="servo", targets={"gripper": 1500}, duration=.1)]), self.ctx.cfg)
        self.assertTrue(self.runner.start(plan))
        wait_until(lambda: any(item[1] == "servo" for item in self.ctx.sent))
        self.runner.stop()
        self.finished()
        self.assertEqual([item[2] for item in self.ctx.sent if item[1] == "servo"], [{"gripper": 2000}])
        self.assertEqual(self.nonzero(), [])

    def test_disconnect_permission_loss_or_session_change_cancels_without_replay(self):
        for condition in ("ros", "bridge", "camera", "identity", "calibration", "permit"):
            with self.subTest(condition=condition):
                self.ctx = Context()
                self.permit = True
                runner = AIActionRunner(self.ctx, allowed=lambda: self.permit)
                self.addCleanup(runner.close)
                self.assertTrue(runner.start(self.move(2)))
                wait_until(lambda: bool(self.nonzero()))
                if condition == "ros":
                    self.ctx.state.update(ws_connected=False)
                elif condition == "bridge":
                    self.ctx.bridge = types.SimpleNamespace(connected=True)
                elif condition == "camera":
                    self.ctx.app.camera = object()
                elif condition == "identity":
                    self.ctx.cfg["car"]["host"] = "192.0.2.10"
                elif condition == "calibration":
                    self.ctx.cfg["servo"]["limits"]["pitch"] = [1150, 1800]
                else:
                    self.permit = False
                wait_until(lambda: not runner.motion_active)
                count = len(self.ctx.sent)
                self.ctx.state.update(ws_connected=True)
                self.permit = True
                time.sleep(.06)
                self.assertEqual(len(self.ctx.sent), count)
                self.assertIn("停止", runner.phase)

    def test_transport_failure_stops_remaining_plan(self):
        self.ctx.publish_ok = False
        self.assertTrue(self.runner.start(self.move()))
        self.finished()
        self.assertIn("发送失败", self.runner.phase)
        self.assertEqual(self.ctx.sent[-1][2], (0., 0., 0.))

    def test_ownership_query_does_not_block_on_transport_or_release_early(self):
        entered, release = threading.Event(), threading.Event()
        original = self.ctx.publish_twist
        def delayed(*args, **kwargs):
            entered.set()
            release.wait(1.)
            return original(*args, **kwargs)
        self.ctx.publish_twist = delayed
        self.assertTrue(self.runner.start(self.move(.05)))
        self.assertTrue(entered.wait(.5))
        before = time.monotonic()
        self.assertTrue(self.runner.motion_active)
        self.assertLess(time.monotonic() - before, .05)
        stopped = threading.Thread(target=self.runner.stop)
        stopped.start()
        time.sleep(.03)
        self.assertTrue(self.runner.motion_active)
        release.set()
        stopped.join(1.)
        self.finished()
        self.assertEqual(self.ctx.sent[-1][2], (0., 0., 0.))

    def test_closed_runner_cannot_execute_late_result(self):
        self.runner.close()
        self.assertFalse(self.runner.start(self.move()))
        self.assertEqual(self.ctx.sent, [])

    def test_motion_ownership_stays_reserved_until_final_stop_send_finishes(self):
        entered, release = threading.Event(), threading.Event()
        original = self.ctx.stop_base
        def delayed_stop():
            entered.set()
            release.wait(1.)
            original()
        self.ctx.stop_base = delayed_stop
        try:
            self.assertTrue(self.runner.start(self.move(.02)))
            self.assertTrue(entered.wait(.5))
            self.assertTrue(self.runner.motion_active)
            release.set()
            self.finished()
            self.assertEqual(self.ctx.sent[-1][2], (0., 0., 0.))
        finally:
            release.set()


if __name__ == "__main__":
    unittest.main()
