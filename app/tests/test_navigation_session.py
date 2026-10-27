"""Navigation worker ownership, background status and cancel race boundaries."""
import threading
import time
import unittest

from carapp.core.navigation_session import NavigationSession


def wait_until(predicate, seconds=2):
    end = time.monotonic() + seconds
    while not predicate() and time.monotonic() < end:
        time.sleep(.005)
    if not predicate():
        raise AssertionError("background operation did not finish")


class Remote:
    def __init__(self):
        self.calls = []
        self.closed = False
        self.revoked = 0
        self.running = False
        self.goal_state = "idle"
        self.motion_active = False

    def connect(self):
        self.calls.append("connect")
        return dict(connected=True, running=False, ready=False, owned=False)

    def start(self, bundle, speed=.06):
        self.calls.append("start")
        self.running = True
        return dict(running=True, ready=True, owned=True)

    def status(self):
        self.calls.append("status")
        return dict(connected=True, running=self.running, ready=self.running,
                    goal_state=self.goal_state, motion_active=self.motion_active)

    def initial_pose(self, *pose):
        self.calls.append("initial_pose")
        return dict(ready=True)

    def plan(self, *goal):
        self.calls.append("plan")
        return dict(path=[(0., 0.), goal[:2]])

    def navigate(self, *goal, intent_epoch=None):
        self.calls.append("navigate")
        if intent_epoch is not None and intent_epoch != self.revoked:
            return dict(goal_state="canceled", motion_active=False)
        self.goal_state = "active"
        self.motion_active = True
        return dict(goal_state="active", motion_active=True)

    def request_cancel(self):
        self.revoked += 1

    def capture_intent(self):
        return self.revoked

    def cancel(self):
        self.calls.append("cancel")
        self.goal_state = "canceled"
        self.motion_active = False
        return dict(goal_state="canceled", motion_active=False)

    def stop(self):
        self.calls.append("stop")
        self.running = self.motion_active = False
        self.goal_state = "stopped"
        return dict(running=False, ready=False, owned=False, motion_active=False, goal_state="stopped")

    def close(self):
        self.closed = True


class NavigationSessionTests(unittest.TestCase):
    def session(self, *, running=True, interval=10):
        backend = Remote()
        session = NavigationSession(backend, heartbeat_interval=interval)
        self.addCleanup(self.close_session, session, backend)
        self.assertTrue(session.submit("connect"))
        wait_until(lambda: not session.snapshot()["busy"])
        if running:
            self.assertTrue(session.submit("start", {"yaml_text": "test"}))
            wait_until(lambda: not session.snapshot()["busy"])
        session.drain()
        return session, backend

    @staticmethod
    def close_session(session, backend):
        session.close_async()
        wait_until(lambda: backend.closed)

    def block(self, backend, operation, result=None):
        entered, release = threading.Event(), threading.Event()
        original = getattr(backend, operation)
        def blocking(*args, **kwargs):
            entered.set()
            release.wait(2)
            return original(*args, **kwargs) if result is None else result
        setattr(backend, operation, blocking)
        self.addCleanup(release.set)
        return entered, release

    def test_navigation_requires_start_and_ready_before_motion_is_reserved(self):
        session, backend = self.session(running=False)
        self.assertFalse(session.submit("navigate", 1., 2., 0.))
        self.assertFalse(session.snapshot()["motion_active"])
        self.assertNotIn("navigate", backend.calls)

    def test_background_status_polls_without_any_page_tick(self):
        session, backend = self.session(interval=.05)
        wait_until(lambda: backend.calls.count("status") >= 2)
        self.assertTrue(session.snapshot()["connected"])

    def test_explicit_goal_reserves_motion_before_remote_acknowledgement(self):
        session, backend = self.session()
        entered, release = self.block(backend, "navigate")
        self.assertTrue(session.submit("navigate", 1., 2., 0.))
        self.assertTrue(entered.wait(1))
        self.assertTrue(session.snapshot()["motion_active"])
        self.assertFalse(session.submit("plan", 3., 4., 0.))
        release.set()
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertTrue(session.snapshot()["motion_active"])

    def test_user_action_queues_once_behind_status_and_idle_result_cannot_clear_latch(self):
        session, backend = self.session()
        entered, release = self.block(backend, "status")
        session.submit("status")
        self.assertTrue(entered.wait(1))
        self.assertTrue(session.submit("navigate", 1., 2., 0.))
        self.assertEqual(session.snapshot()["operation"], "navigate")
        self.assertTrue(session.snapshot()["motion_active"])
        self.assertFalse(session.submit("navigate", 2., 3., 0.))
        release.set()
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertEqual(backend.calls.count("navigate"), 1)
        self.assertTrue(session.snapshot()["motion_active"])

    def test_cancel_discards_queued_goal_and_is_not_starved_by_status(self):
        session, backend = self.session()
        entered, release = self.block(backend, "status")
        session.submit("status")
        self.assertTrue(entered.wait(1))
        self.assertTrue(session.submit("navigate", 1., 2., 0.))
        self.assertTrue(session.request_cancel())
        self.assertEqual(backend.revoked, 1)
        self.assertFalse(session.submit("navigate", 2., 3., 0.))
        release.set()
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertNotIn("navigate", backend.calls)
        self.assertIn("cancel", backend.calls)
        self.assertFalse(session.snapshot()["motion_active"])
        events = session.drain()
        self.assertTrue(any(event.get("canceled") and event["operation"] == "navigate" for event in events))

    def test_late_navigate_acknowledgement_after_cancel_cannot_restore_motion(self):
        session, backend = self.session()
        entered, release = self.block(backend, "navigate")
        session.submit("navigate", 1., 2., 0.)
        self.assertTrue(entered.wait(1))
        session.request_cancel()
        self.assertTrue(session.snapshot()["motion_active"])
        release.set()
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertFalse(session.snapshot()["motion_active"])
        self.assertEqual(session.snapshot()["goal_state"], "canceled")
        self.assertTrue(next(event for event in session.drain() if event["operation"] == "navigate")["stale"])

    def test_close_during_start_stops_late_owned_runtime(self):
        session, backend = self.session(running=False)
        entered, release = self.block(backend, "start")
        session.submit("start", {})
        self.assertTrue(entered.wait(1))
        session.close_async()
        self.assertGreaterEqual(backend.revoked, 1)
        self.assertFalse(session.submit("start", {}))
        release.set()
        wait_until(lambda: backend.closed)
        self.assertEqual(backend.calls.count("stop"), 1)
        self.assertFalse(session.snapshot()["running"])

    def test_cancel_during_start_shuts_down_instead_of_leaving_unused_stack(self):
        session, backend = self.session(running=False)
        entered, release = self.block(backend, "start")
        session.submit("start", {})
        self.assertTrue(entered.wait(1))
        session.request_cancel()
        release.set()
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertEqual(backend.calls[-1], "stop")
        self.assertFalse(session.snapshot()["running"])

    def test_terminal_status_releases_motion_without_launching_anything(self):
        session, backend = self.session()
        session.submit("navigate", 1., 2., 0.)
        wait_until(lambda: not session.snapshot()["busy"])
        backend.goal_state, backend.motion_active = "succeeded", False
        session.submit("status")
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertFalse(session.snapshot()["motion_active"])
        self.assertEqual(backend.calls.count("navigate"), 1)

    def test_status_failure_revokes_motion_and_never_replays_goal_after_recovery(self):
        session, backend = self.session()
        session.submit("navigate", 1., 2., 0.)
        wait_until(lambda: not session.snapshot()["busy"])
        def failure():
            raise RuntimeError("SSH lost")
        backend.status = failure
        session.submit("status")
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertFalse(session.snapshot()["connected"])
        self.assertFalse(session.snapshot()["ready"])
        self.assertGreaterEqual(backend.revoked, 1)
        self.assertFalse(session.submit("navigate", 1., 2., 0.))
        self.assertEqual(backend.calls.count("navigate"), 1)

    def test_failed_cancel_keeps_uncertain_motion_reserved_until_cleanup(self):
        session, backend = self.session()
        session.submit("navigate", 1., 2., 0.)
        wait_until(lambda: not session.snapshot()["busy"])
        def failure():
            raise RuntimeError("cancel confirmation unavailable")
        backend.cancel = failure
        session.request_cancel()
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertTrue(session.snapshot()["motion_active"])

    def test_start_failure_attempts_owned_cleanup_and_reports_error(self):
        session, backend = self.session(running=False)
        def failure(*args, **kwargs):
            raise RuntimeError("startup uncertain")
        backend.start = failure
        session.submit("start", {})
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertIn("stop", backend.calls)
        self.assertFalse(session.snapshot()["running"])
        event = next(event for event in session.drain() if event["operation"] == "start")
        self.assertIn("startup uncertain", event["error"])


    def test_repeated_cancel_does_not_invalidate_its_own_confirmation(self):
        session, backend = self.session()
        session.submit("navigate", 1., 2., 0.)
        wait_until(lambda: not session.snapshot()["busy"])
        entered, release = self.block(backend, "cancel")
        session.request_cancel()
        self.assertTrue(entered.wait(1))
        for _ in range(5):
            session.request_cancel()
        self.assertEqual(backend.revoked, 1)
        release.set()
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertFalse(session.snapshot()["motion_active"])
        cancel = next(event for event in session.drain() if event["operation"] == "cancel")
        self.assertFalse(cancel["stale"])
        self.assertEqual(backend.calls.count("cancel"), 1)

    def test_shutdown_upgrade_survives_repeated_cancel_during_confirmation(self):
        session, backend = self.session()
        entered, release = self.block(backend, "cancel")
        session.request_cancel()
        self.assertTrue(entered.wait(1))
        session.request_cancel(stop=True)
        for _ in range(5):
            session.request_cancel()
            session.request_cancel(stop=True)
        self.assertEqual(backend.revoked, 2)
        release.set()
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertEqual(backend.calls[-1], "stop")
        self.assertEqual(backend.calls.count("stop"), 1)
        self.assertFalse(session.snapshot()["running"])

    def test_result_target_identifies_plans_that_finished_before_a_new_target(self):
        session, backend = self.session()
        session.submit("plan", 1., 2., .3)
        wait_until(lambda: not session.snapshot()["busy"])
        session.submit("plan", 4., 5., .6)
        wait_until(lambda: not session.snapshot()["busy"])
        plans = [event for event in session.drain() if event["operation"] == "plan"]
        self.assertEqual([event["target"] for event in plans], [(1., 2., .3), (4., 5., .6)])

    def test_cancel_between_dispatch_and_adapter_entry_rejects_old_intent(self):
        session, backend = self.session()
        entered, release = self.block(backend, "navigate")
        cancel_entered, cancel_release = self.block(backend, "cancel")
        session.submit("navigate", 1., 2., 0.)
        self.assertTrue(entered.wait(1))
        session.request_cancel()
        release.set()
        self.assertTrue(cancel_entered.wait(1))
        # Method entry resumed after revoke: the captured old intent cannot
        # activate the robot, even before serial cancellation confirms.
        self.assertFalse(backend.motion_active)
        cancel_release.set()
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertFalse(session.snapshot()["motion_active"])

    def test_terminal_error_status_releases_motion(self):
        session, backend = self.session()
        session.submit("navigate", 1., 2., 0.)
        wait_until(lambda: not session.snapshot()["busy"])
        backend.goal_state = "error"
        # The production runtime reports goal_state rather than a redundant
        # motion flag; this path must also free the public occupancy latch.
        backend.status = lambda: dict(connected=True, running=True, ready=True, goal_state="error")
        session.submit("status")
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertFalse(session.snapshot()["motion_active"])

    def test_unconfirmed_disconnected_cancel_does_not_claim_vehicle_already_stopped(self):
        session, backend = self.session()
        session.submit("navigate", 1., 2., 0.)
        wait_until(lambda: not session.snapshot()["busy"])
        backend.cancel = lambda: dict(connected=False, running=False, goal_state="canceled", canceled=False)
        session.request_cancel()
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertTrue(session.snapshot()["motion_active"])
        self.assertFalse(session.snapshot()["connected"])
        self.assertFalse(session.submit("navigate", 1., 2., 0.))

    def test_offline_shutdown_confirmed_after_lease_quarantine_releases_motion(self):
        session, backend = self.session()
        session.submit("navigate", 1., 2., 0.)
        wait_until(lambda: not session.snapshot()["busy"])
        backend.stop = lambda: dict(connected=False, running=False, ready=False,
                                   motion_stopped=True, motion_lease_seconds=0.)
        session.request_cancel(stop=True)
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertFalse(session.snapshot()["motion_active"])
        self.assertFalse(session.snapshot()["running"])

    def test_explicit_unconfirmed_stop_overrides_terminal_goal_label(self):
        session, backend = self.session()
        session.submit("navigate", 1., 2., 0.)
        wait_until(lambda: not session.snapshot()["busy"])
        backend.cancel = lambda: dict(connected=True, running=True, goal_state="canceled",
                                     motion_stopped=False)
        session.request_cancel()
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertTrue(session.snapshot()["motion_active"])


if __name__ == "__main__":
    unittest.main()
