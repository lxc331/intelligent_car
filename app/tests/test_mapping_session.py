"""Background lifetime, late results, and latest-map delivery without a car."""
import threading
import time
import unittest

from carapp.core.map_data import MapModel
from carapp.core.mapping_session import MapFeed, MappingSession


def wait_until(predicate, seconds=2):
    end = time.monotonic() + seconds
    while not predicate() and time.monotonic() < end:
        time.sleep(.005)
    if not predicate():
        raise AssertionError("background operation did not finish")


class Remote:
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()
        self.stops = 0
        self.closed = False

    def connect(self):
        return dict(connected=True, running=False, owned=False)

    def start(self):
        self.entered.set()
        self.release.wait(2)
        return dict(running=True, owned=True)

    def stop(self):
        self.stops += 1
        return dict(running=False, owned=False)

    def close(self):
        self.closed = True

    def status(self):
        return dict(connected=False, running=False, owned=False)


class MappingSessionTests(unittest.TestCase):
    def session(self):
        backend = Remote()
        session = MappingSession(backend)
        self.assertTrue(session.submit("connect"))
        wait_until(lambda: not session.snapshot()["busy"])
        self.addCleanup(session.close_async)
        return session, backend

    def test_close_during_start_cleans_up_late_owned_process(self):
        session, backend = self.session()
        self.assertTrue(session.submit("start"))
        self.assertTrue(backend.entered.wait(1))
        self.assertFalse(session.submit("status"))
        session.close_async()
        self.assertFalse(session.submit("start"))
        backend.release.set()
        wait_until(lambda: backend.closed)
        self.assertEqual(backend.stops, 1)
        self.assertFalse(session.snapshot()["connected"])

    def test_status_disconnection_revokes_connection_state(self):
        session, backend = self.session()
        session.submit("status")
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertFalse(session.snapshot()["connected"])
        self.assertFalse(session.submit("start"))

    def slow_status(self, connected=True):
        session, backend = self.session()
        entered, release = threading.Event(), threading.Event()
        backend.start_called = threading.Event()
        def status():
            entered.set()
            release.wait(2)
            return dict(connected=connected, running=False, owned=False)
        def start():
            backend.start_called.set()
            return dict(running=True, owned=True)
        backend.status, backend.start = status, start
        session.drain()
        session.submit("status")
        self.assertTrue(entered.wait(1))
        self.addCleanup(release.set)
        return session, backend, release

    def test_explicit_start_waits_for_status_then_runs_once(self):
        session, backend, release = self.slow_status()
        self.assertTrue(session.submit("start"))
        self.assertEqual(session.snapshot()["operation"], "start")
        self.assertFalse(backend.start_called.is_set())
        self.assertFalse(session.submit("start"))
        self.assertFalse(session.submit("status"))
        release.set()
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertTrue(backend.start_called.is_set())
        self.assertTrue(session.snapshot()["running"])
        self.assertEqual([event["operation"] for event in session.drain()], ["status", "start"])

    def test_disconnected_status_cancels_queued_start_instead_of_replaying(self):
        session, backend, release = self.slow_status(connected=False)
        self.assertTrue(session.submit("start"))
        release.set()
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertFalse(backend.start_called.is_set())
        events = session.drain()
        self.assertEqual(events[-1]["operation"], "start")
        self.assertIn("断开", events[-1]["error"])

    def test_close_during_poll_discards_queued_action_and_cleans_old_owner(self):
        session, backend, release = self.slow_status()
        self.assertTrue(session.submit("start"))
        session.close_async()
        release.set()
        wait_until(lambda: backend.closed)
        self.assertFalse(backend.start_called.is_set())
        self.assertEqual(backend.stops, 1)

    def test_start_failure_stops_any_late_owned_process(self):
        session, backend = self.session()
        def failure():
            raise RuntimeError("process startup uncertain")
        backend.start = failure
        session.submit("start")
        wait_until(lambda: not session.snapshot()["busy"])
        self.assertEqual(backend.stops, 1)
        self.assertFalse(session.snapshot()["running"])

    def test_partial_save_error_is_exposed_without_reporting_success(self):
        class Failure(RuntimeError):
            partial = dict(remote_dir="/maps/partial", local_dir=None)
        session, backend = self.session()
        def save():
            raise Failure("download interrupted")
        backend.save_map = save
        session.drain()
        session.submit("save_map")
        wait_until(lambda: not session.snapshot()["busy"])
        event = session.drain()[0]
        self.assertEqual(event["result"]["remote_dir"], "/maps/partial")
        self.assertIn("interrupted", event["error"])

    def test_map_feed_reset_invalidates_old_vehicle_inflight_frame(self):
        entered, release = threading.Event(), threading.Event()
        class SlowModel(MapModel):
            def accept_map(self, msg, received_at, **kw):
                entered.set(); release.wait(2)
                return super().accept_map(msg, received_at, **kw)
        model = SlowModel()
        feed = MapFeed(model)
        self.addCleanup(feed.close)
        message = dict(header=dict(frame_id="map"), info=dict(width=1,height=1,resolution=.05,
            origin=dict(position=dict(x=0,y=0), orientation=dict(x=0,y=0,z=0,w=1))),data=[0])
        feed.put(message, time.monotonic())
        self.assertTrue(entered.wait(1))
        before = time.monotonic()
        feed.reset()
        self.assertLess(time.monotonic()-before, .1)
        release.set()
        time.sleep(.03)
        self.assertIsNone(model.snapshot()["grid"])


if __name__ == "__main__":
    unittest.main()
