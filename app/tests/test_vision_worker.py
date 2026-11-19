"""Offline tests for the background vision worker (no GPU, no weights)."""
import time
import types
import unittest

from carapp.core.vision.detector import VisionObject
from carapp.core.vision.worker import VisionWorker


def wait_until(check, timeout=3.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if check():
            return True
        time.sleep(.01)
    return False


class FakeDetector:
    def __init__(self, delay=0., error=None):
        self.delay, self.error = delay, error
        self.calls = []

    def detect(self, payload, mode="jpeg"):
        self.calls.append(payload)
        if self.delay:
            time.sleep(self.delay)
        if self.error:
            raise ValueError(self.error)
        return types.SimpleNamespace(
            objects=(VisionObject("person", .9, (10, 20, 30, 40)),), image_size=(640, 480))


class WorkerTests(unittest.TestCase):
    def setUp(self):
        self.worker = VisionWorker(FakeDetector(), interval=0.)
        self.worker.start()

    def tearDown(self):
        self.worker.stop()

    def test_result_is_published(self):
        self.worker.submit(b"frame-1", "jpeg", received_at=100.)
        self.assertTrue(wait_until(lambda: self.worker.snapshot()))
        result = self.worker.snapshot()
        self.assertEqual(result["image_size"], (640, 480))
        self.assertEqual(result["objects"][0].label, "person")
        self.assertEqual(result["received_at"], 100.)
        self.assertEqual(result["error"], "")

    def test_newest_frame_wins_when_frames_arrive_faster(self):
        self.worker.stop()
        detector = FakeDetector(delay=.1)
        self.worker = VisionWorker(detector, interval=0.)
        self.worker.start()
        for index in range(5):
            self.worker.submit(b"frame-%d" % index)
        self.assertTrue(wait_until(lambda: self.worker.snapshot(), timeout=3.))
        time.sleep(.3)
        # One frame is being inferred while the rest collapse into a single slot.
        self.assertLess(len(detector.calls), 5)
        self.assertEqual(detector.calls[-1], b"frame-4")

    def test_error_is_reported_without_killing_the_thread(self):
        self.worker.stop()
        self.worker = VisionWorker(FakeDetector(error="解码失败"), interval=0.)
        self.worker.start()
        self.worker.submit(b"bad")
        self.assertTrue(wait_until(lambda: self.worker.snapshot()))
        self.assertIn("解码失败", self.worker.snapshot()["error"])
        self.assertEqual(self.worker.stats()["errors"], 1)
        self.assertTrue(self.worker.running)

    def test_reset_clears_result(self):
        self.worker.submit(b"frame")
        self.assertTrue(wait_until(lambda: self.worker.snapshot()))
        self.worker.reset()
        self.assertIsNone(self.worker.snapshot())

    def test_stop_clears_result_and_thread(self):
        self.worker.submit(b"frame")
        self.assertTrue(wait_until(lambda: self.worker.snapshot()))
        self.worker.stop()
        self.assertIsNone(self.worker.snapshot())
        self.assertFalse(self.worker.running)

    def test_submit_after_stop_is_refused(self):
        self.worker.stop()
        self.assertFalse(self.worker.submit(b"frame"))


if __name__ == "__main__":
    unittest.main()
