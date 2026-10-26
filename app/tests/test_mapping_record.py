"""PC recording ownership and persistence errors must not leak recordings."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from carapp.core.mapping_record import MappingRecord


class Camera:
    def __init__(self, existing=None):
        self.record_path = existing
        self.stops = 0
    @property
    def recording(self):
        return bool(self.record_path)
    def start_record(self, folder):
        self.record_path = folder + "/video.mjpg"
        return self.record_path
    def stop_record(self):
        self.stops += 1
        self.record_path = None


class MappingRecordTests(unittest.TestCase):
    def test_camera_keyframes_and_remote_writes_are_saved_only_on_pc(self):
        with tempfile.TemporaryDirectory() as folder:
            camera = Camera()
            record = MappingRecord(folder, camera, ("car", "host"))
            record.event("开始建图", remote_paths=["/tmp/carapp_slam_token/slam.log"])
            record.frame("开始建图", (0, b"JPEG", "jpeg"))
            self.assertEqual((Path(folder)/"开始建图.jpg").read_bytes(), b"JPEG")
            record.finish()
            report = json.loads((Path(folder)/"操作记录.json").read_text(encoding="utf-8"))
            self.assertEqual(camera.stops, 1)
            self.assertEqual(report["remote_writes"], ["/tmp/carapp_slam_token/slam.log"])
            self.assertEqual(report["car_source_changes"], [])

    def test_manual_recording_and_later_replacement_are_not_stopped(self):
        with tempfile.TemporaryDirectory() as folder:
            camera = Camera("already-running.mjpg")
            record = MappingRecord(folder, camera, None)
            record.finish()
            self.assertEqual(camera.stops, 0)
        with tempfile.TemporaryDirectory() as folder:
            camera = Camera()
            record = MappingRecord(folder, camera, None)
            camera.record_path = "new-manual-video.mjpg"
            record.finish()
            self.assertEqual(camera.stops, 0)

    def test_initial_report_failure_stops_owned_video(self):
        with tempfile.TemporaryDirectory() as folder:
            camera = Camera()
            with patch.object(MappingRecord, "flush", side_effect=OSError("disk full")):
                with self.assertRaises(OSError):
                    MappingRecord(folder, camera, None)
            self.assertFalse(camera.recording)
            self.assertEqual(camera.stops, 1)


if __name__ == "__main__":
    unittest.main()
