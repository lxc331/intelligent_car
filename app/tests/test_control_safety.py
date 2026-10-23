"""No hardware required. Run: python -m unittest discover -s tests -v"""
import copy
import tempfile
import threading
import time
import types
import unittest
from pathlib import Path

from carapp.config import DEFAULTS, SAFE_LIMITS, normalize_servo_config
from carapp.features.base import AppContext
from carapp.features.gripper import GripperFeature
from carapp.model.state import CarState
from carapp.ui.camerapanel import CameraPanel


class ControlSafetyTests(unittest.TestCase):
    def setUp(self):
        self.cfg = copy.deepcopy(DEFAULTS)
        self.sent, self.logs = [], []
        bridge = types.SimpleNamespace(publish=lambda topic, msg: self.sent.append((topic, msg)) or True)
        self.app = types.SimpleNamespace(cfg=self.cfg, state=CarState(), bridge=bridge, log=self.logs.append)
        self.ctx = AppContext(self.app)

    def feature(self):
        f = GripperFeature(self.ctx)
        f.base_cfg, f.actions = self.cfg['base'], self.cfg['servo']['actions']
        f.var_level = types.SimpleNamespace(get=lambda: '慢')
        f.var_fwd = types.SimpleNamespace(get=lambda: '0')
        f.var_back = types.SimpleNamespace(get=lambda: '0')
        return f

    def test_boundaries_and_non_finite_values(self):
        for key, (lo, hi) in SAFE_LIMITS.items():
            self.assertTrue(self.ctx.publish_servo({key: lo}))
            self.assertTrue(self.ctx.publish_servo({key: hi}))
            count = len(self.sent)
            for bad in (lo-1, hi+1, float('nan'), float('inf'), 1500.5):
                self.assertFalse(self.ctx.publish_servo({key: bad}))
            self.assertEqual(len(self.sent), count)

    def test_rejects_entire_mixed_request(self):
        self.assertFalse(self.ctx.publish_servo({'pitch': 1300, 'gripper': 2200}))
        self.assertEqual(self.sent, [])

    def test_old_limits_and_presets_cannot_bypass_safety(self):
        self.cfg['servo']['limits']['gripper'] = [500, 2500]
        self.cfg['servo']['presets']['old'] = {'pitch': 1000, 'gripper': 2200}
        normalize_servo_config(self.cfg)
        self.assertEqual(self.cfg['servo']['presets']['old'], {'pitch': 1150, 'gripper': 2000})
        self.assertFalse(self.ctx.publish_servo({'gripper': 2200}))

    def test_all_presets_are_valid(self):
        for pose in self.cfg['servo']['presets'].values():
            self.assertTrue(self.ctx.publish_servo(pose))
        self.assertEqual(self.cfg['servo']['actions']['look_down'], 1300)
        self.assertEqual(self.cfg['servo']['actions']['look_forward'], 1500)

    def test_stop_cancels_nudge(self):
        f = self.feature()
        f.nudge(1, 0, .8)
        time.sleep(.1)
        f.stop_all()
        count = len(self.sent)
        time.sleep(.12)
        self.assertFalse(f._motion_thread.is_alive())
        self.assertFalse(any(msg.get('linear', {}).get('x', 0) for _, msg in self.sent[count:]))

    def test_hold_timeout_stops_without_button_release(self):
        f = self.feature()
        f.base_cfg['jog_max_sec'] = .12
        f.jog(1, 0)
        f._motion_thread.join(timeout=1)
        self.assertFalse(f._motion_thread.is_alive())
        self.assertEqual(self.sent[-1][1]['linear']['x'], 0)

    def test_sequence_abort_prevents_later_close(self):
        f = self.feature()
        f.run_pick_sequence()
        time.sleep(.08)
        f.abort_sequence()
        time.sleep(.15)
        self.assertFalse(f._busy)
        self.assertFalse(any(topic.endswith('set_state') and any(s['position'] == [1500]
                             for s in msg['state']) for topic, msg in self.sent))

    def test_snapshot_survives_display_consuming_pending_frame(self):
        with tempfile.TemporaryDirectory() as folder:
            panel = types.SimpleNamespace(_pending=None, _last_frame=(0, b'jpeg-data', 'jpeg'),
                       ctx=types.SimpleNamespace(save_dir=lambda: folder, log=self.logs.append))
            CameraPanel.snapshot(panel)
            files = list(Path(folder).glob('*.jpg'))
            self.assertEqual(len(files), 1)
            self.assertEqual(files[0].read_bytes(), b'jpeg-data')


if __name__ == '__main__':
    unittest.main()
