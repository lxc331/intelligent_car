"""Desktop AI coordination: canceled work cannot execute after reconnect."""
import copy
import json
import threading
import time
import tkinter as tk
import types
import unittest
from unittest.mock import Mock, patch

from carapp.config import DEFAULTS
from carapp.core.ai_actions import parse_plan
from carapp.features.ai_assistant import AIAssistantFeature
from carapp.features.base import AppContext
from carapp.model.state import CarState


PLAN = json.dumps(dict(actions=[dict(type="move", vx=.06, seconds=.03)], response="前进"))


def wait_until(predicate, seconds=1.5):
    deadline = time.monotonic() + seconds
    while not predicate() and time.monotonic() < deadline:
        time.sleep(.005)
    if not predicate():
        raise AssertionError("AI feature worker did not finish")


class CameraStub(tk.Frame):
    def __init__(self, parent, ctx):
        super().__init__(parent)
        self.on_tick = Mock()

    def on_close(self):
        pass


class AIFeatureTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.features = []
        self.sent, self.global_stops = [], []
        self.patch_env = patch.dict('os.environ', {'DEEPSEEK_API_KEY': ''})
        self.patch_env.start()
        self.camera_patch = patch('carapp.features.ai_assistant.CameraPanel', CameraStub)
        self.camera_patch.start()
        self.prefs_patch = patch('carapp.features.ai_assistant.load_preferences', return_value={})
        self.prefs_patch.start()
        self.feature = self.make_feature()

    def tearDown(self):
        for feature in self.features:
            feature.on_close()
        self.prefs_patch.stop()
        self.camera_patch.stop()
        self.patch_env.stop()
        self.root.destroy()

    def make_feature(self, connected=True):
        state = CarState()
        state.update(ws_connected=connected, cam_connected=connected)
        app = types.SimpleNamespace(cfg=copy.deepcopy(DEFAULTS), root=self.root, state=state,
            features=[], _connection_generation=1, frame_listeners=[], log=lambda message: None,
            camera=object(), bridge=types.SimpleNamespace(connected=connected,
                publish=lambda topic, message: self.sent.append(message) or True))
        app.stop_all_motion = lambda: self.global_stops.append(True)
        feature = AIAssistantFeature(AppContext(app))
        app.features = [feature]
        feature.build(self.root)
        feature.api_key.set('unit-test-key')
        feature.on_activate()
        self.features.append(feature)
        return feature

    def tick(self, feature=None):
        feature = feature or self.feature
        feature.on_background_tick(feature.ctx.state.snapshot())

    def pending_plan(self, feature=None, execute=True):
        feature = feature or self.feature
        entered, release = threading.Event(), threading.Event()
        def plan(text, prompt, cancel_event=None):
            entered.set()
            release.wait(1.)
            return PLAN
        client = types.SimpleNamespace(plan=plan)
        with patch('carapp.features.ai_assistant.AIClient', return_value=client):
            feature.submit_command(execute, text="前进一秒")
        self.assertTrue(entered.wait(.5))
        return release

    def finish_pending(self, release, feature=None):
        feature = feature or self.feature
        release.set()
        wait_until(lambda: not feature._events.empty())
        self.tick(feature)

    def test_global_stop_words_bypass_api_key_and_current_busy_state(self):
        self.feature.api_key.set('')
        self.feature.busy = True
        with patch('carapp.features.ai_assistant.AIClient') as client:
            self.feature.submit_command(True, text="停车！")
            client.assert_not_called()
        self.assertEqual(len(self.global_stops), 1)

    def test_idle_ai_stop_never_sends_zero_to_another_feature(self):
        other = types.SimpleNamespace(motion_active=True)
        self.feature.ctx.app.features.append(other)
        self.feature.stop_all()
        self.assertEqual(self.sent, [])
        self.assertEqual(self.global_stops, [])

    def test_global_cancellation_discards_late_model_response(self):
        release = self.pending_plan()
        self.feature.stop_all()
        self.finish_pending(release)
        self.assertIsNone(self.feature._plan)
        self.assertFalse(self.feature.motion_active)
        self.assertEqual(self.sent, [])

    def test_deactivate_discards_pending_execute_and_clears_permission(self):
        release = self.pending_plan()
        self.feature.on_deactivate()
        self.finish_pending(release)
        self.assertFalse(self.feature._permit.is_set())
        self.assertIsNone(self.feature._plan)
        self.assertEqual(self.sent, [])

    def test_changed_api_configuration_discards_pending_execution(self):
        release = self.pending_plan()
        self.feature.model.set('replacement-model')
        self.finish_pending(release)
        self.assertIsNone(self.feature._plan)
        self.assertFalse(self.feature.motion_active)
        self.assertEqual(self.sent, [])

    def test_car_or_ros_session_change_discards_old_api_results(self):
        for condition in ('car', 'bridge', 'generation'):
            with self.subTest(condition=condition):
                feature = self.make_feature()
                release = self.pending_plan(feature)
                if condition == 'car':
                    feature.ctx.cfg['car']['host'] = '192.0.2.11'
                elif condition == 'bridge':
                    feature.ctx.app.bridge = types.SimpleNamespace(connected=True, publish=Mock(return_value=True))
                else:
                    feature.ctx.app._connection_generation += 1
                self.tick(feature)
                self.finish_pending(release, feature)
                self.assertIsNone(feature._plan)
                self.assertFalse(feature.motion_active)
        self.assertEqual(self.sent, [])

    def test_disconnect_then_reconnect_does_not_authorize_inflight_plan(self):
        release = self.pending_plan()
        self.feature.ctx.state.update(ws_connected=False)
        self.tick()
        self.feature.ctx.state.update(ws_connected=True)
        self.finish_pending(release)
        self.assertIsNone(self.feature._plan)
        self.assertEqual(self.sent, [])

    def test_initially_offline_readonly_connection_test_and_planning_still_work(self):
        feature = self.make_feature(connected=False)
        client = types.SimpleNamespace(check_connection=lambda: 'OK', plan=lambda *args, **kwargs: PLAN)
        with patch('carapp.features.ai_assistant.AIClient', return_value=client):
            feature.test_connection()
            wait_until(lambda: not feature._events.empty())
            self.tick(feature)
            self.assertIn('API 已连接', feature._status)
            feature.submit_command(False, text="前进一秒")
        wait_until(lambda: not feature._events.empty())
        self.tick(feature)
        self.assertIsNotNone(feature._plan)
        self.assertFalse(feature.motion_active)
        feature.execute_plan()
        self.assertFalse(feature.motion_active)
        self.assertEqual(self.sent, [])

    def test_background_results_do_not_refresh_hidden_controls_or_camera(self):
        before = self.feature.status.cget('text')
        self.feature._events.put((self.feature._generation, 'test', ('OK', None), None))
        self.tick()
        self.assertEqual(self.feature.status.cget('text'), before)
        self.feature.cam.on_tick.assert_not_called()
        self.feature.on_tick(self.feature.ctx.state.snapshot())
        self.assertIn('API 已连接', self.feature.status.cget('text'))
        self.feature.cam.on_tick.assert_called_once()

    def test_saved_preferences_exclude_both_keys_and_display_redacts_them(self):
        self.feature.api_key.set('unit-test-key')
        self.feature.asr_key.set('unit-asr-key')
        with patch('carapp.features.ai_assistant.save_preferences') as save:
            self.feature.save_settings()
        values = save.call_args.args[1]
        self.assertFalse(any('key' in name.lower() for name in values))
        self.assertNotIn('unit-test-key', str(values))
        self.feature._append('错误', 'unit-test-key and unit-asr-key')
        self.assertNotIn('unit-test-key', str(self.feature._history))
        self.assertNotIn('unit-asr-key', str(self.feature._history))

    def test_deepseek_environment_key_is_not_prefilled_for_another_provider(self):
        with patch.dict('os.environ', {'DEEPSEEK_API_KEY': 'private-deepseek-test-key'}), \
             patch('carapp.features.ai_assistant.load_preferences', return_value={'provider':'通义千问'}):
            feature = AIAssistantFeature(self.feature.ctx)
            feature.build(self.root)
            self.features.append(feature)
            self.assertEqual(feature.api_key.get(), '')
            self.assertEqual(feature.base_url.get(), 'https://dashscope.aliyuncs.com/compatible-mode/v1')
            self.assertEqual(feature.model.get(), 'qwen-plus')

    def test_recording_start_stop_and_transcription_use_workers(self):
        main_thread = threading.get_ident()
        calls = []
        class Recorder:
            recording = False
            def start(self):
                calls.append(('start', threading.get_ident()))
                self.recording = True
            def stop(self):
                calls.append(('stop', threading.get_ident()))
                self.recording = False
                return b'wav-test'
            def cancel(self):
                calls.append(('cancel', threading.get_ident()))
                self.recording = False
        def transcribe(audio, model, **kwargs):
            calls.append(('transcribe', threading.get_ident()))
            self.assertEqual(audio, b'wav-test')
            return '停车'
        client = types.SimpleNamespace(transcribe=transcribe)
        self.feature.cloud_voice.set(True)
        self.feature.asr_key.set('unit-asr-key')
        with patch('carapp.features.ai_assistant.MicrophoneRecorder', Recorder), \
             patch('carapp.features.ai_assistant.AIClient', return_value=client):
            self.feature.toggle_recording()
            wait_until(lambda: not self.feature._events.empty())
            self.tick()
            self.assertTrue(self.feature._recording)
            self.assertFalse(self.feature._voice_opening)
            self.feature.toggle_recording()
            wait_until(lambda: not self.feature._events.empty())
            self.tick()
        self.assertEqual([name for name, _ in calls], ['start', 'stop', 'transcribe'])
        self.assertTrue(all(thread != main_thread for _, thread in calls))
        self.assertEqual(len(self.global_stops), 1)
        self.assertEqual(self.sent, [])

    def test_cancel_microphone_closes_off_the_ui_thread(self):
        closed = threading.Event()
        thread_ids = []
        def cancel():
            thread_ids.append(threading.get_ident())
            closed.set()
        self.feature._recorder = types.SimpleNamespace(cancel=cancel)
        self.feature._recording = True
        self.feature.stop_all()
        self.assertTrue(closed.wait(.5))
        self.assertNotEqual(thread_ids[0], threading.get_ident())
        self.assertIsNone(self.feature._recorder)

    def test_changed_servo_calibration_rejects_retained_plan_without_callback_error(self):
        self.feature._plan = parse_plan(dict(actions=[dict(type="servo", targets={"pitch": 2000})]),
                                        self.feature.ctx.cfg)
        self.feature._plan_identity = self.feature._car_identity()
        self.feature.ctx.cfg['servo']['limits']['pitch'] = [1150, 1800]
        self.feature.execute_plan()
        self.assertIsNone(self.feature._plan)
        self.assertIn('超出', self.feature._status)
        self.assertFalse(self.feature.motion_active)
        self.assertEqual(self.sent, [])

    def test_canceled_dictation_callback_cannot_open_system_input_later(self):
        with patch.object(self.root, 'after') as after, \
             patch('carapp.features.ai_assistant.start_system_dictation') as start:
            self.feature.system_dictation()
            callback = after.call_args.args[1]
            self.feature.stop_all()
            callback()
            start.assert_not_called()


if __name__ == '__main__':
    unittest.main()
