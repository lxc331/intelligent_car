"""Microphone tests use fake PortAudio; they never open a real input device."""
import io
import threading
import time
import types
import unittest
import wave
from unittest.mock import Mock, patch

from carapp.core.ai_voice import MicrophoneRecorder, start_system_dictation


def wait_for(predicate, timeout=1.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.002)
    if not predicate():
        raise AssertionError("microphone worker did not finish")


class CallbackStop(Exception):
    pass


class CallbackAbort(Exception):
    pass


class FakeStream:
    def __init__(self, **values):
        self.values = values
        self.callback = values["callback"]
        self.finished = values["finished_callback"]
        self.active = self.closed = False
        self.stop_calls = self.close_calls = 0
        self.stop_entered = self.stop_release = None
        self.close_error = None

    def start(self):
        self.active = True

    def emit(self, data, *, frames=None, status=False):
        try:
            self.callback(data, len(data)//2 if frames is None else frames, None, status)
        except (CallbackStop, CallbackAbort):
            self.active = False
            self.finished()

    def stop(self):
        self.stop_calls += 1
        if self.stop_entered:
            self.stop_entered.set()
            self.stop_release.wait(1.)
        if self.active:
            self.active = False
            self.finished()

    def close(self):
        self.close_calls += 1
        if self.close_error:
            raise self.close_error
        self.closed = True


class FakeSoundDevice:
    CallbackStop, CallbackAbort = CallbackStop, CallbackAbort

    def __init__(self):
        self.streams = []
        self.open_error = self.open_entered = self.open_release = None

    def RawInputStream(self, **values):
        if self.open_entered:
            self.open_entered.set()
            self.open_release.wait(1.)
        if self.open_error:
            raise self.open_error
        stream = FakeStream(**values)
        self.streams.append(stream)
        return stream

    def query_devices(self):
        return [dict(name="speaker", max_input_channels=0),
                dict(name="default microphone", max_input_channels=1)]


class MicrophoneRecorderTests(unittest.TestCase):
    def setUp(self):
        self.sd = FakeSoundDevice()
        self.loader = patch("carapp.core.ai_voice.importlib.import_module", return_value=self.sd)
        self.import_module = self.loader.start()
        self.addCleanup(self.loader.stop)

    def recorder(self, **values):
        value = MicrophoneRecorder(**values)
        self.addCleanup(value.cancel)
        return value

    def test_dependency_is_lazy_and_input_device_inventory_is_filtered(self):
        self.recorder()
        self.import_module.assert_not_called()
        self.assertEqual(MicrophoneRecorder.devices(), [dict(index=1, name="default microphone")])
        self.assertEqual(self.sd.streams, [])

    def test_default_capture_returns_mono_pcm16_wav_and_releases_device(self):
        recorder = self.recorder()
        recorder.start()
        stream = self.sd.streams[0]
        self.assertEqual((stream.values["samplerate"], stream.values["channels"], stream.values["dtype"]),
                         (16000, 1, "int16"))
        self.assertTrue(recorder.recording)
        raw = b"\x01\x02" * 320
        stream.emit(raw)
        wav = recorder.stop()
        with wave.open(io.BytesIO(wav)) as reader:
            self.assertEqual((reader.getnchannels(), reader.getsampwidth(), reader.getframerate()), (1, 2, 16000))
            self.assertEqual(reader.getnframes(), 320)
            self.assertEqual(reader.readframes(320), raw)
        self.assertFalse(recorder.recording)
        self.assertIsNone(recorder.error)
        self.assertTrue(stream.closed)
        self.assertEqual((stream.stop_calls, stream.close_calls), (1, 1))

    def test_maximum_duration_trims_audio_and_auto_finishes_before_stop(self):
        recorder = self.recorder(max_seconds=.1)
        recorder.start()
        stream = self.sd.streams[0]
        stream.emit(b"\x01\x02" * 2000)
        self.assertFalse(recorder.recording)
        wait_for(lambda: stream.closed)
        with wave.open(io.BytesIO(recorder.stop())) as reader:
            self.assertEqual(reader.getnframes(), 1600)
        self.assertEqual(stream.close_calls, 1)

    def test_wall_clock_limit_also_ends_a_stalled_input(self):
        recorder = self.recorder(max_seconds=.04)
        recorder.start()
        stream = self.sd.streams[0]
        wait_for(lambda: not recorder.recording and stream.closed)
        with self.assertRaisesRegex(RuntimeError, "未采集到音频"):
            recorder.stop()

    def test_second_start_does_not_replace_an_active_capture(self):
        recorder = self.recorder()
        recorder.start()
        with self.assertRaisesRegex(RuntimeError, "已有录音"):
            recorder.start()
        self.assertEqual(len(self.sd.streams), 1)
        self.sd.streams[0].emit(b"\x00\x01" * 12)
        self.assertTrue(recorder.stop().startswith(b"RIFF"))

    def test_stop_without_recording_and_cancel_are_well_defined(self):
        recorder = self.recorder()
        with self.assertRaisesRegex(RuntimeError, "先开始录音"):
            recorder.stop()
        recorder.cancel()
        recorder.cancel()
        self.assertFalse(recorder.recording)
        self.assertEqual(self.sd.streams, [])

    def test_cancel_discards_audio_and_old_callbacks_cannot_enter_new_capture(self):
        recorder = self.recorder()
        recorder.start()
        old = self.sd.streams[0]
        old.emit(b"\x01\x02" * 20)
        recorder.cancel()
        with self.assertRaisesRegex(RuntimeError, "先开始录音"):
            recorder.stop()
        recorder.start()
        new = self.sd.streams[1]
        old.emit(b"\xff\xff" * 80)
        new.emit(b"\x03\x04" * 7)
        with wave.open(io.BytesIO(recorder.stop())) as reader:
            self.assertEqual(reader.readframes(100), b"\x03\x04" * 7)
        self.assertTrue(old.closed)

    def test_overflow_rejects_all_audio_instead_of_transcribing_partial_command(self):
        recorder = self.recorder()
        recorder.start()
        stream = self.sd.streams[0]
        stream.emit(b"\x01\x02" * 20)
        stream.emit(b"\x03\x04" * 20, status=True)
        self.assertFalse(recorder.recording)
        self.assertIn("丢帧", recorder.error)
        with self.assertRaisesRegex(RuntimeError, "丢帧"):
            recorder.stop()
        self.assertTrue(stream.closed)

    def test_incomplete_input_buffer_is_rejected(self):
        recorder = self.recorder()
        recorder.start()
        self.sd.streams[0].emit(b"\x01\x02", frames=10)
        with self.assertRaisesRegex(RuntimeError, "不完整音频"):
            recorder.stop()

    def test_open_failure_is_actionable_and_retry_clears_error(self):
        recorder = self.recorder()
        self.sd.open_error = OSError("no input device")
        with self.assertRaisesRegex(RuntimeError, "检查默认录音设备"):
            recorder.start()
        self.assertFalse(recorder.recording)
        self.assertIn("no input device", recorder.error)
        self.sd.open_error = None
        recorder.start()
        self.sd.streams[0].emit(b"\0\0" * 12)
        self.assertTrue(recorder.stop())
        self.assertIsNone(recorder.error)

    def test_missing_optional_dependency_does_not_open_or_persist_anything(self):
        self.import_module.side_effect = ImportError("missing sounddevice")
        recorder = self.recorder()
        with self.assertRaisesRegex(RuntimeError, "requirements.txt"):
            recorder.start()
        self.assertFalse(recorder.recording)
        self.assertEqual(self.sd.streams, [])

    def test_unexpected_backend_finish_is_an_error_not_partial_voice(self):
        recorder = self.recorder()
        recorder.start()
        stream = self.sd.streams[0]
        stream.emit(b"\0\1" * 12)
        stream.active = False
        stream.finished()
        with self.assertRaisesRegex(RuntimeError, "提前停止"):
            recorder.stop()

    def test_cancel_during_device_open_discards_late_open_and_closes_once(self):
        recorder = self.recorder()
        self.sd.open_entered, self.sd.open_release = threading.Event(), threading.Event()
        errors = []
        def start():
            try:
                recorder.start()
            except RuntimeError as exc:
                errors.append(str(exc))
        worker = threading.Thread(target=start)
        worker.start()
        self.assertTrue(self.sd.open_entered.wait(.5))
        release = threading.Timer(.03, self.sd.open_release.set)
        release.start()
        recorder.cancel()
        worker.join(1.)
        release.join(1.)
        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, ["录音已取消"])
        self.assertTrue(self.sd.streams[0].closed)
        self.assertEqual(self.sd.streams[0].close_calls, 1)
        with self.assertRaises(RuntimeError):
            recorder.stop()

    def test_cancel_overtakes_stopping_without_returning_cancelled_audio(self):
        recorder = self.recorder()
        recorder.start()
        stream = self.sd.streams[0]
        stream.emit(b"\0\1" * 12)
        stream.stop_entered, stream.stop_release = threading.Event(), threading.Event()
        values, errors = [], []
        def stop():
            try:
                values.append(recorder.stop())
            except RuntimeError as exc:
                errors.append(str(exc))
        worker = threading.Thread(target=stop)
        worker.start()
        self.assertTrue(stream.stop_entered.wait(.5))
        with self.assertRaisesRegex(RuntimeError, "已有录音"):
            recorder.start()
        release = threading.Timer(.03, stream.stop_release.set)
        release.start()
        recorder.cancel()
        worker.join(1.)
        release.join(1.)
        self.assertEqual(values, [])
        self.assertEqual(errors, ["录音已取消"])
        self.assertEqual(stream.close_calls, 1)

    def test_close_failure_is_reported_before_audio_can_be_uploaded(self):
        recorder = self.recorder()
        recorder.start()
        stream = self.sd.streams[0]
        stream.emit(b"\0\1" * 12)
        stream.close_error = OSError("device close failed")
        with self.assertRaisesRegex(RuntimeError, "关闭麦克风失败"):
            recorder.stop()
        self.assertFalse(recorder.recording)

    def test_invalid_limits_are_rejected_without_importing_sounddevice(self):
        for value in (0., -1., 15.1, float("inf"), float("nan"), True):
            with self.subTest(seconds=value), self.assertRaises(ValueError):
                self.recorder(max_seconds=value)
        for rate in (True, 0, 7999, 48001, 16000.5):
            with self.subTest(rate=rate), self.assertRaises(ValueError):
                self.recorder(sample_rate=rate)
        self.import_module.assert_not_called()


class SystemDictationTests(unittest.TestCase):
    def test_non_windows_has_actionable_fallback(self):
        with patch("carapp.core.ai_voice.os.name", "posix"):
            with self.assertRaisesRegex(RuntimeError, "仅适用于 Windows"):
                start_system_dictation()

    def test_shortcut_sends_balanced_windows_h_keys_without_real_keypress(self):
        send = Mock(return_value=4)
        with patch("carapp.core.ai_voice.os.name", "nt"), \
             patch("ctypes.WinDLL", return_value=types.SimpleNamespace(SendInput=send), create=True):
            self.assertIsNone(start_system_dictation())
        count, events, size = send.call_args.args
        self.assertEqual(count, 4)
        self.assertEqual([event.ki.wVk for event in events], [0x5B, 0x48, 0x48, 0x5B])
        self.assertEqual([event.ki.dwFlags for event in events], [0, 0, 2, 2])
        import ctypes
        self.assertEqual(size, ctypes.sizeof(events._type_))

    def test_partial_batch_releases_modifier_and_reports_manual_shortcut(self):
        send = Mock(side_effect=(2, 2))
        with patch("carapp.core.ai_voice.os.name", "nt"), \
             patch("ctypes.WinDLL", return_value=types.SimpleNamespace(SendInput=send), create=True):
            with self.assertRaisesRegex(RuntimeError, "手动按 Win\\+H"):
                start_system_dictation()
        _, events, _ = send.call_args.args
        self.assertEqual([event.ki.wVk for event in events], [0x48, 0x5B])
        self.assertTrue(all(event.ki.dwFlags == 2 for event in events))

    def test_missing_windows_api_does_not_request_microphone(self):
        with patch("carapp.core.ai_voice.os.name", "nt"), \
             patch("ctypes.WinDLL", side_effect=OSError("user32 unavailable"), create=True):
            with self.assertRaisesRegex(RuntimeError, "麦克风权限"):
                start_system_dictation()


if __name__ == "__main__":
    unittest.main()
