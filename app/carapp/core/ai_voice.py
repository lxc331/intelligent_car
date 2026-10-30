"""Bounded microphone capture for AI commands; WAV exists only in memory.

The page calls start/stop/cancel from its IO workers. PortAudio callbacks never
touch Tk or perform cloud requests. sounddevice is optional until recording.
"""
import importlib
import io
import math
import os
import threading
import wave


def start_system_dictation():
    """Open Windows Win+H dictation only on an explicit UI-button action.

    The page must focus its text input first. Windows inserts recognized text
    there; the user reviews and sends it, so this function never executes a car
    action or returns a transcription. A supported dictation language, enabled
    Windows voice input and microphone permission are required. The OS may use
    its own online speech service; no AI-provider ASR key is used here.
    """
    if os.name != "nt":
        raise RuntimeError("系统语音输入仅适用于 Windows；可直接输入文字或使用云端录音识别。")
    import ctypes
    from ctypes import wintypes

    class KeyboardInput(ctypes.Structure):
        _fields_ = (("wVk", wintypes.WORD), ("wScan", wintypes.WORD),
                    ("dwFlags", wintypes.DWORD), ("time", wintypes.DWORD),
                    ("dwExtraInfo", ctypes.c_size_t))

    class MouseInput(ctypes.Structure):
        _fields_ = (("dx", wintypes.LONG), ("dy", wintypes.LONG),
                    ("mouseData", wintypes.DWORD), ("dwFlags", wintypes.DWORD),
                    ("time", wintypes.DWORD), ("dwExtraInfo", ctypes.c_size_t))

    class HardwareInput(ctypes.Structure):
        _fields_ = (("uMsg", wintypes.DWORD), ("wParamL", wintypes.WORD), ("wParamH", wintypes.WORD))

    class InputData(ctypes.Union):
        _fields_ = (("ki", KeyboardInput), ("mi", MouseInput), ("hi", HardwareInput))

    class Input(ctypes.Structure):
        _anonymous_ = ("data",)
        _fields_ = (("type", wintypes.DWORD), ("data", InputData))

    try:
        user32 = ctypes.WinDLL("user32", use_last_error=True)
        send = user32.SendInput
        send.argtypes = (wintypes.UINT, ctypes.POINTER(Input), ctypes.c_int)
        send.restype = wintypes.UINT
        values = (Input * 4)()
        for event, (key, flags) in zip(values, ((0x5B, 0), (0x48, 0), (0x48, 2), (0x5B, 2))):
            event.type = 1
            event.ki = KeyboardInput(key, 0, flags, 0, 0)
        if send(4, values, ctypes.sizeof(Input)) != 4:
            # Never leave the Windows key held if the input batch was partial.
            releases = (Input * 2)()
            for event, key in zip(releases, (0x48, 0x5B)):
                event.type = 1
                event.ki = KeyboardInput(key, 0, 2, 0, 0)
            send(2, releases, ctypes.sizeof(Input))
            raise OSError("Windows rejected the dictation shortcut")
    except (OSError, AttributeError) as exc:
        raise RuntimeError("无法启动 Windows 语音输入；请将光标放在文本框后手动按 Win+H，并检查系统语音输入和麦克风权限。") from exc


class MicrophoneRecorder:
    CHANNELS = 1
    SAMPLE_WIDTH = 2

    def __init__(self, max_seconds=15., sample_rate=16000, device=None):
        if isinstance(max_seconds, bool):
            raise ValueError("录音时长须为 0–15 秒")
        max_seconds = float(max_seconds)
        if not math.isfinite(max_seconds) or not 0. < max_seconds <= 15.:
            raise ValueError("录音时长须为 0–15 秒")
        if (isinstance(sample_rate, bool) or not isinstance(sample_rate, int)
                or not 8000 <= sample_rate <= 48000):
            raise ValueError("录音采样率须为 8000–48000 Hz 的整数")
        self.max_seconds, self.sample_rate, self.device = max_seconds, sample_rate, device
        self.max_bytes = max(1, round(max_seconds * sample_rate)) * self.SAMPLE_WIDTH
        self._lock = threading.RLock()
        self._io_lock = threading.RLock()
        self._generation = 0
        self._buffer = bytearray()
        self._stream = self._timer = None
        self._recording = self._opening = self._stopping = self._pending = False
        self._closing = set()
        self._error = None

    @property
    def recording(self):
        with self._lock:
            return self._recording

    @property
    def error(self):
        with self._lock:
            return self._error

    @staticmethod
    def _sounddevice():
        try:
            return importlib.import_module("sounddevice")
        except (ImportError, OSError) as exc:
            raise RuntimeError("语音录音需要 sounddevice 和可用麦克风；请在 App 的 Python 环境安装 app/requirements.txt。") from exc

    @classmethod
    def devices(cls):
        """Optional input-device inventory; call from an IO worker."""
        try:
            return [dict(index=index, name=str(device["name"]))
                    for index, device in enumerate(cls._sounddevice().query_devices())
                    if device.get("max_input_channels", 0) > 0]
        except RuntimeError:
            raise
        except Exception as exc:
            raise RuntimeError("无法读取电脑麦克风列表：%s" % exc) from exc

    @staticmethod
    def _dispose(stream):
        if stream is None:
            return None
        error = None
        try:
            stream.stop()
        except Exception as exc:
            error = exc
        try:
            stream.close()
        except Exception as exc:
            error = error or exc
        return error

    def _cancel_timer_locked(self):
        timer, self._timer = self._timer, None
        if timer:
            timer.cancel()

    def _finish_cleanup(self, generation):
        with self._io_lock:
            with self._lock:
                if generation != self._generation:
                    return
                stream, self._stream = self._stream, None
                self._cancel_timer_locked()
            error = self._dispose(stream)
            if error:
                with self._lock:
                    if generation == self._generation:
                        self._error = self._error or "关闭麦克风失败：%s" % error

    def _finished(self, generation, *, timed=False):
        with self._lock:
            if generation != self._generation or not self._pending:
                return
            if self._recording and not timed:
                self._error = self._error or "麦克风提前停止，请检查输入设备后重新录音。"
            self._recording = False
            self._cancel_timer_locked()
        threading.Thread(target=self._finish_cleanup, args=(generation,),
                         name="ai-microphone-close", daemon=True).start()

    def start(self):
        """Open the default input device; raises with no partial audio on failure."""
        sd = self._sounddevice()
        with self._lock:
            if self._opening or self._closing or self._pending or self._stream is not None:
                raise RuntimeError("已有录音正在处理，请先停止识别或取消。")
            self._generation += 1
            generation = self._generation
            self._opening = self._pending = True
            self._buffer.clear()
            self._error = None

        def callback(data, frames, _timing, status):
            with self._lock:
                if generation != self._generation or not self._recording:
                    raise sd.CallbackAbort
                if status:
                    self._error = "麦克风录音发生丢帧或设备异常，请重新录音。"
                    self._recording = False
                    self._buffer.clear()
                    raise sd.CallbackAbort
                raw = bytes(data)
                expected = frames * self.SAMPLE_WIDTH
                if expected < 0 or len(raw) < expected:
                    self._error = "麦克风返回了不完整音频，请重新录音。"
                    self._recording = False
                    self._buffer.clear()
                    raise sd.CallbackAbort
                remaining = self.max_bytes - len(self._buffer)
                self._buffer.extend(raw[:min(remaining, expected)])
                if len(self._buffer) >= self.max_bytes:
                    self._recording = False
                    raise sd.CallbackStop

        stream = None
        with self._io_lock:
            try:
                with self._lock:
                    if generation != self._generation:
                        raise RuntimeError("录音已取消")
                stream = sd.RawInputStream(samplerate=self.sample_rate, channels=self.CHANNELS,
                    dtype="int16", device=self.device, callback=callback,
                    finished_callback=lambda: self._finished(generation))
                with self._lock:
                    if generation != self._generation:
                        raise RuntimeError("录音已取消")
                    self._stream = stream
                    self._opening = False
                    self._recording = True
                stream.start()
                with self._lock:
                    if generation != self._generation:
                        raise RuntimeError("录音已取消")
                    if self._recording:
                        timer = threading.Timer(self.max_seconds, self._finished,
                                                args=(generation,), kwargs=dict(timed=True))
                        timer.daemon = True
                        self._timer = timer
                        timer.start()
            except Exception as exc:
                with self._lock:
                    if self._stream is stream:
                        self._stream = None
                self._dispose(stream)
                with self._lock:
                    if generation == self._generation:
                        self._stream = None
                        self._recording = self._opening = self._pending = False
                        self._buffer.clear()
                        self._error = "无法打开麦克风：%s。请检查默认录音设备、麦克风权限和采样率。" % exc
                        self._cancel_timer_locked()
                        error = self._error
                    else:
                        error = "录音已取消"
                raise RuntimeError(error) from exc

    def stop(self):
        """Release the device and return bounded mono PCM16 WAV bytes.

        Also retrieves an automatically completed recording. Failed/overflowed
        captures raise instead of sending potentially incomplete voice commands.
        """
        with self._lock:
            if not self._pending:
                raise RuntimeError("没有可识别的录音，请先开始录音。")
            self._generation += 1
            generation = self._generation
            self._recording = self._opening = self._pending = False
            self._stopping = True
            self._closing.add(generation)
            self._cancel_timer_locked()
            raw = bytes(self._buffer)
            self._buffer.clear()
            capture_error = self._error
        with self._io_lock:
            with self._lock:
                stream, self._stream = self._stream, None
            close_error = self._dispose(stream)
            with self._lock:
                self._closing.discard(generation)
                self._stopping = bool(self._closing)
                if generation != self._generation:
                    raise RuntimeError("录音已取消")
                if close_error:
                    self._error = capture_error or "关闭麦克风失败：%s" % close_error
                capture_error = self._error
        if capture_error:
            raise RuntimeError(capture_error)
        if not raw:
            raise RuntimeError("未采集到音频，请检查电脑麦克风后重试。")
        audio = io.BytesIO()
        with wave.open(audio, "wb") as writer:
            writer.setnchannels(self.CHANNELS)
            writer.setsampwidth(self.SAMPLE_WIDTH)
            writer.setframerate(self.sample_rate)
            writer.writeframes(raw)
        return audio.getvalue()

    def cancel(self):
        """Invalidate opening/callback/stop work and discard audio without ASR."""
        with self._lock:
            self._generation += 1
            generation = self._generation
            self._recording = self._opening = self._pending = False
            self._stopping = True
            self._closing.add(generation)
            self._cancel_timer_locked()
            self._buffer.clear()
            self._error = None
        with self._io_lock:
            with self._lock:
                stream, self._stream = self._stream, None
            error = self._dispose(stream)
            with self._lock:
                self._closing.discard(generation)
                self._stopping = bool(self._closing)
                if error and generation == self._generation:
                    self._error = "关闭麦克风失败：%s" % error
