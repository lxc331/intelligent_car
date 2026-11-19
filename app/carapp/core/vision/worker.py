"""Background YOLO inference over the newest camera frame.

``submit()`` is called from the UI thread and only stores a reference, so the
Tk loop never waits for a model. The worker keeps a single pending frame: if
frames arrive faster than inference, older ones are dropped instead of queued,
which is what a live preview wants.

The worker knows nothing about pages or widgets; callers decide what to draw.
"""
import threading
import time


class VisionWorker:
    """One background thread, one model, always the newest frame."""

    def __init__(self, detector, interval=.25, clock=time.monotonic):
        self.detector = detector
        self.interval = max(0., float(interval))
        self.clock = clock
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._pending = None
        self._result = None
        self._thread = None
        self._frames = 0
        self._errors = 0
        self._last_ms = None

    # ---------------- 生命周期 ----------------
    def start(self):
        with self._lock:
            if self._thread and self._thread.is_alive():
                return
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="vision-worker", daemon=True)
            self._thread.start()

    def stop(self, timeout=.6):
        self._stop.set()
        self._wake.set()
        thread = self._thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout)
        with self._lock:
            self._thread = None
            self._pending = None
            self._result = None

    @property
    def running(self):
        thread = self._thread
        return bool(thread and thread.is_alive())

    # ---------------- 数据 ----------------
    def submit(self, payload, mode="jpeg", received_at=None):
        """Store the newest frame; called from the UI thread, must stay cheap."""
        if self._stop.is_set():
            return False
        with self._lock:
            self._pending = (payload, mode, self.clock() if received_at is None else received_at)
        self._wake.set()
        return True

    def reset(self):
        """Drop the pending frame and the last result (disconnect / vehicle change)."""
        with self._lock:
            self._pending = None
            self._result = None
            self._last_ms = None

    def snapshot(self):
        with self._lock:
            return dict(self._result) if self._result else None

    def stats(self):
        with self._lock:
            return dict(frames=self._frames, errors=self._errors, last_ms=self._last_ms,
                        running=self.running)

    # ---------------- 线程 ----------------
    def _run(self):
        while not self._stop.is_set():
            self._wake.wait(.2)
            self._wake.clear()
            if self._stop.is_set():
                break
            with self._lock:
                item, self._pending = self._pending, None
            if not item:
                continue
            payload, mode, received_at = item
            started = self.clock()
            try:
                detection = self.detector.detect(payload, mode)
                # ``objects`` 保持给目标检测用；车道线检测读 ``detection.lanes``。
                objects = getattr(detection, "objects", ())
                size, error = getattr(detection, "image_size", ()), ""
            except Exception as exc:  # a bad frame must not kill the preview
                detection, objects, size, error = None, (), (), str(exc)
            elapsed = self.clock() - started
            with self._lock:
                self._frames += 1
                self._last_ms = elapsed * 1000.
                if error:
                    self._errors += 1
                self._result = dict(detection=detection, objects=objects, image_size=size,
                                    received_at=received_at, processed_at=self.clock(),
                                    error=error, ms=elapsed * 1000.)
            wait = self.interval - (self.clock() - started)
            if wait > 0:
                self._stop.wait(wait)
