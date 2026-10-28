"""Latest-frame traffic-light interlock, independent of Tk and page rendering."""
import threading
import time
from carapp.core.traffic_scene import TrafficScenePolicy, limit_twist


class TrafficLightPolicy:
    FRAME_TIMEOUT = 1.2
    CLEAR_SECONDS = .8
    CLEAR_FRAMES = 3

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.enabled = False
        self.reset()

    def reset(self):
        self.last_at = None
        self.red = False
        self.blocked = True
        self.error = ""
        self.clear_since = None
        self.clear_frames = 0

    def observe(self, red, received_at, error=""):
        now = self.clock()
        if (received_at > now or now - received_at > self.FRAME_TIMEOUT
                or (self.last_at is not None and received_at <= self.last_at)):
            return
        if self.last_at is None or received_at - self.last_at > self.FRAME_TIMEOUT:
            self.reset()
        self.last_at, self.error = received_at, str(error)
        if error or red:
            self.red, self.blocked = bool(red), True
            self.clear_since, self.clear_frames = None, 0
        else:
            self.red = False
            if self.clear_since is None:
                self.clear_since = received_at
            self.clear_frames += 1
            if self.clear_frames >= self.CLEAR_FRAMES and received_at - self.clear_since >= self.CLEAR_SECONDS:
                self.blocked = False

    def snapshot(self):
        age = None if self.last_at is None else max(0., self.clock() - self.last_at)
        stale = age is None or age > self.FRAME_TIMEOUT
        if stale:
            self.blocked = True
            self.clear_since, self.clear_frames = None, 0
        blocked = self.enabled and (self.blocked or stale or bool(self.error))
        reason = ("未开启" if not self.enabled else
                  "等待画面，停车" if age is None else
                  "画面超时，停车" if stale else
                  "识别异常，停车" if self.error else
                  "红灯停车" if self.red else
                  "确认红灯已消失" if blocked else "未见红灯，可行驶")
        return dict(enabled=self.enabled, blocked=blocked, reason=reason,
                    red=self.red, frame_age=age, error=self.error)


class TrafficLightController:
    """One worker decodes at most 6 fps; old sessions/results cannot release hold.

    guard_motion holds the same lock through the actual velocity send, so a
    red observation and a command cannot overtake each other at publication.
    All callbacks are pure data/communication callbacks, never Tk operations.
    """
    def __init__(self, stop, changed=None, detector=None, *, clock=time.monotonic):
        if detector is None:
            from carapp.core.traffic_light_detector import TrafficLightDetector
            detector = TrafficLightDetector()
        self.detector, self.stop = detector, stop
        self.changed = changed or (lambda value: None)
        self.clock, self.policy = clock, TrafficLightPolicy(clock)
        self.scene = TrafficScenePolicy()
        self.lock = threading.RLock()
        self.wake, self.halt = threading.Event(), threading.Event()
        self.generation = 0
        self.camera_generation = 0
        self.pending = None
        self.last_submit = float("-inf")
        self.last_stop = float("-inf")
        self.last_notice = None
        self.thread = threading.Thread(target=self._run, name="traffic-light", daemon=True)
        self.watchdog = threading.Thread(target=self._watch, name="traffic-light-watchdog", daemon=True)
        self.thread.start()
        self.watchdog.start()

    def set_enabled(self, enabled):
        if enabled:
            # Optional until this function is enabled; manual PNG control still works.
            from PIL import Image  # noqa: F401
            check = getattr(self.detector, "ensure_available", None)
            if check:
                check()
        with self.lock:
            self.policy.enabled = bool(enabled)
            self._reset_locked()
        self.wake.set()

    def _reset_locked(self):
        self.generation += 1
        self.pending = None
        self.last_submit = float("-inf")
        self.policy.reset()
        self.scene.reset()

    def reset(self):
        with self.lock:
            self.camera_generation += 1
            self._reset_locked()
        self.wake.set()

    def frame_handler(self):
        """Bind to one camera lifetime before it is started."""
        with self.lock:
            generation = self.camera_generation
        return lambda received_at, payload, mode: self.submit(received_at, payload, mode, generation)

    def submit(self, received_at, payload, mode, generation=None):
        with self.lock:
            if (self.halt.is_set() or not self.policy.enabled
                    or (generation is not None and generation != self.camera_generation)
                    or received_at - self.last_submit < 1. / 6.):
                return
            self.last_submit = received_at
            self.pending = (self.generation, received_at, payload, mode)
        self.wake.set()

    def snapshot(self):
        with self.lock:
            return self._snapshot_locked()

    def _snapshot_locked(self):
        state = self.policy.snapshot()
        if (not state["enabled"] or state["error"] or state["frame_age"] is None
                or state["frame_age"] > self.policy.FRAME_TIMEOUT):
            self.scene.reset()
        state.update(self.scene.snapshot())
        if not state["enabled"]:
            state["speed_limit"] = None
        elif not state["blocked"] and (state["speed_limit"] is not None or state["scene_red_visible"]):
            state["reason"] = state["scene"]
        return state

    def guard_motion(self, publish):
        with self.lock:
            return publish(self._snapshot_locked()["blocked"])

    def guard_twist(self, vx, vy, wz, publish):
        """Apply scene limits only to an existing command, never start motion."""
        with self.lock:
            state = self._snapshot_locked()
            values = (0., 0., 0.) if state["blocked"] else limit_twist(vx, vy, wz, state["speed_limit"])
            return publish(*values)

    def _observe_locked(self, result, received_at, error):
        if error:
            self.scene.reset()
            self.policy.observe(False, self.clock(), error)
            return
        now, previous = self.clock(), self.policy.last_at
        if (received_at > now or now - received_at > self.policy.FRAME_TIMEOUT
                or previous is not None and received_at <= previous):
            return
        if previous is not None and received_at - previous > self.policy.FRAME_TIMEOUT:
            self.scene.reset()
        try:
            if getattr(result, "image_size", ()):
                red = self.scene.observe(result.objects, result.image_size, received_at,
                                         paused=self.policy.snapshot()["blocked"])
            else:
                # Keep the original result protocol for legacy data callbacks.
                red = result.red
            self.policy.observe(red, received_at)
        except Exception as exc:
            self.scene.reset()
            self.policy.observe(False, self.clock(), str(exc))

    def _maintain(self):
        with self.lock:
            state = self._snapshot_locked()
            if state["blocked"] and self.clock() - self.last_stop >= .2:
                self.last_stop = self.clock()
                self.stop()
            notice = state["enabled"], state["blocked"], state["reason"], state["error"], state["speed_limit"]
            if notice != self.last_notice:
                self.last_notice = notice
                self.changed(state)

    def _run(self):
        while not self.halt.is_set():
            self.wake.wait(.1)
            self.wake.clear()
            with self.lock:
                item, self.pending = self.pending, None
            if item:
                generation, received_at, payload, mode = item
                try:
                    result = self.detector.detect(payload, mode)
                    error = ""
                except Exception as exc:
                    result, error = None, str(exc)
                with self.lock:
                    if generation == self.generation and self.policy.enabled:
                        # Loading failures may arrive after the image deadline.
                        # Report the current fault while still rejecting old
                        # successful detections as a source of clearance.
                        self._observe_locked(result, received_at, error)
    def _watch(self):
        # Independent of decoding: a slow/invalid frame cannot postpone the
        # timeout stop when no feature happens to publish a new command.
        while not self.halt.wait(.1):
            try:
                self._maintain()
            except Exception:
                # A transient send failure cannot kill the interlock watchdog.
                pass

    def close(self):
        self.halt.set()
        with self.lock:
            self.policy.enabled = False
            self._reset_locked()
        self.wake.set()
        self.thread.join(timeout=1.)
        self.watchdog.join(timeout=1.)
