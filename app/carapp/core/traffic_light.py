"""Latest-frame traffic-light interlock, independent of Tk and page rendering."""
import math
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
    def __init__(self, stop, changed=None, detector=None, *, clock=time.monotonic, scene=None):
        if detector is None:
            from carapp.core.traffic_light_detector import TrafficLightDetector
            detector = TrafficLightDetector()
        self.detector, self.stop = detector, stop
        self.changed = changed or (lambda value: None)
        self.clock, self.policy = clock, TrafficLightPolicy(clock)
        self.scene = scene if scene is not None else TrafficScenePolicy()
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


class JunctionViews:
    """Sequences the forward, dip and look-up views for one junction.

    The business rule is red first, crosswalk second: as soon as the forward
    view reports a red light the camera dips to confirm the crosswalk stop line,
    and once the crosswalk is confirmed the camera lifts so the lamp can be
    watched while the car holds.  The reverse order still works when the
    crosswalk is confirmed before any lamp is seen.  Actions are returned
    instead of sent so the sequence stays testable without hardware.
    """
    FORWARD = "forward"
    DIP = "dip"
    LIFT = "lift"
    RESTORE = "restore"
    MIN_PITCH = 1150
    MAX_PITCH = 2250

    def __init__(self, pitch=1720, dip=1450, restore=1500, delay=1.4, hold=8., retry=.8, dip_hold=1.2,
                 settle=0.):
        self.pitch = self._target("红灯确认仰角", pitch)
        self.dip = self._target("斑马线确认俯角", dip)
        self.restore = self._target("红灯恢复俯仰", restore)
        self.delay = self._seconds("红灯确认抬起延时", delay)
        self.hold = self._seconds("红灯确认保持时间", hold)
        self.retry = self._seconds("红灯确认重试间隔", retry)
        self.dip_hold = self._seconds("斑马线确认俯视时间", dip_hold)
        self.settle = self._seconds("红灯确认视角切换稳定时间", settle)
        self.reset()

    def _target(self, name, value):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(name + "无效")
        number = int(value)
        if not self.MIN_PITCH <= number <= self.MAX_PITCH:
            raise ValueError(name + "需在 %d–%d" % (self.MIN_PITCH, self.MAX_PITCH))
        return number

    @staticmethod
    def _seconds(name, value):
        if isinstance(value, bool):
            raise ValueError(name + "无效")
        try:
            number = float(value)
        except (TypeError, ValueError, OverflowError):
            raise ValueError(name + "无效")
        if not math.isfinite(number) or number < 0.:
            raise ValueError(name + "无效")
        return number

    def reset(self):
        self.view = self.FORWARD
        self.view_at = None
        self.red_at = None
        self.retry_at = 0.
        self.settle_until = None

    @property
    def lifted(self):
        return self.view == self.LIFT

    def _go(self, view, now):
        self.view, self.view_at = view, now
        # Frames captured while the gimbal is still moving are unreliable; the
        # caller keeps updating the servo and the sequence ignores detections
        # until the view has settled.
        self.settle_until = (now + self.settle) if self.settle > 0. else None

    def step(self, red, active, now):
        """Return (action, notice); action is None, DIP, LIFT or RESTORE.

        red: the scene saw a red light (independent of the crosswalk).
        active: the crosswalk stop line is confirmed ahead.
        """
        if self.settle_until is not None and now < self.settle_until:
            return None, ""
        if self.view == self.DIP:
            if active:
                self._go(self.LIFT, now)
                return self.LIFT, "斑马线确认，抬头确认红灯 %d" % self.pitch
            if now - self.view_at >= self.dip_hold:
                self._go(self.FORWARD, now)
                self.retry_at = now + self.retry
                return self.RESTORE, "未确认斑马线，云台复位 %d" % self.restore
            return None, ""
        if self.view == self.LIFT:
            if red:
                if self.red_at is None:
                    self.red_at = now
                    return None, "红灯确认，斑马线前停车"
                return None, ""
            if not active:
                # The crosswalk left the view: the car has passed the junction.
                if now - self.view_at >= self.delay:
                    self._go(self.FORWARD, now)
                    self.retry_at = now + self.retry
                    self.red_at = None
                    return self.RESTORE, "离开路口，云台复位 %d" % self.restore
                return None, ""
            if self.red_at is None and now - self.view_at >= self.hold:
                # Staring at a lamp that never reads red for too long is not
                # useful: fall back to the forward view and retry later.
                self._go(self.FORWARD, now)
                self.retry_at = now + self.retry
                return self.RESTORE, "未确认红灯，云台复位 %d" % self.restore
            return None, ""
        if now < self.retry_at:
            return None, ""
        if red and not active:
            self.red_at = None
            self._go(self.DIP, now)
            return self.DIP, "前方红灯，俯视确认斑马线 %d" % self.dip
        if active and not red:
            self._go(self.LIFT, now)
            return self.LIFT, "路口确认，抬头确认红绿灯 %d" % self.pitch
        return None, ""


LookUpConfirm = JunctionViews
