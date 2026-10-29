"""Hold-to-run teleoperation; one worker serializes all held inputs."""
import math
import threading
import time


BASE_KEYS = frozenset("wsadqe")
CAMERA_KEYS = frozenset("ikjl")
ACTION_KEYS = {"t": ("arm", "arm_up"), "g": ("arm", "arm_down"),
               "f": ("gripper", "open"), "h": ("gripper", "close")}
CONTROL_KEYS = BASE_KEYS | CAMERA_KEYS | ACTION_KEYS.keys()


class TeleopController:
    def __init__(self, ctx, *, clock=time.monotonic, autostart=True):
        self.ctx = ctx
        self.clock = clock
        self.lock = threading.RLock()
        self.pressed = set()
        self.blocked = set()
        self.last_twist = (0., 0., 0.)
        self.next_camera = 0.
        self.speed = .06
        self.turn = .4
        self.hz = min(30., max(5., float(ctx.cfg.get("base", {}).get("publish_hz", 10))))
        self.closed = threading.Event()
        self.thread = None
        if autostart:
            self.thread = threading.Thread(target=self._run, name="remote-teleop", daemon=True)
            self.thread.start()

    def set_speed(self, linear, angular):
        with self.lock:
            self.speed = min(.15, max(0., float(linear)))
            self.turn = min(.4, max(0., float(angular)))

    def press(self, key):
        key = key.lower()
        if key not in CONTROL_KEYS or self.closed.is_set():
            return False
        with self.lock:
            if key in self.pressed or key in self.blocked:
                return True  # OS key-repeat must not replay a servo action or undo a stop.
            if not self.ctx.state.get("ws_connected"):
                self.ctx.log("遥控未执行：ROS 未连接")
                return False
            self.pressed.add(key)
            if key in ACTION_KEYS:
                joint, action = ACTION_KEYS[key]
                if not self.ctx.publish_servo({joint: self.ctx.cfg["servo"]["actions"][action]}, .8):
                    self._stop_locked()
            else:
                if key in CAMERA_KEYS:
                    self.next_camera = 0.
                self._tick_locked(self.clock())
        return True

    def release(self, key):
        with self.lock:
            self.pressed.discard(key)
            self.blocked.discard(key)
            self._tick_locked(self.clock())

    def tick(self):
        with self.lock:
            if not self.closed.is_set():
                self._tick_locked(self.clock())

    def _tick_locked(self, now):
        if self.pressed and not self.ctx.state.get("ws_connected"):
            self._stop_locked()
            return
        keys = self.pressed
        sx, sy = int("w" in keys) - int("s" in keys), int("q" in keys) - int("e" in keys)
        scale = max(1., math.hypot(sx, sy))
        twist = (sx * self.speed / scale, sy * self.speed / scale,
                 (int("a" in keys) - int("d" in keys)) * self.turn)
        if any(twist):
            if not self.ctx.publish_twist(twist[0], twist[2], quiet=True, vy=twist[1]):
                self._stop_locked()
                return
        elif any(self.last_twist):
            self.ctx.stop_base()
        self.last_twist = twist

        if keys.intersection(CAMERA_KEYS) and now >= self.next_camera:
            current = self.ctx.state.get("servo_targets") or {}
            limits = self.ctx.cfg["servo"]["limits"]
            actions = self.ctx.cfg["servo"]["actions"]
            signs = self.ctx.cfg["servo"].get("signs") or {}
            targets = {}
            for joint, delta, fallback in (
                ("pitch", int("i" in keys) - int("k" in keys), actions["look_down"]),
                ("yaw", int("j" in keys) - int("l" in keys), 1500),
            ):
                if not delta:
                    continue
                # i/j 是"更仰视 / 更左"的方向键；逐车标定可能把数值方向反过来。
                previous = int(current.get(joint, fallback))
                lo, hi = limits[joint]
                step = delta * 40 * int(signs.get(joint, 1))
                value = max(lo, min(hi, previous + step))
                if value != previous:
                    targets[joint] = value
            if targets and not self.ctx.publish_servo(targets, .15):
                self._stop_locked()
            self.next_camera = now + .15

    def _stop_locked(self):
        self.blocked.update(self.pressed)
        self.pressed.clear()
        self.last_twist = (0., 0., 0.)
        self.ctx.stop_base()

    def stop(self):
        with self.lock:
            self._stop_locked()

    def close(self):
        self.closed.set()
        self.stop()
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=.5)

    def _run(self):
        while not self.closed.wait(1. / self.hz):
            try:
                self.tick()
            except Exception as exc:
                self.stop()
                self.ctx.log("遥控已停止：%s" % exc)
