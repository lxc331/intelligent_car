"""Laser-based forward cruise and forward-arc avoidance for the ClawPi base."""
import math
import threading
import time
from dataclasses import dataclass

from carapp.core.laser_scan import LaserMonitor
from carapp.core.cruise_turning import TurnSelector
from carapp.core.cruise_geometry import VehicleGeometry, evaluate_motion, load_vehicle_geometry


@dataclass(frozen=True)
class CruiseSettings:
    front_back_distance: float = .60
    speed: float = .06
    side_distance: float = .35

    def __post_init__(self):
        for label, distance in (("前后", self.front_back_distance), ("左右", self.side_distance)):
            if not math.isfinite(distance) or not .10 <= distance <= 2.0:
                raise ValueError(label+"检测距离须为 0.10–2.00 米")
        if not math.isfinite(self.speed) or not .02 <= self.speed <= .15:
            raise ValueError("巡航速度须为 0.02–0.15 米/秒")


class CruisePolicy:
    """Keep forward velocity through clear avoidance arcs; stop on unsafe geometry."""
    AVOID_TIMEOUT = 20.
    TURN_SPEED = .40
    ARC_SPEED = .04

    def __init__(self, geometry=None):
        self.geometry = geometry or VehicleGeometry()
        self.phase = "forward"
        self.avoid_started = None
        self.turning = TurnSelector(self.geometry)
        self.clear_count = 0
        self.last_sequence = None
        self.last_now = None

    @property
    def direction(self):
        return self.turning.direction

    @property
    def diagnostics(self):
        elapsed = self.last_now-self.avoid_started if self.avoid_started is not None else 0.
        return dict(self.turning.snapshot(), avoid_elapsed=elapsed)

    def decide(self, scan, settings, now):
        # Result: forward speed, yaw rate, phase text, fatal stop. Lateral speed is always zero.
        self.last_now = now
        age = now-scan.get("received_at", 0.)
        if age < -.01 or age > LaserMonitor.TIMEOUT:
            return 0., 0., "雷达数据超时，已停止，请重新启动", True
        if scan.get("error") or scan.get("front") is None:
            return 0., 0., scan.get("error") or "前方雷达不可用", True
        front = scan["front"]
        sides = [scan.get(key) for key in ("left", "right")]
        ahead_sides = [scan.get(key+"_ahead", scan.get(key)) for key in ("left", "right")]
        if any(value is None for value in sides+ahead_sides):
            return 0., 0., "左右雷达数据不足，已停止，请重新启动", True
        side_limit = settings.side_distance
        # An obstacle already behind the axle must not cause another turn toward it.
        # Forward sectors trigger avoidance; the full footprint checks every candidate.
        side_alert = min(ahead_sides) < side_limit
        if self.phase == "forward":
            if front >= settings.front_back_distance and not side_alert:
                speed = settings.speed if front >= settings.front_back_distance+.20 else min(settings.speed, .06)
                if front < .50:
                    speed = min(speed, .03)
                for candidate_speed in dict.fromkeys((speed, min(speed, .02))):
                    if evaluate_motion(scan, candidate_speed, 0., self.geometry).clear:
                        return candidate_speed, 0., "自动前进" if candidate_speed == settings.speed else "前方接近障碍，慢速前进", False
            self.phase, self.avoid_started = "avoid", now
        if self.avoid_started is not None and now-self.avoid_started >= self.AVOID_TIMEOUT:
            return 0., 0., "20 秒内未找到绕行通道，已停止", True
        if scan["sequence"] != self.last_sequence:
            self.last_sequence = scan["sequence"]
            clear = front >= settings.front_back_distance+.08 and min(ahead_sides) >= side_limit+.03
            self.clear_count = self.clear_count+1 if clear else 0
        if self.clear_count >= 3:
            # Clearing a distance threshold alone cannot overrule a blocked body trajectory.
            straight = evaluate_motion(scan, min(settings.speed, .06), 0., self.geometry)
            if not straight.clear:
                self.clear_count = 0
            else:
                self.turning = TurnSelector(self.geometry)
                self.clear_count = 0
                self.phase = "forward"
                self.avoid_started = None
                return self.decide(scan, settings, now)
        # If an already-passed side obstacle blocks rotation, a separately checked
        # full-width forward corridor can still permit driving away without turning.
        corridor = scan.get("straight_corridor")
        straight_clear = front >= settings.front_back_distance and not side_alert
        if "straight_corridor" in scan:
            straight_clear = front >= settings.front_back_distance and corridor is not None and corridor >= settings.front_back_distance
        speed = min(settings.speed, self.ARC_SPEED, .02 if front < .45 else self.ARC_SPEED)
        straight_clear = straight_clear and evaluate_motion(scan, speed, 0., self.geometry).clear
        direction, text = self.turning.choose(scan, settings, now, straight_clear=straight_clear,
                                               speed=speed, prefer_straight=not side_alert)
        if direction is None:
            return 0., 0., text, False
        if direction == "straight":
            return speed, 0., text, False
        motion = self.turning.motion
        return motion.speed, motion.yaw, "前进向%s转弯绕障（%s）" % (
            "左" if motion.yaw > 0 else "右", {.15:"缓转", .25:"中转", .4:"急转"}[abs(motion.yaw)]), False


class CruiseRunner:
    """Independent watchdog thread; a stalled Tk event loop cannot keep stale commands alive."""
    def __init__(self, ctx, monitor, notify=lambda text: None, clock=time.monotonic):
        self.ctx, self.monitor, self.notify, self.clock = ctx, monitor, notify, clock
        self.lock = threading.RLock()
        self.cancel = threading.Event()
        self.thread = None
        self.running = False
        self.settings = CruiseSettings()
        self.geometry = VehicleGeometry()
        self.phase = "待机"
        self.policy = CruisePolicy()

    def _phase(self, text):
        if text != self.phase:
            self.phase = text
            self.notify(text)

    def configure(self, settings):
        with self.lock:
            self.settings = settings
            self.policy = CruisePolicy(self.geometry)
            if self.running:
                self.ctx.stop_base()
                self._phase("参数已更新，重新评估障碍")

    def start(self, settings):
        with self.lock:
            if self.running or (self.thread and self.thread.is_alive()):
                raise ValueError("巡航已启动或上次动作正在停止")
            if not self.ctx.state.get("ws_connected"):
                raise ValueError("ROS 尚未连接")
            self.geometry = load_vehicle_geometry()
            self.monitor.set_geometry(self.geometry)
            scan = self.monitor.snapshot()
            check = CruisePolicy(self.geometry).decide(scan, settings, self.clock())
            if check[3]:
                raise ValueError(check[2])
            if not self.ctx.state.get("cam_connected"):
                raise ValueError("请等待实时画面就绪后再启动巡航")
            self.settings, self.policy = settings, CruisePolicy(self.geometry)
            self.cancel = threading.Event()
            self.running = True
            self.ctx.stop_base()
            self._phase("巡航已启动")
            self.thread = threading.Thread(target=self._run, name="cruise-avoidance", daemon=True)
            self.thread.start()

    def step(self):
        with self.lock:
            if not self.running or self.cancel.is_set():
                return
            if not self.ctx.state.get("ws_connected") or not self.ctx.bridge.connected:
                decision = (0., 0., "ROS 连接中断，已取消巡航", True)
            elif not self.ctx.state.get("cam_connected"):
                decision = (0., 0., "实时画面中断，已取消巡航", True)
            else:
                scan = self.monitor.snapshot()
                decision = self.policy.decide(scan, self.settings, self.clock())
                if self.clock()-scan.get("received_at",0.) > LaserMonitor.TIMEOUT:
                    decision = (0., 0., "轨迹检查期间雷达数据已超时，已取消巡航", True)
            vx, wz, text, fatal = decision
            if fatal:
                self.running = False
                self.cancel.set()
                self.ctx.stop_base()
            elif not self.ctx.publish_twist(vx, wz, quiet=True):
                self.running = False
                self.cancel.set()
                self.ctx.stop_base()
                text = "底盘指令发送失败，已取消巡航"
            self._phase(text)

    def stop(self, reason="已停止巡航"):
        self.cancel.set()
        with self.lock:
            self.running = False
            self.ctx.stop_base()
            self._phase(reason)
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=.6)

    def _run(self):
        try:
            while not self.cancel.is_set():
                started = time.monotonic()
                self.step()
                self.cancel.wait(max(0., .10-(time.monotonic()-started)))
        except Exception as exc:
            with self.lock:
                self.running = False
                self.cancel.set()
                self._phase("巡航异常，已停止："+str(exc))
        finally:
            self.ctx.stop_base()
