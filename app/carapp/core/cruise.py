"""Laser-based forward cruise and forward-arc avoidance for the ClawPi base."""
import math
import threading
import time
from dataclasses import dataclass


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


class LaserMonitor:
    """Keep sensor receipt time, not UI-processing time; never treat null as free space."""
    SECTORS = {"front": (-45., 45.), "left": (35., 145.), "right": (-145., -35.),
               "rear": (145., 215.), "left_ahead": (35., 90.), "right_ahead": (-90., -35.)}
    # Vendor avoidance uses a 0.40 m vehicle envelope; add 2 cm on each side.
    CORRIDOR_HALF_WIDTH = .22
    TIMEOUT = .55

    def __init__(self):
        self.lock = threading.Lock()
        self.reset()

    def reset(self):
        with self.lock:
            self.stamp = None
            self.sequence = 0
            self.view = dict(received_at=0., sequence=0, front=None, left=None, right=None, rear=None,
                             left_ahead=None, right_ahead=None,
                             coverage={}, error="等待 /scan_raw 雷达数据")

    def update(self, msg, received_at):
        try:
            ranges = msg["ranges"]
            start, increment = float(msg["angle_min"]), float(msg["angle_increment"])
            minimum, maximum = float(msg["range_min"]), float(msg["range_max"])
            header = msg["header"]
            if header.get("frame_id", "").lstrip("/") != "lidar_frame":
                raise ValueError("雷达坐标系不是 lidar_frame，拒绝猜测前方方向")
            stamp = (int(header["stamp"]["sec"]), int(header["stamp"]["nanosec"]))
            if not all(math.isfinite(v) for v in (start, increment, minimum, maximum, received_at)):
                raise ValueError("雷达参数不是有限数值")
            if not isinstance(ranges, (list, tuple)) or not 100 <= len(ranges) <= 10000:
                raise ValueError("雷达点数不完整")
            if not 5.5 <= abs(increment) * (len(ranges)-1) <= 6.5 or not 0 <= minimum < maximum <= 10000:
                raise ValueError("需要完整的 360° 雷达扫描和有效量程")
            sectors = {key: [] for key in self.SECTORS}
            for index, raw in enumerate(ranges):
                angle = math.degrees(math.atan2(math.sin(start+index*increment), math.cos(start+index*increment)))
                value = None
                if isinstance(raw, (float, int)) and not isinstance(raw, bool):
                    # +Inf explicitly means no return within range. JSON null is ambiguous.
                    if raw == math.inf:
                        value = maximum
                    elif math.isfinite(raw) and 0 < raw <= maximum:
                        value = float(raw)  # even a positive reading below range_min is a close hazard
                for key, (lo, hi) in self.SECTORS.items():
                    sector_angle = angle % 360. if key == "rear" else angle
                    if lo <= sector_angle <= hi:
                        distance = value
                        if key == "front" and value is not None and raw != math.inf:
                            radians = math.radians(angle)
                            # Off-axis returns outside the driving corridor belong
                            # to the independently configured side detection.
                            distance = (value*math.cos(radians)
                                        if abs(value*math.sin(radians)) <= self.CORRIDOR_HALF_WIDTH
                                        else maximum)
                        sectors[key].append((sector_angle, distance))
            view = dict(received_at=received_at, error="", coverage={})
            for key, samples in sectors.items():
                samples.sort()
                valid = [value for _, value in samples if value is not None]
                coverage = len(valid) / max(1, len(samples))
                longest = missing = 0
                for _, value in samples:
                    missing = missing+1 if value is None else 0
                    longest = max(longest, missing)
                healthy = len(valid) >= 8 and coverage >= .65 and math.degrees(abs(increment))*longest <= 12.
                view[key] = min(valid) if healthy else None
                view["coverage"][key] = coverage
            if view["front"] is None:
                view["error"] = "前方雷达数据不足或存在过大盲区"
        except (KeyError, ValueError, TypeError, OverflowError) as exc:
            view = dict(received_at=received_at, front=None, left=None, right=None, rear=None,
                        left_ahead=None, right_ahead=None,
                        coverage={}, error="无效雷达数据："+str(exc))
            stamp = None
        with self.lock:
            if stamp is not None and stamp == self.stamp:
                return False
            self.stamp = stamp
            self.sequence += 1
            self.view = dict(view, sequence=self.sequence)
        return True

    def snapshot(self):
        with self.lock:
            return dict(self.view)


class CruisePolicy:
    """Keep forward velocity through clear avoidance arcs; stop on unsafe geometry."""
    AVOID_TIMEOUT = 20.
    TURN_SPEED = .40
    ARC_SPEED = .04

    def __init__(self):
        self.phase = "forward"
        self.avoid_started = None
        self.direction = None
        self.clear_count = 0
        self.last_sequence = None

    def decide(self, scan, settings, now):
        # Result: forward speed, yaw rate, phase text, fatal stop. Lateral speed is always zero.
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
        # Full side sectors still guard rotation; forward sectors decide whether to steer away.
        side_alert = min(ahead_sides) < side_limit
        if self.phase == "forward":
            if front >= settings.front_back_distance and not side_alert:
                speed = settings.speed if front >= settings.front_back_distance+.20 else min(settings.speed, .06)
                if front < .50:
                    speed = min(speed, .03)
                return speed, 0., "自动前进" if speed == settings.speed else "前方接近障碍，慢速前进", False
            self.phase, self.avoid_started = "avoid", now
        if self.avoid_started is not None and now-self.avoid_started > self.AVOID_TIMEOUT:
            return 0., 0., "20 秒内未找到绕行通道，已停止", True
        if scan["sequence"] != self.last_sequence:
            self.last_sequence = scan["sequence"]
            clear = front >= settings.front_back_distance+.08 and min(ahead_sides) >= side_limit+.03
            self.clear_count = self.clear_count+1 if clear else 0
        if self.clear_count >= 3:
            self.phase = "forward"
            self.avoid_started = self.direction = None
            self.clear_count = 0
            return self.decide(scan, settings, now)
        # Forward motion may leave a rear obstacle; a turn still needs rear swing room.
        rear = scan.get("rear")
        if rear is None or rear <= settings.front_back_distance:
            if front >= settings.front_back_distance and not side_alert:
                speed = min(settings.speed, self.ARC_SPEED, .02 if front < .45 else self.ARC_SPEED)
                return speed, 0., "后方转弯空间受限，保持前进直行", False
            return 0., 0., "后方未满足前后检测距离或数据不足，停车等待转弯", False
        if self.direction is None:
            candidates = [(scan[key], key) for key in ("left", "right")
                          if scan[key] > side_limit+.03]
            if not candidates:
                return 0., 0., "左右通道受阻，停车等待", False
            self.direction = max(candidates)[1]
        side = scan.get(self.direction)
        if side is None or side <= side_limit:
            return 0., 0., "绕行侧未满足左右检测距离，停车等待", False
        speed = min(settings.speed, self.ARC_SPEED, .02 if front < .45 else self.ARC_SPEED)
        wz = self.TURN_SPEED * (1 if self.direction == "left" else -1)
        return speed, wz, "前进向%s转弯绕障" % ("左" if wz > 0 else "右"), False


class CruiseRunner:
    """Independent watchdog thread; a stalled Tk event loop cannot keep stale commands alive."""
    def __init__(self, ctx, monitor, notify=lambda text: None, clock=time.monotonic):
        self.ctx, self.monitor, self.notify, self.clock = ctx, monitor, notify, clock
        self.lock = threading.RLock()
        self.cancel = threading.Event()
        self.thread = None
        self.running = False
        self.settings = CruiseSettings()
        self.phase = "待机"
        self.policy = CruisePolicy()

    def _phase(self, text):
        if text != self.phase:
            self.phase = text
            self.notify(text)

    def configure(self, settings):
        with self.lock:
            self.settings = settings
            self.policy = CruisePolicy()
            if self.running:
                self.ctx.stop_base()
                self._phase("参数已更新，重新评估障碍")

    def start(self, settings):
        with self.lock:
            if self.running or (self.thread and self.thread.is_alive()):
                raise ValueError("巡航已启动或上次动作正在停止")
            if not self.ctx.state.get("ws_connected"):
                raise ValueError("ROS 尚未连接")
            scan = self.monitor.snapshot()
            check = CruisePolicy().decide(scan, settings, self.clock())
            if check[3]:
                raise ValueError(check[2])
            if not self.ctx.state.get("cam_connected"):
                raise ValueError("请等待实时画面就绪后再启动巡航")
            self.settings, self.policy = settings, CruisePolicy()
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
                decision = self.policy.decide(self.monitor.snapshot(), self.settings, self.clock())
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
                self.step()
                self.cancel.wait(.10)
        except Exception as exc:
            with self.lock:
                self.running = False
                self.cancel.set()
                self._phase("巡航异常，已停止："+str(exc))
        finally:
            self.ctx.stop_base()
