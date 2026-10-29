"""Stop-line approach framework (pure logic, no wheels, no camera here).

The traffic module answers "may I move / must I hold"; this module answers
"how fast may I go as the car gets closer to the stop line".  It is driven by
two inputs only:

* ``scene`` — the dict from ``TrafficScenePolicy.snapshot()`` (red / junction);
* ``distance`` — metres to the stop line, supplied by the caller.  Today the
  caller can pass ``None`` (no measurement yet); tomorrow it can come from the
  lidar, from a visual stop-line estimate, or from odometry.

The returned ``DriveCommand.speed`` is a *limit* for ``TrafficScenePolicy.limit_twist``
style guarding: ``None`` means "this module does not restrict", ``0.0`` means
"stop".  Nothing here publishes a Twist or touches a servo, so it is fully
testable offline and safe to wire in later.

All numbers come from ``settings.json`` → ``junction``; they are placeholders
until the real stop-line distance and speeds are measured on the sandbox.
"""
import math

PARAMETERS = ("slow_distance", "stop_distance", "brake_distance",
              "approach_speed", "slow_speed", "crawl_speed", "go_speed",
              "crawl_timeout")


class DriveCommand:
    __slots__ = ("speed", "phase", "note")

    def __init__(self, speed, phase, note=""):
        self.speed = speed
        self.phase = phase
        self.note = note

    def __eq__(self, other):
        if not isinstance(other, DriveCommand):
            return NotImplemented
        return (self.speed, self.phase, self.note) == (other.speed, other.phase, other.note)

    def __repr__(self):
        return "DriveCommand(speed=%r, phase=%r, note=%r)" % (self.speed, self.phase, self.note)


def parameters(cfg):
    """Read and validate the junction parameters; a broken value is an error."""
    values = (cfg or {}).get("junction") or {}
    out = {}
    for key in PARAMETERS:
        value = values.get(key)
        if value is None:
            raise ValueError("缺少路口参数：%s" % key)
        number = float(value)
        if not math.isfinite(number) or number <= 0.:
            raise ValueError("路口参数无效：%s" % key)
        out[key] = number
    if not out["brake_distance"] < out["stop_distance"] < out["slow_distance"]:
        raise ValueError("路口距离门限必须满足 刹停 < 停止线 < 减速")
    return out


class StopLineDrive:
    """Distance-aware speed profile; the caller applies the value to the wheels."""

    IDLE = "idle"
    APPROACH = "approach"
    SLOW = "slow"
    CRAWL = "crawl"
    HOLD = "hold"
    GO = "go"

    def __init__(self, slow_distance=3.0, stop_distance=.9, brake_distance=.35,
                 approach_speed=.12, slow_speed=.08, crawl_speed=.04,
                 go_speed=.10, crawl_timeout=8.0):
        values = dict(slow_distance=slow_distance, stop_distance=stop_distance,
                      brake_distance=brake_distance, approach_speed=approach_speed,
                      slow_speed=slow_speed, crawl_speed=crawl_speed,
                      go_speed=go_speed, crawl_timeout=crawl_timeout)
        self.params = parameters({"junction": values})
        self.reset()

    @classmethod
    def from_config(cls, cfg):
        return cls(**parameters(cfg))

    def reset(self):
        self.phase = self.IDLE
        self.crawl_started = None
        self.last_distance = None

    def step(self, scene, distance, now):
        """Return the speed limit for this frame.

        ``scene`` is ``TrafficScenePolicy.snapshot()``; only ``red_pending`` /
        ``red_required`` / ``crosswalk_active`` are read.  ``distance`` is metres
        to the stop line or ``None`` when nothing measures it yet.
        """
        red = bool(scene.get("red_required") or scene.get("red_pending"))
        active = bool(scene.get("crosswalk_active"))
        distance = None if distance is None else float(distance)
        if distance is not None and (not math.isfinite(distance) or distance < 0.):
            raise ValueError("停止线距离无效")
        self.last_distance = distance
        if red and distance is None:
            self._phase(self.HOLD)
            return DriveCommand(0., self.HOLD, "红灯但还没有停止线距离，保持停车")
        if distance is None:
            self._phase(self.IDLE)
            return DriveCommand(None, self.IDLE, "")
        if red:
            command = self._towards_stop(distance, now)
        elif active:
            command = self._through_junction(distance)
        else:
            command = DriveCommand(None, self.IDLE, "")
        self._phase(command.phase)
        return command

    def _towards_stop(self, distance, now):
        p = self.params
        if distance <= p["brake_distance"]:
            self.crawl_started = None
            return DriveCommand(0., self.HOLD, "到达停止线，停车")
        if distance <= p["stop_distance"]:
            if self.crawl_started is None:
                self.crawl_started = now
            if now - self.crawl_started > p["crawl_timeout"]:
                return DriveCommand(0., self.HOLD, "爬行超时，保持停车")
            return DriveCommand(p["crawl_speed"], self.CRAWL, "低速爬行接近停止线")
        self.crawl_started = None
        if distance <= p["slow_distance"]:
            return DriveCommand(p["slow_speed"], self.SLOW, "红灯，减速")
        return DriveCommand(p["approach_speed"], self.APPROACH, "远处红灯，接近")

    def _through_junction(self, distance):
        p = self.params
        self.crawl_started = None
        if distance <= p["brake_distance"]:
            return DriveCommand(p["go_speed"], self.GO, "绿灯，起步")
        return DriveCommand(p["approach_speed"], self.GO, "路口通行，按接近速度")

    def _phase(self, phase):
        self.phase = phase


APPROACH_PARAMETERS = ("row_slope", "row_intercept", "row_trigger",
                       "finish_travel", "approach_speed", "crawl_speed",
                       "speed_scale")


def approach_parameters(cfg):
    """Read the pixel-row → distance model used when no rangefinder is available."""
    values = (cfg or {}).get("junction") or {}
    out = {}
    for key in APPROACH_PARAMETERS:
        value = values.get(key)
        if value is None:
            raise ValueError("缺少路口接近参数：%s" % key)
        number = float(value)
        if not math.isfinite(number):
            raise ValueError("路口接近参数无效：%s" % key)
        out[key] = number
    if out["row_slope"] >= 0.:
        raise ValueError("斑马线行号斜率必须为负")
    if out["row_trigger"] <= 0. or out["finish_travel"] <= 0.:
        raise ValueError("路口接近参数无效：row_trigger / finish_travel")
    if out["approach_speed"] <= 0. or not 0. < out["crawl_speed"] <= out["approach_speed"]:
        raise ValueError("路口接近速度无效")
    if out["speed_scale"] <= 0.:
        raise ValueError("实际速度比例无效")
    return out


class CrosswalkApproach:
    """Stop-line approach driven by the crosswalk pixel row, finished by odometry.

    The near crosswalk is only visible while the car is roughly 0.30–0.55 m from
    the stop line, so the profile is: approach at ``approach_speed`` until the
    crosswalk bottom row reaches ``row_trigger`` (≈0.32 m), then crawl and let the
    odometry finish the last ``finish_travel`` metres, then hold.

    ``step(row, red, travel, now)`` takes the crosswalk bottom row in the forward
    view (``None`` when it is not detected), whether the light is red, and the
    accumulated odometry travel since ``reset()``.  ``travel`` is in *commanded*
    metres (the vendor odometry integrates ``cmd_vel``, not the wheels), so it is
    multiplied by ``speed_scale`` before comparing with ``finish_travel``.
    Nothing here drives wheels.
    """

    IDLE = "idle"
    APPROACH = "approach"
    CRAWL = "crawl"
    HOLD = "hold"

    def __init__(self, row_slope=-0.002220, row_intercept=1.36125, row_trigger=470.,
                 finish_travel=.22, approach_speed=.10, crawl_speed=.04,
                 speed_scale=1.0):
        self.params = approach_parameters({"junction": dict(
            row_slope=row_slope, row_intercept=row_intercept, row_trigger=row_trigger,
            finish_travel=finish_travel, approach_speed=approach_speed,
            crawl_speed=crawl_speed, speed_scale=speed_scale)})
        self.reset()

    @classmethod
    def from_config(cls, cfg):
        return cls(**approach_parameters(cfg))

    def reset(self):
        self.phase = self.IDLE
        self.triggered = False
        self.trigger_travel = None

    def distance(self, row):
        """Metres to the stop line estimated from the crosswalk bottom row."""
        p = self.params
        return p["row_intercept"] + p["row_slope"] * float(row)

    @property
    def approach_speed(self):
        return self.params["approach_speed"]

    @property
    def crawl_speed(self):
        return self.params["crawl_speed"]

    def approach_command(self, speed, note):
        """Command for a caller that decides the speed itself (e.g. far approach)."""
        self.phase = self.APPROACH
        return DriveCommand(float(speed), self.APPROACH, note)

    def hold_command(self, note):
        """Command that stops the car for a reason outside the pixel-row profile."""
        self.phase = self.HOLD
        return DriveCommand(0., self.HOLD, note)

    def step(self, row, red, travel, now):
        p = self.params
        if travel is None:
            travel = 0.
        travel = float(travel)
        if not math.isfinite(travel) or travel < 0.:
            raise ValueError("里程计行程无效")
        if row is not None:
            row = float(row)
            if not math.isfinite(row) or row <= 0.:
                raise ValueError("斑马线行号无效")
        if not red:
            self.phase = self.IDLE
            return DriveCommand(None, self.IDLE, "红灯未确认，不接近")
        if not self.triggered:
            if row is None:
                self.phase = self.CRAWL
                return DriveCommand(p["crawl_speed"], self.CRAWL, "还没有斑马线行号，低速试探")
            if self.distance(row) <= p["row_intercept"] + p["row_slope"] * p["row_trigger"]:
                self.triggered = True
                self.trigger_start = travel
                self.phase = self.CRAWL
                return DriveCommand(p["crawl_speed"], self.CRAWL, "斑马线已到阈值，按里程计收尾")
            self.phase = self.APPROACH
            return DriveCommand(p["approach_speed"], self.APPROACH, "红灯，接近停止线")
        # /odom 是按指令积分出来的（厂家 odom_publisher_node 用 linear_x*dt 累加），
        # 实测真实速度约为指令速度的 speed_scale 倍，所以里程换算成真实米数再比较。
        used = (travel - float(self.trigger_start)) * p["speed_scale"]
        remaining = p["finish_travel"] - used
        if remaining <= 0.:
            self.phase = self.HOLD
            return DriveCommand(0., self.HOLD, "已到停止线，停车")
        self.phase = self.CRAWL
        return DriveCommand(p["crawl_speed"], self.CRAWL, "里程计收尾，还剩 %.2f m" % remaining)
