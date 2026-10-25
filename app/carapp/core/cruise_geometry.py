"""Measured vehicle footprint and short forward trajectories, independent of ROS/Tk."""
import json
import math
from dataclasses import asdict, dataclass
from functools import lru_cache
from pathlib import Path


@dataclass(frozen=True)
class VehicleGeometry:
    # User measured: width 17 cm, lidar-to-claw 22 cm, lidar-to-tail 9 cm.
    # The CAD lidar joint is 3.24356 cm behind the cmd_vel/base origin.
    front: float = .1875643963119436
    rear: float = .1224356036880564
    half_width: float = .085
    lidar_x: float = -.0324356036880564
    lidar_y: float = 0.
    lidar_yaw: float = 0.
    margin: float = .02
    prediction_seconds: float = 2.
    control_delay: float = .25
    linear_deceleration: float = .25
    angular_deceleration: float = .8

    def __post_init__(self):
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
               for v in asdict(self).values()):
            raise ValueError("车体轮廓参数必须为有限数值")
        if not (.05 <= self.front <= .6 and .03 <= self.rear <= .4 and .04 <= self.half_width <= .3):
            raise ValueError("车体前伸、后伸或半宽超出有效范围")
        if not (-self.rear < self.lidar_x < self.front and abs(self.lidar_y) < self.half_width):
            raise ValueError("雷达原点必须位于车体轮廓内")
        if not (0 <= self.margin <= .10 and 1. <= self.prediction_seconds <= 4.
                and .1 <= self.control_delay <= 1. and .05 <= self.linear_deceleration <= 1.
                and .1 <= self.angular_deceleration <= 3.):
            raise ValueError("轨迹余量或预测、制动参数无效")


def load_vehicle_geometry(path=None):
    path = Path(path) if path else Path(__file__).resolve().parents[2]/"config/cruise_geometry.json"
    try:
        payload = json.loads(path.read_text(encoding="utf-8-sig"))
        return VehicleGeometry(**payload["geometry"])
    except (OSError, ValueError, TypeError, KeyError) as exc:
        raise ValueError("无法读取巡航车体轮廓："+str(exc)) from exc


@dataclass(frozen=True)
class MotionCheck:
    clear: bool
    reason: str
    speed: float
    yaw: float
    horizon: float = 0.
    collision_point: tuple | None = None
    collision_time: float | None = None
    blocked_angle: float | None = None


def _advance(x, y, angle, speed, yaw, dt):
    if abs(yaw) < 1e-10:
        return x+speed*math.cos(angle)*dt, y+speed*math.sin(angle)*dt, angle
    end = angle+yaw*dt
    return (x+speed/yaw*(math.sin(end)-math.sin(angle)),
            y+speed/yaw*(math.cos(angle)-math.cos(end)), end)


@lru_cache(maxsize=96)
def _trajectory(speed, yaw, geometry):
    """Constant motion, control delay, then simultaneous linear/angular braking."""
    x = y = angle = elapsed = 0.
    poses = [(0., 0., 0., 1., 0.)]
    moving = geometry.prediction_seconds+geometry.control_delay
    braking = max(speed/geometry.linear_deceleration, abs(yaw)/geometry.angular_deceleration)
    steps = math.ceil((moving+braking)/.05)
    for index in range(1, steps+1):
        end = min(index*.05, moving+braking)
        dt = end-elapsed
        middle = (end+elapsed)/2-moving
        v = speed if middle <= 0 else max(0., speed-geometry.linear_deceleration*middle)
        w = yaw if middle <= 0 else math.copysign(max(0., abs(yaw)-geometry.angular_deceleration*middle), yaw)
        x, y, angle = _advance(x, y, angle, v, w, dt)
        elapsed = end
        poses.append((elapsed, x, y, math.cos(angle), math.sin(angle)))
    # Inflate by half a sample's maximum boundary displacement, covering between poses.
    corner = math.hypot(max(geometry.front, geometry.rear), geometry.half_width)
    sampling_pad = (speed+abs(yaw)*corner)*.025
    return tuple(poses), elapsed, sampling_pad


def _intersection(ox, oy, dx, dy, front, rear, half_width):
    near, far = 0., math.inf
    for origin, direction, lo, hi in ((ox, dx, -rear, front), (oy, dy, -half_width, half_width)):
        if abs(direction) < 1e-10:
            if not lo <= origin <= hi:
                return None
        else:
            a, b = (lo-origin)/direction, (hi-origin)/direction
            if a > b:
                a, b = b, a
            near, far = max(near, a), min(far, b)
            if far < near:
                return None
    return near, far


def evaluate_motion(scan, speed, yaw, geometry=None, extra_margin=0.):
    """Check swept rectangles and newly needed visibility against complete scan rays.

    Each ray is (angle in lidar frame, known free distance or None, has_return).
    Unknown/occluded space exposed by the trajectory cannot be certified clear.
    """
    geometry = geometry or VehicleGeometry()
    if not all(math.isfinite(v) for v in (speed, yaw, extra_margin)) or not 0 < speed <= .15 or abs(yaw) > .4 or extra_margin < 0:
        return MotionCheck(False, "轨迹速度或余量无效", speed, yaw)
    rays = scan.get("rays")
    if scan.get("error") or not isinstance(rays, (tuple, list)) or not 100 <= len(rays) <= 10000:
        return MotionCheck(False, "缺少完整雷达点，无法检查转弯轨迹", speed, yaw)
    visible = scan.get("visibility_rays", rays)
    if not isinstance(visible,(tuple,list)) or len(visible)!=len(rays):
        return MotionCheck(False, "雷达可见范围数据无效", speed, yaw)
    poses, horizon, sampling_pad = _trajectory(speed, yaw, geometry)
    pad = geometry.margin+extra_margin+sampling_pad
    front, rear, width = geometry.front+pad, geometry.rear+pad, geometry.half_width+pad
    bound = math.hypot(max(front, rear), width)+math.hypot(geometry.lidar_x, geometry.lidar_y)+speed*horizon
    relevant = []
    for ray in visible:
        try:
            angle, distance, hit = ray
            if not math.isfinite(angle) or (distance is not None and (not math.isfinite(distance) or distance <= 0)):
                raise ValueError
        except (ValueError, TypeError):
            return MotionCheck(False, "雷达点格式无效", speed, yaw)
        if distance is not None and distance > bound:
            continue
        a = angle+geometry.lidar_yaw
        dx, dy = math.cos(a), math.sin(a)
        baseline = _intersection(geometry.lidar_x, geometry.lidar_y, dx, dy, front, rear, width)
        relevant.append((angle, distance, hit, dx, dy, baseline[1]))
    unknown = None
    for elapsed, x, y, c, s in poses:
        lx, ly = geometry.lidar_x-x, geometry.lidar_y-y
        ox, oy = lx*c+ly*s, -lx*s+ly*c
        for angle, distance, hit, dx, dy, baseline in relevant:
            interval = _intersection(ox, oy, dx*c+dy*s, -dx*s+dy*c, front, rear, width)
            if interval is None:
                continue
            near, far = interval
            if distance is not None and hit and near-1e-6 <= distance <= far+1e-6:
                px, py = geometry.lidar_x+distance*dx, geometry.lidar_y+distance*dy
                region = ("左" if py >= 0 else "右")+("前" if px >= 0 else "后")
                return MotionCheck(False, region+"扫掠区域有障碍", speed, yaw, horizon,
                                   (px, py), elapsed, math.degrees(angle))
            if far > baseline+.002 and (distance is None or distance < far):
                if unknown is None:
                    unknown = MotionCheck(False, "轨迹区域雷达数据不足" if distance is None else "轨迹区域被障碍遮挡",
                                          speed, yaw, horizon, collision_time=elapsed, blocked_angle=math.degrees(angle))
    return unknown or MotionCheck(True, "轨迹可行", speed, yaw, horizon)


def check_turns(scan, speed, geometry=None, extra_margin=0.):
    """Prefer gentle arcs when clear; test stronger steering before rejecting a side."""
    return {direction: tuple(evaluate_motion(scan, speed, sign*yaw, geometry, extra_margin)
                             for yaw in (.15, .25, .40))
            for direction, sign in (("left", 1), ("right", -1))}
