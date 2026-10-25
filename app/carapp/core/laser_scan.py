"""LaserScan parsing and freshness checks, independent of the cruise strategy and UI."""
import math
import threading

from carapp.core.cruise_geometry import VehicleGeometry


def supported_visibility(rays):
    """Bound tiny holes by the nearer real neighbor; retain raw unknown rays.

    At most two missing beams spanning <= 2 degrees are supported, with a
    further 3 cm range deduction. No extrapolation or chained filling.
    """
    known = next((index for index,ray in enumerate(rays) if ray[1] is not None), None)
    if known is None:
        return tuple(rays), ()
    result, filled, gap = list(rays), [], []
    anchor = known
    for offset in range(1,len(rays)+1):
        index = (known+offset)%len(rays)
        if rays[index][1] is None:
            gap.append(index)
            continue
        if gap:
            span = abs(math.atan2(math.sin(rays[index][0]-rays[anchor][0]),
                                  math.cos(rays[index][0]-rays[anchor][0])))
            missing_angle = span*len(gap)/(len(gap)+1)
            distance = min(rays[anchor][1],rays[index][1])-.03
            if len(gap)<=2 and missing_angle<=math.radians(2.)+1e-6 and distance>0:
                for hole in gap:
                    result[hole]=(rays[hole][0],distance,False)
                    filled.append(hole)
            gap=[]
        anchor=index
    return tuple(result),tuple(filled)


class LaserMonitor:
    """Keep sensor receipt time, not UI-processing time; never treat null as free space."""
    SECTORS = {"front": (-45., 45.), "left": (35., 145.), "right": (-145., -35.),
               "rear": (145., 215.), "left_ahead": (35., 90.), "right_ahead": (-90., -35.),
               "left_front": (0., 90.), "right_front": (-90., 0.),
               "left_rear": (90., 180.), "right_rear": (180., 270.),
               "straight_corridor": (-90., 90.)}
    CORRIDOR_HALF_WIDTH = VehicleGeometry().half_width+VehicleGeometry().margin
    TIMEOUT = .55

    def __init__(self):
        self.lock = threading.Lock()
        self.corridor_half_width = self.CORRIDOR_HALF_WIDTH
        self.reset()

    def set_geometry(self, geometry):
        with self.lock:
            self.corridor_half_width = geometry.half_width+geometry.margin

    def reset(self):
        with self.lock:
            self.stamp = None
            self.sequence = 0
            self.view = dict(received_at=0., sequence=0, front=None, left=None, right=None, rear=None,
                             left_ahead=None, right_ahead=None,
                             straight_corridor=None,
                             rays=None, coverage={}, error="等待 /scan_raw 雷达数据")

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
            rays = []
            for index, raw in enumerate(ranges):
                angle = math.degrees(math.atan2(math.sin(start+index*increment), math.cos(start+index*increment)))
                value = None
                if isinstance(raw, (float, int)) and not isinstance(raw, bool):
                    # +Inf explicitly means no return within range. JSON null is ambiguous.
                    if raw == math.inf:
                        value = maximum
                    elif math.isfinite(raw) and 0 < raw <= maximum:
                        value = float(raw)  # even a positive reading below range_min is a close hazard
                rays.append((math.radians(angle), value, value is not None and raw != math.inf))
                for key, (lo, hi) in self.SECTORS.items():
                    sector_angle = angle % 360. if key in ("rear", "left_rear", "right_rear") else angle
                    if lo-1e-6 <= sector_angle <= hi+1e-6:
                        distance = value
                        if key in ("front", "straight_corridor") and value is not None and raw != math.inf:
                            radians = math.radians(angle)
                            # Off-axis returns outside the driving corridor belong
                            # to the independently configured side detection.
                            distance = (value*math.cos(radians)
                                        if abs(value*math.sin(radians)) <= self.corridor_half_width
                                        else maximum)
                        sectors[key].append((sector_angle, distance))
            visible, filled = supported_visibility(rays)
            view = dict(received_at=received_at, error="", coverage={}, rays=tuple(rays),
                        visibility_rays=visible, supported_gap_indices=filled)
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
                        straight_corridor=None,
                        rays=None, coverage={}, error="无效雷达数据："+str(exc))
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

