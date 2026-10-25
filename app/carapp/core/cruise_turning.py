"""Directional turn checks and bounded, fresh-scan confirmation when changing direction."""
from dataclasses import asdict, dataclass

from carapp.core.cruise_geometry import MotionCheck, VehicleGeometry, check_turns


@dataclass(frozen=True)
class TurnClearance:
    clear: bool
    reason: str
    front: float | None
    side: float | None
    tail: float | None
    tail_region: str
    motion: MotionCheck | None = None
    candidates: tuple = ()


def assess_turns(scan, settings, margin=0., geometry=None, *, speed=None):
    """Distances describe the scene; only swept trajectories approve a turn."""
    speed = min(settings.speed, .04, .02 if (scan.get("front") or 0.) < .45 else .04) if speed is None else speed
    candidates = check_turns(scan, speed, geometry, margin)
    result = {}
    for direction, front_key, tail_key in (
        ("left", "left_front", "right_rear"),
        ("right", "right_front", "left_rear"),
    ):
        front, side, tail = scan.get(front_key), scan.get(direction), scan.get(tail_key)
        options = candidates[direction]
        selected = next((item for item in options if item.clear), None)
        reason = "轨迹可行" if selected else "、".join(dict.fromkeys(item.reason for item in options))
        result[direction] = TurnClearance(selected is not None, reason, front, side, tail, tail_key,
                                          selected, options)
    return result


class TurnSelector:
    """Never prefer a wider alternative while the current turn remains available."""
    PAUSE_SECONDS = .30
    CHANGE_INTERVAL = 1.
    CONFIRM_FRAMES = 3
    ENTRY_MARGIN = .01  # added to the measured footprint's 2 cm geometric margin

    def __init__(self, geometry=None):
        self.geometry = geometry or VehicleGeometry()
        self.motion = None
        self.direction = None
        self.last_change = None
        self.blocked_at = None
        self.candidate = None
        self.confirmations = 0
        self.confirm_sequence = None
        self.options = {}
        self.entries = {}

    def _clear_confirmation(self):
        self.blocked_at = self.candidate = self.confirm_sequence = None
        self.confirmations = 0

    def _both_blocked(self):
        return "左右均不可行，停车等待（左：%s；右：%s）" % (
            self.entries["left"].reason, self.entries["right"].reason)

    def _use(self, direction, item):
        # Keep the current curvature while it is clear; try another magnitude on
        # this side before declaring the direction blocked.
        same = next((option for option in item.candidates if option.clear and self.motion
                     and option.yaw == self.motion.yaw), None)
        self.motion = same or item.motion
        return direction, ""

    def choose(self, scan, settings, now, straight_clear=False, *, speed=None, prefer_straight=False):
        self.options = assess_turns(scan, settings, geometry=self.geometry, speed=speed)
        self.entries = assess_turns(scan, settings, self.ENTRY_MARGIN, self.geometry, speed=speed)
        if self.direction is None:
            if straight_clear and prefer_straight:
                return "straight", "前方轨迹恢复可见，低速直行确认"
            # Like the vendor's app.lidar_controller avoidance, compare the two
            # front halves first. Side width only breaks ties after all entry
            # checks (including the opposite tail) have passed.
            available = [(item.front or 0., item.side or 0., direction)
                         for direction, item in self.entries.items() if item.clear]
            if not available:
                return ("straight", "转弯空间受限，保持前进直行") if straight_clear else (None, self._both_blocked())
            self.direction = max(available)[2]
            self.last_change = now
            return self._use(self.direction, self.entries[self.direction])

        current = self.options[self.direction]
        if current.clear and self.blocked_at is None:
            return self._use(self.direction, current)
        if not current.clear and straight_clear:
            self._clear_confirmation()
            return "straight", "转弯空间受限，保持前进直行"
        if self.blocked_at is None:
            self.blocked_at = now

        other = "right" if self.direction == "left" else "left"
        candidate = self.direction if current.clear else (other if self.entries[other].clear else None)
        fresh = scan["sequence"] != self.confirm_sequence
        if fresh:
            self.confirm_sequence = scan["sequence"]
            if candidate is not None and candidate == self.candidate:
                self.confirmations = min(self.CONFIRM_FRAMES, self.confirmations+1)
            else:
                self.candidate = candidate
                self.confirmations = 1 if candidate is not None else 0
        changing = candidate is not None and candidate != self.direction
        pause_done = now-self.blocked_at >= self.PAUSE_SECONDS
        interval_done = not changing or now-self.last_change >= self.CHANGE_INTERVAL
        if fresh and candidate is not None and self.confirmations >= self.CONFIRM_FRAMES and pause_done and interval_done:
            self.direction = candidate
            if changing:
                self.last_change = now
            self._clear_confirmation()
            return self._use(self.direction, self.entries[self.direction] if changing else current)
        if candidate is None:
            return None, self._both_blocked()
        name = "左" if candidate == "left" else "右"
        if self.confirmations >= self.CONFIRM_FRAMES:
            if not pause_done:
                return None, "已确认%s转，等待停车稳定" % name
            if not interval_done:
                return None, "已确认%s转，等待换向间隔" % name
        return None, "当前转弯受阻，停车确认%s转 %d/%d" % (name, self.confirmations, self.CONFIRM_FRAMES)

    def snapshot(self):
        return dict(direction=self.direction, candidate=self.candidate, confirmations=self.confirmations,
                    motion=asdict(self.motion) if self.motion else None,
                    blocked_at=self.blocked_at, last_change=self.last_change,
                    options={direction:dict(asdict(item), can_start=self.entries[direction].clear,
                                            start_reason=self.entries[direction].reason)
                             for direction,item in self.options.items()})
