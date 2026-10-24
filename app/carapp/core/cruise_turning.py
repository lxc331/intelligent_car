"""Directional turn checks and bounded, fresh-scan confirmation when changing direction."""
import math
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class TurnClearance:
    clear: bool
    reason: str
    front: float | None
    side: float | None
    tail: float | None
    tail_region: str


def assess_turns(scan, settings, margin=0.):
    """Left arcs need right-rear swing room; right arcs need left-rear swing room."""
    result = {}
    for direction, front_key, tail_key, name, tail_name in (
        ("left", "left_front", "right_rear", "左", "右后"),
        ("right", "right_front", "left_rear", "右", "左后"),
    ):
        front, side, tail = scan.get(front_key), scan.get(direction), scan.get(tail_key)
        reasons = []
        for label, value, threshold in ((name+"前", front, settings.side_distance+margin),
                                         (name+"侧", side, settings.side_distance+margin),
                                         (tail_name, tail, settings.front_back_distance+margin)):
            if not isinstance(value, (float, int)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                reasons.append(label+"数据不足")
            elif value <= threshold:
                reasons.append(label+"空间不足")
        result[direction] = TurnClearance(not reasons, "、".join(reasons) or "通路可行",
                                          front, side, tail, tail_key)
    return result


class TurnSelector:
    """Never prefer a wider alternative while the current turn remains available."""
    PAUSE_SECONDS = .30
    CHANGE_INTERVAL = 1.
    CONFIRM_FRAMES = 3
    ENTRY_MARGIN = .03

    def __init__(self):
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

    def choose(self, scan, settings, now, straight_clear=False):
        self.options = assess_turns(scan, settings)
        self.entries = assess_turns(scan, settings, self.ENTRY_MARGIN)
        if self.direction is None:
            # Like the vendor's app.lidar_controller avoidance, compare the two
            # front halves first. Side width only breaks ties after all entry
            # checks (including the opposite tail) have passed.
            available = [(item.front, item.side, direction)
                         for direction, item in self.entries.items() if item.clear]
            if not available:
                return ("straight", "转弯空间受限，保持前进直行") if straight_clear else (None, self._both_blocked())
            self.direction = max(available)[2]
            self.last_change = now
            return self.direction, ""

        current = self.options[self.direction]
        if current.clear and self.blocked_at is None:
            return self.direction, ""
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
            return self.direction, ""
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
                    blocked_at=self.blocked_at, last_change=self.last_change,
                    options={direction:dict(asdict(item), can_start=self.entries[direction].clear,
                                            start_reason=self.entries[direction].reason)
                             for direction,item in self.options.items()})
