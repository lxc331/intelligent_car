"""Reroute-first obstacle handling for the sandbox road network.

The car stops itself and keeps the goal alive (see ForwardGuard on the car), so
this module only decides *what the driver should do next*: reroute from where
the car stands, or wait for the road to clear. Waiting must never cancel the
retained goal — that would leave the car unable to continue once the obstacle
is gone, which is exactly what the operator expects to happen automatically.
It is deliberately free of ROS, Tk and timers so the whole decision table can be
tested offline.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class AvoidanceSettings:
    enabled: bool = True
    reroute_after: float = 1.0      # 障碍稳定多久才尝试绕行
    wait_seconds: float = 10.0      # 等待多久后升级一次提示（不取消）
    give_up_seconds: float = 0.     # >0 时等待超过这么久才自动停车；0 = 一直等
    max_turn: float = math.radians(75.)

    def __post_init__(self):
        for name, value in (('reroute_after', self.reroute_after),
                            ('wait_seconds', self.wait_seconds),
                            ('give_up_seconds', self.give_up_seconds),
                            ('max_turn', self.max_turn)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('避障处置参数须为有限数值：' + name)
        if not .3 <= self.reroute_after <= 5.:
            raise ValueError('绕行确认时间须在 0.3–5 秒')
        if not 2. <= self.wait_seconds <= 120.:
            raise ValueError('等待提示时间须在 2–120 秒')
        if self.give_up_seconds != 0. and not 10. <= self.give_up_seconds <= 600.:
            raise ValueError('等待上限须为 0（一直等）或 10–600 秒')
        if not math.radians(30.) <= self.max_turn <= math.radians(85.):
            raise ValueError('绕行转向上限须在 30–85°')

    @classmethod
    def from_config(cls, values):
        """Build from ``config.avoid_settings`` (already range-checked there)."""
        values = dict(values or {})
        return cls(enabled=bool(values.get('enabled', True)),
                   reroute_after=float(values.get('reroute_after', 1.0)),
                   wait_seconds=float(values.get('wait_seconds', 10.)),
                   give_up_seconds=float(values.get('give_up_seconds', 0.)),
                   max_turn=float(values.get('max_turn', math.radians(75.))))


@dataclass(frozen=True)
class Decision:
    action: str            # hold | reroute | recovered | escalate | stop
    reason: str = ''


class AvoidancePolicy:
    """idle → blocked → rerouting →（idle 或 waiting）→ stop（仅在显式设上限时）."""

    def __init__(self, settings=None):
        self.settings = settings or AvoidanceSettings()
        self.reset()

    @property
    def state(self):
        return self._state

    def reset(self):
        self._state = 'idle'
        self._blocked_at = 0.
        self._wait_at = 0.
        self._escalated = False
        self.reason = ''

    def update(self, now, *, phase=None, avoid=None):
        """One status sample; ``phase`` is the car goal_state."""
        avoid = avoid or {}
        blocked = bool(avoid.get('blocked')) or phase == 'blocked'
        if not self.settings.enabled:
            return Decision('hold')
        if self._state == 'stopped':
            return Decision('stop', self.reason)
        if not blocked:
            if self._state != 'idle':
                self.reset()
                return Decision('recovered', '障碍已让开，继续沿路网行驶。')
            return Decision('hold')
        if self._state == 'idle':
            self._state, self._blocked_at = 'blocked', now
            distance = avoid.get('distance')
            hint = ('前方 %.2f 米有障碍，已停车等待。' % distance
                    if isinstance(distance, (int, float)) else '前方有障碍，已停车等待。')
            return Decision('hold', hint)
        if self._state == 'blocked':
            if now - self._blocked_at >= self.settings.reroute_after:
                self._state = 'rerouting'
                return Decision('reroute', '前方受阻，尝试从当前位置重新规划绕行。')
            return Decision('hold')
        if self._state == 'waiting':
            elapsed = now - self._wait_at
            if self.settings.give_up_seconds > 0 and elapsed >= self.settings.give_up_seconds:
                self._state = 'stopped'
                self.reason = '障碍长时间未移开，且没有可绕行的道路；已停车，请人工处理后再点开始导航。'
                return Decision('stop', self.reason)
            if not self._escalated and elapsed >= self.settings.wait_seconds:
                self._escalated = True
                return Decision('escalate',
                                '障碍还没移开；车已停稳等待，让开后会自动继续，'
                                '也可以点“■ 停止导航”结束本次导航。')
            return Decision('hold')
        return Decision('hold')

    def rerouted(self, ok, now):
        """Report the outcome of one reroute attempt."""
        if self._state != 'rerouting':
            return
        if ok:
            self.reset()
        else:
            self._state, self._wait_at, self._escalated = 'waiting', now, False
