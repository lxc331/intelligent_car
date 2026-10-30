"""ROS/Tk-free limits and progress supervision for longer visual approaches."""
import math


def approach_config(config, distance):
    distance = float(distance)
    if not math.isfinite(distance) or not .3 <= distance <= 3.:
        raise ValueError('最大追踪行程应在 0.3–3.0 米之间')
    result = dict(config)
    result['max_distance'] = distance
    result['max_seconds'] = max(float(config['max_seconds']), distance / max(.02, float(config['min_speed'])) + 30.)
    return result


class ApproachProgress:
    def __init__(self, timeout=12.):
        self.timeout = timeout
        self.moving = 0.
        self.last_at = None
        self.reference = None

    def update(self, target, vx, now):
        dt = 0. if self.last_at is None else max(0., min(.5, now-self.last_at))
        self.last_at = now
        if not target or vx <= 0:
            return
        if self.reference is None:
            self.reference = (target['y'], target['width'])
        y, width = self.reference
        if target['y'] > y+.012 or target['width'] > width*1.08:
            self.reference = (target['y'], target['width'])
            self.moving = 0.
        else:
            self.moving += dt
        if self.moving > self.timeout:
            raise RuntimeError('持续前进但目标未接近，已停车；请检查打滑、目标移动或识别错误')
