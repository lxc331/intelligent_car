"""Traffic context adapted from the supplied self_driving.py, without steering.

Original 640x480 thresholds: crosswalk y>300 confirmed 3 times, red area>700,
slow speed .1m/s, passage length .4m. Right/park signs annotate approaching
junctions; they never issue turns, parking moves or forward commands here.
"""
import math


def limit_twist(vx, vy, wz, limit):
    """Limit translation without increasing speed or changing path curvature."""
    if limit is None:
        return vx, vy, wz
    if not math.isfinite(limit) or limit <= 0.:
        raise ValueError("交通场景速度限制无效")
    speed = math.hypot(vx, vy)
    ratio = min(1., limit / speed) if speed else 1.
    return vx * ratio, vy * ratio, wz * ratio


class TrafficScenePolicy:
    SLOW_SPEED = .1
    PASS_SECONDS = .4 / SLOW_SPEED
    FRAME_TIMEOUT = 1.2

    def __init__(self):
        self.reset()

    def reset(self):
        self.last_at = None
        self.confirmed = 0
        self.active = False
        self.remaining = 0.
        self.early_park = False
        self.right_pending = False
        self.route = ""
        self.crosswalk_y = 0.
        self.red_required = False
        self.red_visible = False
        self.red_area = 0.
        self.red_latched = False

    def observe(self, objects, image_size, received_at, *, paused=False):
        width, height = image_size
        if width <= 0 or height <= 0:
            raise ValueError("交通场景图像尺寸无效")
        if self.last_at is not None and received_at <= self.last_at:
            return self.red_required
        elapsed = 0. if self.last_at is None else received_at - self.last_at
        if elapsed > self.FRAME_TIMEOUT:
            self.reset()
            elapsed = 0.
        self.last_at = received_at
        crosswalk_y, red_area, park, right, go = 0., 0., False, False, False
        for obj in objects:
            x, y, w, h = obj.box
            cx, cy = (x + w / 2) / width, (y + h / 2) / height
            if not .15 < cx < .85:
                continue
            area = w * h * 640. * 480. / (width * height)
            if obj.label == "crosswalk":
                crosswalk_y = max(crosswalk_y, cy * 480.)
            elif obj.label == "red" and .02 < cy < .80:
                red_area = max(red_area, area)
            elif obj.label == "park" and area > 150.:
                park = True
            elif obj.label == "right" and area > 200.:
                right = True
            elif obj.label == "go":
                go = True
        self.crosswalk_y = crosswalk_y
        self.red_area, self.red_visible = red_area, red_area > 0.
        self.early_park = park and crosswalk_y > 160.
        self.right_pending = self.right_pending or right
        if not self.active:
            self.confirmed = self.confirmed + 1 if crosswalk_y > 300. else 0
            if self.confirmed >= 3:
                self.active, self.remaining, self.confirmed = True, self.PASS_SECONDS, 0
                elapsed = 0.
        red = self.active and (red_area > 700. or self.red_latched and red_area > 0.)
        if red:
            self.red_latched = True
        if self.active:
            if self.right_pending and crosswalk_y > 365.:
                self.route = "右转路口"
            elif go and not self.route:
                self.route = "直行路口"
            if not paused and not red:
                self.remaining = max(0., self.remaining - elapsed)
            if self.remaining <= 0.:
                self.active = False
                self.right_pending, self.route = False, ""
                self.red_latched = False
        self.red_required = bool(self.active and red)
        return self.red_required

    def snapshot(self):
        slowing = self.active or self.early_park
        reason = (("红灯目标较小，路口减速" if self.red_visible and self.red_area <= 700. and not self.red_latched
                   else (self.route or "斑马线路口") + "减速") if self.active else
                  "停车标志路口减速" if self.early_park else
                  "红灯未满足路口条件" if self.red_visible else
                  "确认斑马线 %d/3" % self.confirmed if self.confirmed else "交通场景观察中")
        return dict(scene=reason, speed_limit=self.SLOW_SPEED if slowing else None,
                    crosswalk_active=self.active, crosswalk_confirmed=self.confirmed,
                    crosswalk_y=self.crosswalk_y, passage_remaining=self.remaining,
                    scene_red_visible=self.red_visible, scene_red_area=self.red_area)
