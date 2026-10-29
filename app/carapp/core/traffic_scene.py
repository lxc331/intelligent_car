"""Traffic context adapted from the supplied self_driving.py, without steering.

Order of evidence is red first, junction second: a red light is tracked on its
own (area gate, confirmation frames, short hold), and the vehicle is only held
when that red light is seen together with a confirmed crosswalk ahead.  The
original 640x480 thresholds are kept: crosswalk centre y>300 confirmed three
times, red area>700 (configurable), slow speed .1m/s, passage length .4m.
Right/park signs annotate approaching junctions; they never issue turns,
parking moves or forward commands here.
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

    def __init__(self, red_area_min=700., red_frames=1, red_hold=0.,
                 green_area_min=300., release_green_frames=0, release_timeout=12.):
        red_area_min = float(red_area_min)
        if not math.isfinite(red_area_min) or red_area_min <= 0.:
            raise ValueError("红灯面积门限无效")
        if isinstance(red_frames, bool) or not isinstance(red_frames, int) or not 1 <= red_frames <= 5:
            raise ValueError("红灯确认帧数无效")
        red_hold = float(red_hold)
        if not math.isfinite(red_hold) or red_hold < 0.:
            raise ValueError("红灯保持时间无效")
        green_area_min = float(green_area_min)
        if not math.isfinite(green_area_min) or green_area_min <= 0.:
            raise ValueError("绿灯面积门限无效")
        if (isinstance(release_green_frames, bool) or not isinstance(release_green_frames, int)
                or not 0 <= release_green_frames <= 5):
            raise ValueError("绿灯确认帧数无效")
        release_timeout = float(release_timeout)
        if not math.isfinite(release_timeout) or release_timeout < 0.:
            raise ValueError("绿灯确认超时无效")
        self.red_area_min = red_area_min
        self.red_frames = red_frames
        self.red_hold = red_hold
        self.green_area_min = green_area_min
        # 0 = 红灯消失即放行（当前默认）；>0 = 必须再看到绿灯确认这么多次才放行。
        # 默认关是因为实车仰视 1380 时右侧竖直绿灯目前召回 0，开了会一直停着。
        self.release_green_frames = release_green_frames
        self.release_timeout = release_timeout
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
        self.red_pending = False
        self.red_visible = False
        self.red_area = 0.
        self.red_latched = False
        self.red_streak = 0
        self.red_seen_at = None
        self.red_cleared_at = None
        self.waiting_green = False
        self.green_pending = False
        self.green_visible = False
        self.green_area = 0.
        self.green_streak = 0

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
        crosswalk_y, red_area, park, right, go, green_area = 0., 0., False, False, False, 0.
        for obj in objects:
            x, y, w, h = obj.box
            cx, cy = (x + w / 2) / width, (y + h / 2) / height
            if not .15 < cx < .85:
                continue
            area = w * h * 640. * 480. / (width * height)
            if obj.label == "crosswalk":
                crosswalk_y = max(crosswalk_y, cy * 480.)
            elif obj.label == "red" and .02 < cy < .95:
                red_area = max(red_area, area)
            elif obj.label == "green" and .02 < cy < .95:
                green_area = max(green_area, area)
            elif obj.label == "park" and area > 150.:
                park = True
            elif obj.label == "right" and area > 200.:
                right = True
            elif obj.label == "go":
                go = True
        self.crosswalk_y = crosswalk_y
        self.red_area, self.red_visible = red_area, red_area > 0.
        self.green_area, self.green_visible = green_area, green_area > 0.
        green_sighting = green_area > self.green_area_min
        self.green_streak = self.green_streak + 1 if green_sighting else 0
        self.green_pending = bool(self.release_green_frames and
                                  self.green_streak >= self.release_green_frames)
        self.early_park = park and crosswalk_y > 160.
        self.right_pending = self.right_pending or right
        if not self.active:
            self.confirmed = self.confirmed + 1 if crosswalk_y > 300. else 0
            if self.confirmed >= 3:
                self.active, self.remaining, self.confirmed = True, self.PASS_SECONDS, 0.
                elapsed = 0.
        # The red light is its own evidence: it is tracked whether or not the
        # crosswalk has been confirmed yet, so a car already parked on the stop
        # line (no crosswalk in view) still holds for a red light.
        sighting = red_area > self.red_area_min
        self.red_streak = self.red_streak + 1 if sighting else 0
        if sighting:
            self.red_seen_at = received_at
        # Enter red only after N sightings; keep it through short detection
        # gaps (LED flicker / exposure) for red_hold seconds.
        held = self.red_seen_at is not None and received_at - self.red_seen_at <= self.red_hold
        red = bool(self.red_streak >= self.red_frames or
                   self.red_latched and (red_area > 0. or held))
        # 可选的「绿灯确认才放行」：红灯从画面消失后先按住，等 green 确认帧数达到
        # release_green_frames 才真正解除；超时 release_timeout 后兜底放行，
        # 避免绿灯漏检把车永久钉在停止线前。
        self.waiting_green = False
        if red:
            self.red_latched, self.red_cleared_at = True, None
        elif self.red_latched:
            if self.release_green_frames and not self.green_pending:
                if self.red_cleared_at is None:
                    self.red_cleared_at = received_at
                if received_at - self.red_cleared_at < self.release_timeout:
                    red, self.waiting_green = True, True
            if not red:
                self.red_latched, self.red_cleared_at = False, None
        self.red_pending = bool(red)
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
                self.red_latched, self.red_seen_at, self.red_streak = False, None, 0
                self.red_pending = False
                self.red_cleared_at, self.waiting_green = None, False
        self.red_required = bool(self.active and red)
        return self.red_required

    def snapshot(self):
        slowing = self.active or self.early_park
        if self.waiting_green:
            reason = "红灯已消失，等待绿灯确认"
        elif self.red_required:
            reason = "红灯确认，路口停车"
        elif self.active:
            reason = (self.route or "斑马线路口") + "减速"
            if self.red_visible and not self.red_pending:
                reason = "红灯目标较小，路口减速"
        elif self.early_park:
            reason = "停车标志路口减速"
        elif self.red_pending:
            reason = "前方红灯，确认斑马线"
        elif self.confirmed:
            reason = "确认斑马线 %d/3" % self.confirmed
        else:
            reason = "交通场景观察中"
        return dict(scene=reason, speed_limit=self.SLOW_SPEED if slowing else None,
                    crosswalk_active=self.active, crosswalk_confirmed=self.confirmed,
                    crosswalk_y=self.crosswalk_y, passage_remaining=self.remaining,
                    red_pending=self.red_pending,
                    red_area_min=self.red_area_min, red_frames=self.red_frames, red_hold=self.red_hold,
                    green_pending=self.green_pending, green_visible=self.green_visible,
                    green_area=self.green_area, green_area_min=self.green_area_min,
                    release_green_frames=self.release_green_frames,
                    waiting_green=self.waiting_green,
                    scene_red_visible=self.red_visible, scene_red_area=self.red_area)
