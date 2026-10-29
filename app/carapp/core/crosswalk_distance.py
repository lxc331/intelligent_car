"""Ground-plane distance from a crosswalk box, for the stop-line approach.

The camera is mounted at a fixed height ``height`` and pitched down by ``pitch``
degrees.  For a pixel row ``y`` the depression angle of the ray below the
horizon is

    alpha = pitch + atan((y - center_y) / focal_px)

and the flat-ground distance of that ray is ``height / tan(alpha)``.  The two
camera constants (``height``, ``pitch``) are calibrated on the car by standing
still at known distances; ``calibrate`` solves them from a handful of
(row, distance) samples with a least-squares fit so the model also tolerates a
slightly wrong focal length.

Everything here is pure geometry: no ROS, no Tk, no I/O.
"""
import math

MIN_DISTANCE = .05
MAX_DISTANCE = 30.


def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("%s必须是有限数值" % name)
    return value


class CrosswalkDistance:
    """Map a crosswalk box to metres ahead of the camera."""

    def __init__(self, height=.30, pitch=35., focal_px=480., center_y=240.):
        self.height = _finite(height, "相机安装高度")
        self.pitch = _finite(pitch, "相机俯角")
        self.focal_px = _finite(focal_px, "相机焦距")
        self.center_y = _finite(center_y, "画面主点")
        if not 0. < self.height <= 2.:
            raise ValueError("相机安装高度无效：%s" % self.height)
        if not 0. <= self.pitch < 89.:
            raise ValueError("相机俯角无效：%s" % self.pitch)
        if not 10. <= self.focal_px <= 5000.:
            raise ValueError("相机焦距无效：%s" % self.focal_px)

    def _slope(self, row):
        return (float(row) - self.center_y) / self.focal_px

    def angle(self, row):
        """Depression angle in degrees for a pixel row below the principal point."""
        return self.pitch + math.degrees(math.atan(self._slope(row)))

    def distance(self, row):
        """Ground distance in metres for the pixel row of the box's near edge."""
        angle = math.radians(self.angle(row))
        if angle <= 0.:
            raise ValueError("该像素行在地平线以上，无法测距：%s" % row)
        return self.height / math.tan(angle)

    def from_box(self, box, *, edge="bottom"):
        """Distance for a ``(x, y, w, h)`` box; the near edge is the bottom row."""
        if len(box) != 4:
            raise ValueError("斑马线框需要 x/y/w/h 四个数值")
        x, y, w, h = (float(value) for value in box)
        row = y + h if edge == "bottom" else y + h / 2.
        distance = self.distance(row)
        if not MIN_DISTANCE <= distance <= MAX_DISTANCE:
            raise ValueError("斑马线距离超出可用范围：%.2f m" % distance)
        return distance

    def calibrated(self, samples):
        """Return a copy fitted to ``samples`` = ((row, distance), ...)."""
        height, pitch = self.fit(samples, self.focal_px, self.center_y)
        return CrosswalkDistance(height, pitch, self.focal_px, self.center_y)

    @staticmethod
    def fit(samples, focal_px=480., center_y=240.):
        """Least-squares fit of (height, pitch) from (row, distance) samples."""
        pairs = []
        for row, distance in samples:
            distance = _finite(distance, "标定距离")
            if distance <= 0.:
                raise ValueError("标定距离必须大于 0：%s" % distance)
            pairs.append((_finite(row, "标定像素行"), distance))
        if len(pairs) < 2:
            raise ValueError("标定至少需要 2 个样本")
        # 1/d = tan(alpha)/h with alpha = pitch + atan((y-cy)/f).
        # Writing t = tan(pitch) and x = (y-cy)/f:
        #   tan(alpha) = (t + x) / (1 - t*x)   ->   h*u*(1 - t*x) = t + x
        #   h*u - h*t*u*x - t = x
        # Linear in (h, h*t, t): p*u - q*u*x - t = x
        rows = []
        values = []
        for y, distance in pairs:
            x = (y - center_y) / focal_px
            u = 1. / distance
            rows.append([u, -u * x, -1.])
            values.append(x)
        try:
            import numpy as np
        except ImportError:  # pragma: no cover - numpy is a hard requirement
            raise RuntimeError("标定需要 numpy")
        matrix = np.array(rows, dtype=float)
        vector = np.array(values, dtype=float)
        solution, residuals, rank, _ = np.linalg.lstsq(matrix, vector, rcond=None)
        height, _height_times_t, tan_pitch = solution
        if not (math.isfinite(height) and math.isfinite(tan_pitch)) or height <= 0.:
            raise ValueError("标定结果无效，请检查样本")
        pitch = math.degrees(math.atan(tan_pitch))
        if not 0. <= pitch < 89.:
            raise ValueError("标定出的俯角无效：%.1f" % pitch)
        return float(height), float(pitch)

    def as_dict(self):
        return {"height": self.height, "pitch": self.pitch,
                "focal_px": self.focal_px, "center_y": self.center_y}

    def text(self):
        return "height=%.3f m pitch=%.1f° focal=%.0f px center_y=%.0f" % (
            self.height, self.pitch, self.focal_px, self.center_y)
