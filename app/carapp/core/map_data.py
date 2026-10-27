"""Validated OccupancyGrid and planar TF state, independent of Tk and ROS installs.

All timestamps passed to this class use the PC monotonic clock. ROS timestamps are
not compared to the PC clock. A failed update preserves the previous valid map.
"""
import math
import threading
import time
from collections import deque


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("需要数值")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("数值不是有限值")
    return result


def _frame(value):
    if not isinstance(value, str):
        raise ValueError("坐标系名称无效")
    value = value.lstrip("/")
    if not value or len(value) > 128 or any(c.isspace() for c in value):
        raise ValueError("坐标系名称无效")
    return value


def yaw_from_quaternion(rotation):
    """Accept non-unit quaternions but reject zero/non-finite orientations."""
    x, y, z, w = (_number(rotation[k]) for k in ("x", "y", "z", "w"))
    norm = math.hypot(x, y, z, w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("四元数为零")
    x, y, z, w = (v / norm for v in (x, y, z, w))
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def compose_pose(parent, child):
    """Planar parent->middle composed with middle->child."""
    x, y, yaw = parent
    dx, dy, dyaw = child
    c, s = math.cos(yaw), math.sin(yaw)
    angle = yaw + dyaw
    return (x + c * dx - s * dy, y + s * dx + c * dy,
            math.atan2(math.sin(angle), math.cos(angle)))


def inverse_pose(pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return -c * x - s * y, s * x - c * y, -yaw


def world_to_grid(grid, x, y):
    """Map-frame metres to continuous cell coordinates (origin can be rotated)."""
    ox, oy, yaw = grid["origin"]
    dx, dy = x - ox, y - oy
    c, s = math.cos(yaw), math.sin(yaw)
    resolution = grid["resolution"]
    return (c * dx + s * dy) / resolution, (-s * dx + c * dy) / resolution


def _display_pixels(data, width, height):
    """Worker-side palette indices, top row first: unknown=0/free=1/wall=2.

    Preparing this immutable byte buffer in the map ingestion worker keeps a
    multi-million-cell Python loop out of the Tk input/stop event thread.
    """
    pixels = bytearray(len(data))
    for row in range(height):
        start = row * width
        target = (height - 1 - row) * width
        pixels[target:target + width] = bytes(
            0 if value < 0 else 2 if value >= 65 else 1
            for value in data[start:start + width])
    return bytes(pixels)


class MapModel:
    MAX_CELLS = 4_000_000
    MAX_SIDE = 8192
    MAX_TF = 512
    POSE_TIMEOUT = 2.0

    def __init__(self):
        self.lock = threading.RLock()
        self._revision = 0
        self._generation = 0
        self.reset()

    @property
    def generation(self):
        with self.lock:
            return self._generation

    def reset(self, *, keep_map=False):
        """Clear session localization; an imported static map may be retained."""
        with self.lock:
            self._generation += 1
            self._revision += 1
            if not keep_map:
                self._grid = None
            self._map_error = ""
            self._tf_error = ""
            self._odom_error = ""
            self._transforms = {}
            self._odom = None
            self._trajectory = deque(maxlen=2000)
            self._trajectory_frame = None

    def accept_map(self, msg, received_at, *, generation=None):
        # A reset or vehicle change invalidates queued/in-flight decoding work.
        with self.lock:
            epoch = self._generation if generation is None else generation
            if epoch != self._generation:
                return False
        try:
            received_at = _number(received_at)
            info = msg["info"]
            width, height = info["width"], info["height"]
            if (type(width) is not int or type(height) is not int or
                    not 1 <= width <= self.MAX_SIDE or not 1 <= height <= self.MAX_SIDE or
                    width * height > self.MAX_CELLS):
                raise ValueError("地图尺寸超出范围（最多 400 万单元）")
            resolution = _number(info["resolution"])
            if not 0.001 <= resolution <= 10:
                raise ValueError("地图分辨率超出范围")
            origin = info["origin"]
            position = origin["position"]
            ox, oy = _number(position["x"]), _number(position["y"])
            _number(position.get("z", 0.0))
            if abs(ox) > 1e6 or abs(oy) > 1e6:
                raise ValueError("地图原点超出范围")
            yaw = yaw_from_quaternion(origin["orientation"])
            frame_id = _frame(msg["header"]["frame_id"])
            data = msg["data"]
            if not isinstance(data, (list, tuple)) or len(data) != width * height:
                raise ValueError("地图单元数与宽高不一致")
            if any(type(v) is not int or not -1 <= v <= 100 for v in data):
                raise ValueError("地图占用值必须是 -1 到 100 的整数")
            # Only one immutable tuple is shared by snapshots and rendering.
            data = tuple(data)
            pixels = _display_pixels(data, width, height)
            grid = dict(width=width, height=height, resolution=resolution,
                        origin=(ox, oy, yaw), frame_id=frame_id, data=data, pixels=pixels,
                        received_at=received_at)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            with self.lock:
                if epoch == self._generation:
                    self._map_error = "地图数据无效：" + str(exc)
            return False
        with self.lock:
            if epoch != self._generation:
                return False
            self._revision += 1
            grid["revision"] = self._revision
            self._grid = grid
            self._map_error = ""
            if self._trajectory_frame != frame_id:
                self._trajectory.clear()
                self._trajectory_frame = frame_id
        return True

    def accept_tf(self, msg, received_at, static=False):
        """Consume /tf or /tf_static; pass static=True for the latter if known."""
        try:
            received_at = _number(received_at)
            transforms = msg["transforms"]
            if not isinstance(transforms, (list, tuple)) or len(transforms) > self.MAX_TF:
                raise ValueError("TF 数量超出范围")
            updates = {}
            for item in transforms:
                parent = _frame(item["header"]["frame_id"])
                child = _frame(item["child_frame_id"])
                if parent == child:
                    raise ValueError("TF 父子坐标系相同")
                transform = item["transform"]
                translation = transform["translation"]
                x, y = _number(translation["x"]), _number(translation["y"])
                _number(translation.get("z", 0.0))
                if abs(x) > 1e6 or abs(y) > 1e6:
                    raise ValueError("TF 位移超出范围")
                pose = (x, y, yaw_from_quaternion(transform["rotation"]))
                updates[parent, child] = (pose, received_at, bool(static))
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            with self.lock:
                self._tf_error = "TF 数据无效：" + str(exc)
            return False
        with self.lock:
            self._transforms.update(updates)
            if len(self._transforms) > self.MAX_TF:
                ordered = sorted(self._transforms, key=lambda k: self._transforms[k][1])
                for key in ordered[:len(self._transforms) - self.MAX_TF]:
                    del self._transforms[key]
            self._tf_error = ""
        return True

    def accept_odom(self, msg, received_at):
        try:
            received_at = _number(received_at)
            frame_id = _frame(msg["header"]["frame_id"])
            child = _frame(msg.get("child_frame_id") or "base_link")
            pose = msg["pose"]["pose"]
            position = pose["position"]
            x, y = _number(position["x"]), _number(position["y"])
            _number(position.get("z", 0.0))
            if abs(x) > 1e6 or abs(y) > 1e6:
                raise ValueError("里程计位置超出范围")
            odom = dict(frame_id=frame_id, child=child,
                        pose=(x, y, yaw_from_quaternion(pose["orientation"])),
                        received_at=received_at)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            with self.lock:
                self._odom_error = "里程计数据无效：" + str(exc)
            return False
        with self.lock:
            self._odom = odom
            self._odom_error = ""
        return True

    def _lookup(self, parent, child):
        if parent == child:
            return (0., 0., 0.), None
        graph = {}
        for (start, end), (pose, stamp, static) in self._transforms.items():
            stamp = None if static else stamp
            graph.setdefault(start, []).append((end, pose, stamp))
            graph.setdefault(end, []).append((start, inverse_pose(pose), stamp))
        queue = deque([(parent, (0., 0., 0.), None)])
        visited = {parent}
        while queue:
            current, pose, oldest = queue.popleft()
            for end, step, stamp in graph.get(current, ()):
                if end in visited:
                    continue
                visited.add(end)
                result = compose_pose(pose, step)
                age_stamp = stamp if oldest is None else oldest if stamp is None else min(oldest, stamp)
                if end == child:
                    return result, age_stamp
                queue.append((end, result, age_stamp))
        return None

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else _number(now)
        with self.lock:
            grid = dict(self._grid) if self._grid is not None else None
            pose = None
            pose_stamp = None
            pose_error = "等待里程计和地图定位"
            if grid and self._odom:
                odom = self._odom
                transform = self._lookup(grid["frame_id"], odom["frame_id"])
                if transform:
                    pose = compose_pose(transform[0], odom["pose"])
                    pose_stamp = odom["received_at"]
                    if transform[1] is not None:
                        pose_stamp = min(pose_stamp, transform[1])
                    pose_error = ""
                else:
                    pose_error = "等待 %s → %s 坐标变换" % (grid["frame_id"], odom["frame_id"])
            elif grid:
                for child in ("base_footprint", "base_link"):
                    transform = self._lookup(grid["frame_id"], child)
                    if transform:
                        pose, pose_stamp = transform
                        pose_error = ""
                        break
            pose_age = max(0., now - pose_stamp) if pose_stamp is not None else None
            stale = pose is not None and pose_age is not None and pose_age > self.POSE_TIMEOUT
            if stale:
                pose_error = "定位数据已过期（%.1f 秒）" % pose_age
            if pose is not None and not stale:
                point = pose[:2]
                if not self._trajectory or math.dist(point, self._trajectory[-1]) >= .02:
                    self._trajectory.append(point)
            return dict(grid=grid, pose=pose, error=self._map_error,
                        pose_error=self._odom_error or self._tf_error or pose_error,
                        map_age=max(0., now - grid["received_at"]) if grid else None,
                        pose_age=pose_age, pose_stale=stale,
                        trajectory=tuple(self._trajectory))
