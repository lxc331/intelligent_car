"""Local colour tracking on the latest camera frame. Pillow is optional until used."""
import io
import math
import queue
import threading
import time

COLORS = {"红色": "red", "绿色": "green", "蓝色": "blue", "黄色": "yellow"}


def detect_color(payload, color="red", previous=None, min_area=24):
    from PIL import Image, ImageChops, ImageFilter
    image = Image.open(io.BytesIO(payload)).convert("RGB")
    image.thumbnail((320, 240))
    width, height = image.size
    h, s, v = image.convert("HSV").split()
    bands = {"red": lambda n: n <= 15 or n >= 240,
             "green": lambda n: 65 <= n <= 125,
             "blue": lambda n: 135 <= n <= 185,
             "yellow": lambda n: 25 <= n <= 50}
    if color not in bands:
        raise ValueError("不支持的目标颜色")
    mask = h.point([255 if bands[color](n) else 0 for n in range(256)])
    mask = ImageChops.multiply(mask, s.point([255 if n >= 105 else 0 for n in range(256)]))
    mask = ImageChops.multiply(mask, v.point([255 if n >= 55 else 0 for n in range(256)]))
    pixels = bytearray(mask.filter(ImageFilter.MedianFilter(3)).tobytes())
    candidates = []
    for start in range(len(pixels)):
        if not pixels[start]:
            continue
        pixels[start] = 0
        stack, area, sx, sy = [start], 0, 0, 0
        left, right, top, bottom = width, 0, height, 0
        while stack:
            pos = stack.pop()
            y, x = divmod(pos, width)
            area += 1
            sx += x
            sy += y
            left, right = min(left, x), max(right, x)
            top, bottom = min(top, y), max(bottom, y)
            neighbours = []
            if x:
                neighbours.append(pos-1)
            if x+1 < width:
                neighbours.append(pos+1)
            if y:
                neighbours.append(pos-width)
            if y+1 < height:
                neighbours.append(pos+width)
            for p in neighbours:
                if pixels[p]:
                    pixels[p] = 0
                    stack.append(p)
        if not min_area <= area <= width*height*.35 or right-left < 3 or bottom-top < 3:
            continue
        candidates.append({"x": sx/area/width, "y": sy/area/height,
                           "width": (right-left+1)/width, "height": (bottom-top+1)/height,
                           "area": area/(width*height),
                           "bbox": [left/width, top/height, (right+1)/width, (bottom+1)/height]})
    if not candidates:
        return None
    if previous:
        candidates = [c for c in candidates if math.hypot(c["x"]-previous["x"], c["y"]-previous["y"]) < .20
                      and .2 <= c["area"]/max(previous["area"], .00001) <= 5]
        if not candidates:
            return None
        return min(candidates, key=lambda c: math.hypot(c["x"]-previous["x"], c["y"]-previous["y"]))
    return max(candidates, key=lambda c: c["area"])


class ColorTracker(threading.Thread):
    def __init__(self, color="red"):
        super().__init__(daemon=True, name="target-tracker")
        self.color = color
        self.frames = queue.Queue(maxsize=1)
        self.halt = threading.Event()
        self.lock = threading.Lock()
        self.generation = 0
        self.last = {"frame_id": 0, "seen_at": 0, "target": None, "stable": 0}
        self.last_payload = None

    def reset(self, color=None):
        with self.lock:
            if color:
                self.color = color
            self.generation += 1
            self.last = {"frame_id": 0, "seen_at": 0, "target": None, "stable": 0}
            self.last_payload = None

    def submit(self, payload):
        try:
            self.frames.get_nowait()
        except queue.Empty:
            pass
        with self.lock:
            item = (time.monotonic(), payload, self.color, self.generation)
        try:
            self.frames.put_nowait(item)
        except queue.Full:
            pass

    def snapshot(self):
        with self.lock:
            return dict(self.last)

    def run(self):
        while not self.halt.is_set():
            try:
                seen_at, payload, color, generation = self.frames.get(timeout=.2)
            except queue.Empty:
                continue
            with self.lock:
                before = dict(self.last)
            # Do not jump to another same-colour object after losing this one.
            try:
                target = detect_color(payload, color, before.get("locked_target"))
                error = None
            except Exception as exc:
                target, error = None, str(exc)
            with self.lock:
                if generation != self.generation:
                    continue
                self.last_payload = payload
                self.last = {"frame_id": before["frame_id"]+1, "seen_at": seen_at,
                             "target": target, "error": error,
                             "locked_target": target or before.get("locked_target"),
                             "stable": before["stable"]+1 if target else 0}

    def stop(self):
        self.halt.set()
