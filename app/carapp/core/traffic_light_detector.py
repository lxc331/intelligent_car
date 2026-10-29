"""Traffic-model adapter for the supplied YOLO26/OpenVINO export.

Reuses the model and class/confidence/xyxy interpretation from the supplied
example/example/yolo_detect/yolo_node.py. ROS transport and autonomous route
logic stay outside this module. No colour/shape fallback is used on failure.
"""
from dataclasses import dataclass
from importlib.util import find_spec
import io
import math
from pathlib import Path
import threading


MODEL_DIR = Path(__file__).resolve().parents[2] / "models" / "traffic_light"

# Fractional (x, y, w, h) regions of the source frame that are cropped and
# scaled up to the model input before inference. A 640x480 lamp is only ~16-30 px
# wide, so a full-frame pass letterboxes it down to ~8 px and the model misses
# it; cropping the lamp's band into overlapping tiles keeps its true size and
# avoids splitting a lamp that sits on a tile boundary. The model is sensitive to
# framing (the same lamp scores 0.8 in one tile and 0.05 in another), so the App
# supplies tiles for three distances: top band while approaching, middle band at
# 1-2 m and bottom band while parked under the light. The detector keeps them
# opt-in so existing callers and tests are unchanged.
MAX_ZOOM_REGIONS = 8


def parse_zoom_regions(regions):
    """Validate [(x, y, w, h), ...] normalized regions; None/() disables zoom."""
    if regions is None or regions == () or regions == []:
        return ()
    if not isinstance(regions, (list, tuple)):
        raise ValueError("红绿灯放大检测区域应为矩形列表")
    if len(regions) > MAX_ZOOM_REGIONS:
        raise ValueError("红绿灯放大检测区域最多 %d 块" % MAX_ZOOM_REGIONS)
    parsed = []
    for region in regions:
        try:
            values = [float(value) for value in region]
        except (TypeError, ValueError) as exc:
            raise ValueError("红绿灯放大检测区域必须是数值") from exc
        if len(values) != 4 or not all(math.isfinite(value) for value in values):
            raise ValueError("红绿灯放大检测区域应为 4 个有限数值")
        x, y, w, h = values
        if w <= .02 or h <= .02 or x < -1e-6 or y < -1e-6 or x + w > 1. + 1e-6 or y + h > 1. + 1e-6:
            raise ValueError("红绿灯放大检测区域超出画面：%s" % (tuple(values),))
        parsed.append((x, y, w, h))
    return tuple(parsed)


def parse_red_aspect(value):
    """Validate the red-lamp height/width window; None/() disables the filter.

    A round lens is roughly square in the image (measured 0.88-1.05 across ten
    real frames) while a red-white cone or a red roof is taller or wider, so a
    generous window removes obvious false positives without touching lamps.
    """
    if value is None or value == () or value == []:
        return None
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError("红灯形状比例应为上下限两个数值")
    try:
        low, high = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError("红灯形状比例应为数值") from exc
    if not (math.isfinite(low) and math.isfinite(high)) or not 0. < low < high:
        raise ValueError("红灯形状比例无效：%s" % (tuple(value),))
    return low, high


@dataclass(frozen=True)
class TrafficObject:
    label: str
    score: float
    box: tuple


@dataclass(frozen=True)
class TrafficLightDetection:
    red: bool
    reason: str
    # Frontal red-light boxes in source-image pixels: x, y, width, height.
    boxes: tuple = ()
    scores: tuple = ()
    objects: tuple = ()
    image_size: tuple = ()


class TrafficLightDetector:
    """Infer the full frame, then each zoom region, and keep frontal red boxes.

    The model emits end-to-end xyxy/confidence/class_id rows; its red class
    replaces the old RGB/circle heuristic. Only box centres within x15–85%,
    y2–95% of the source frame trigger the existing stop interlock; the lower
    limit allows the lifted camera to still see the lamp near the bottom edge.
    Zoom regions are cropped, letterboxed to 320 and mapped back to the frame;
    duplicate boxes across passes are merged by label + IoU. Zoom boxes that
    only reach the low candidate threshold are re-checked once in a square
    context crop, because the model is sensitive to how the lamp is framed.
    Imports, compilation and inference run lazily in the detection worker.
    """

    def __init__(self, model_dir=None, confidence=.5, *, infer=None, zoom=None, zoom_confidence=None,
                 red_aspect=None, candidate_confidence=None):
        confidence = float(confidence)
        if not math.isfinite(confidence) or not 0. < confidence <= 1.:
            raise ValueError("红灯模型置信度应在 0–1 之间")
        zoom_confidence = confidence if zoom_confidence is None else float(zoom_confidence)
        if not math.isfinite(zoom_confidence) or not 0. < zoom_confidence <= 1.:
            raise ValueError("红灯放大区域置信度应在 0–1 之间")
        candidate_confidence = (zoom_confidence if candidate_confidence is None
                                else float(candidate_confidence))
        if not math.isfinite(candidate_confidence) or not 0. < candidate_confidence <= zoom_confidence:
            raise ValueError("红灯候选置信度应在 0 到放大区域置信度之间")
        self.model_dir = Path(model_dir) if model_dir is not None else MODEL_DIR
        self.confidence = confidence
        self.zoom_confidence = zoom_confidence
        self.candidate_confidence = candidate_confidence
        self.zoom = parse_zoom_regions(zoom)
        self.red_aspect = parse_red_aspect(red_aspect)
        self._infer = infer
        self._lock = threading.Lock()
        self._request = None
        self._metadata = None

    def ensure_available(self):
        """Fast enable-time check; never compile a model on the UI thread."""
        required = ("PIL", "numpy", "yaml") + (() if self._infer else ("openvino",))
        missing = [name for name in required if find_spec(name) is None]
        if missing:
            raise ImportError("红灯模型缺少依赖 %s；请在 App 的 Python 环境安装 app/requirements.txt" % ", ".join(missing))
        for name in ("best_traffic_26.xml", "best_traffic_26.bin", "metadata.yaml"):
            if not (self.model_dir / name).is_file():
                raise FileNotFoundError("红灯模型文件缺失：%s" % (self.model_dir / name))

    def _load_metadata(self):
        if self._metadata is None:
            import yaml
            with (self.model_dir / "metadata.yaml").open(encoding="utf-8") as source:
                metadata = yaml.safe_load(source)
            if not isinstance(metadata, dict) or metadata.get("task") != "detect" or not metadata.get("end2end"):
                raise ValueError("红灯模型须为原 YOLO26 end-to-end 检测模型")
            names = metadata.get("names")
            if isinstance(names, list):
                names = dict(enumerate(names))
            if not isinstance(names, dict):
                raise ValueError("红灯模型类别表无效")
            names = {int(key): str(value) for key, value in names.items()}
            if "red" not in names.values() or "green" not in names.values():
                raise ValueError("模型缺少 red / green 交通灯类别")
            size = metadata.get("imgsz")
            if not isinstance(size, list) or size != [320, 320]:
                raise ValueError("红灯模型输入应为原 320×320")
            self._metadata = names, size[0]
        return self._metadata

    def _predict(self, tensor):
        if self._infer is not None:
            return self._infer(tensor)
        if self._request is None:
            import openvino as ov
            core = ov.Core()
            compiled = core.compile_model(str(self.model_dir / "best_traffic_26.xml"), "CPU", {
                "PERFORMANCE_HINT": "LATENCY", "INFERENCE_NUM_THREADS": 2, "NUM_STREAMS": "1",
            })
            if tuple(compiled.input(0).shape) != (1, 3, 320, 320) or tuple(compiled.output(0).shape) != (1, 300, 6):
                raise ValueError("红灯模型输入/输出与原模型不匹配")
            self._request = compiled.create_infer_request()
        result = self._request.infer({0: tensor})
        return next(iter(result.values()))

    def _objects_in(self, image, region, names, size, confidence):
        """Infer one (cropped) region; return boxes in source-frame pixels."""
        from PIL import Image
        import numpy as np

        width, height = image.size
        box = (round(region[0] * width), round(region[1] * height),
               round((region[0] + region[2]) * width), round((region[1] + region[3]) * height))
        crop = image if box == (0, 0, width, height) else image.crop(box)
        crop_width, crop_height = crop.size
        if crop_width < 8 or crop_height < 8:
            return []
        # Ultralytics letterbox: preserve aspect ratio, pad with 114,
        # feed RGB float32 NCHW in [0,1], never stretch to a square.
        gain = min(size / crop_width, size / crop_height)
        resized = (round(crop_width * gain), round(crop_height * gain))
        left = round((size - resized[0]) / 2 - .1)
        top = round((size - resized[1]) / 2 - .1)
        canvas = Image.new("RGB", (size, size), (114, 114, 114))
        canvas.paste(crop.resize(resized, Image.Resampling.BILINEAR), (left, top))
        tensor = np.ascontiguousarray(np.asarray(canvas).transpose(2, 0, 1)[None], dtype=np.float32)
        tensor /= 255.
        rows = np.asarray(self._predict(tensor))
        if rows.ndim != 3 or rows.shape[0] != 1 or rows.shape[2] != 6 or rows.shape[1] > 300:
            raise ValueError("红灯模型返回格式错误，应为 [1,N,6]")
        if not np.isfinite(rows).all():
            raise ValueError("红灯模型返回了无效数值")
        found = []
        for x1, y1, x2, y2, score, class_id in rows[0]:
            if not 0. <= score <= 1.:
                raise ValueError("红灯模型置信度无效")
            if score < confidence:
                continue
            label = names.get(int(class_id))
            if class_id != int(class_id) or label is None:
                raise ValueError("红灯模型返回了未知类别")
            x1, x2 = [max(0., min(width, (float(value) - left) / gain + box[0])) for value in (x1, x2)]
            y1, y2 = [max(0., min(height, (float(value) - top) / gain + box[1])) for value in (y1, y2)]
            if x2 <= x1 or y2 <= y1:
                continue
            x, y = round(x1), round(y1)
            w, h = round(x2) - x, round(y2) - y
            if w > 0 and h > 0:
                found.append(TrafficObject(label, float(score), (x, y, w, h)))
        return found

    @staticmethod
    def _merge(objects):
        """Drop duplicate boxes of the same label across passes (keep best score)."""
        def overlap(first, second):
            ax, ay, aw, ah = first
            bx, by, bw, bh = second
            x1, y1 = max(ax, bx), max(ay, by)
            x2, y2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
            if x2 <= x1 or y2 <= y1:
                return 0.
            inter = (x2 - x1) * (y2 - y1)
            return inter / float(aw * ah + bw * bh - inter)

        kept = []
        for item in sorted(objects, key=lambda value: value.score, reverse=True):
            if any(item.label == other.label and overlap(item.box, other.box) > .45 for other in kept):
                continue
            kept.append(item)
        return kept

    def _confirm(self, image, box, names, size):
        """Re-infer a square context crop around a weak candidate box.

        The zoom bands run at a low candidate threshold so a lamp that the band
        framing scores weakly still becomes a candidate; that box is then
        re-examined with generous context at the normal zoom confidence, which
        normalises the framing the model is sensitive to.
        """
        width, height = image.size
        x, y, w, h = box
        cx, cy = x + w / 2., y + h / 2.
        side = min(max(6. * max(w, h), 96.), float(min(width, height)))
        left = int(max(0., min(width - side, cx - side / 2.)))
        top = int(max(0., min(height - side, cy - side / 2.)))
        region = (left / width, top / height, side / width, side / height)
        found = []
        for item in self._objects_in(image, region, names, size, self.zoom_confidence):
            if item.label != "red":
                continue
            ox, oy, ow, oh = item.box
            reach = max(w, h)
            if abs(ox + ow / 2. - cx) <= reach and abs(oy + oh / 2. - cy) <= reach:
                found.append(item)
        return found

    def detect(self, payload, mode="jpeg"):
        from PIL import Image, UnidentifiedImageError

        if mode not in ("jpeg", "jpg", "png"):
            raise ValueError("红绿灯识别不支持该图像格式")
        try:
            with Image.open(io.BytesIO(payload)) as source:
                if source.width < 8 or source.height < 8:
                    raise ValueError("红绿灯识别画面尺寸过小")
                image = source.convert("RGB")
        except (UnidentifiedImageError, OSError, TypeError, Image.DecompressionBombError) as exc:
            raise ValueError("红绿灯识别无法解码画面") from exc

        with self._lock:
            names, size = self._load_metadata()
            width, height = image.size
            objects = self._objects_in(image, (0., 0., 1., 1.), names, size, self.confidence)
            if self.zoom:
                accepted, weak = [], []
                for region in self.zoom:
                    for item in self._objects_in(image, region, names, size, self.candidate_confidence):
                        if item.score >= self.zoom_confidence:
                            accepted.append(item)
                        elif item.label == "red":
                            weak.append(item)
                if weak:
                    weak.sort(key=lambda item: item.score, reverse=True)
                    confirmed = []
                    for item in weak[:3]:
                        confirmed.extend(self._confirm(image, item.box, names, size))
                    accepted.extend(confirmed)
                objects.extend(accepted)
                objects = self._merge(objects)
            boxes, scores = [], []
            for item in objects:
                if item.label != "red":
                    continue
                x, y, w, h = item.box
                if self.red_aspect is not None:
                    ratio = h / float(w)
                    if not self.red_aspect[0] <= ratio <= self.red_aspect[1]:
                        continue
                if (.15 < (x + w / 2.) / width < .85
                        and .02 < (y + h / 2.) / height < .95):
                    boxes.append(item.box)
                    scores.append(item.score)
        return TrafficLightDetection(bool(boxes), "模型识别到正前方红灯" if boxes else "模型未识别到正前方红灯",
                                     tuple(boxes), tuple(scores), tuple(objects), (width, height))
