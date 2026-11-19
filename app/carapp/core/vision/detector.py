"""Generic YOLOv8 frame detection: tiled inference and duplicate merging.

Deliberately independent of every feature. It knows nothing about traffic
lights, lanes or obstacles: it returns raw objects and the caller decides what
they mean. That keeps the module reusable and testable on its own.

The optional tiling rule is generic as well: a caller may pass normalized
regions that are cropped, upscaled by the network and mapped back, which keeps
small distant targets visible after the model's own letterbox.
"""
from __future__ import annotations

import io
import math
import threading
from dataclasses import dataclass
from pathlib import Path

from carapp.core.vision.backend import OFFICIAL_WEIGHTS, UltralyticsBackend

APP_DIR = Path(__file__).resolve().parents[3]
MODEL_DIR = APP_DIR / "models" / "vision"
DEFAULT_MODEL = MODEL_DIR / "yolov8n.pt"

MAX_TILES = 8


@dataclass(frozen=True)
class VisionObject:
    label: str
    score: float
    box: tuple  # x, y, w, h in source-image pixels


@dataclass(frozen=True)
class VisionDetection:
    """Detections for one frame, in source-image pixels."""

    objects: tuple = ()
    image_size: tuple = ()

    def labels(self):
        return {item.label for item in self.objects}

    def find(self, label):
        return tuple(item for item in self.objects if item.label == label)


def parse_tiles(regions):
    """Validate [(x, y, w, h), ...] normalized regions; None/() disables tiling."""
    if regions is None or regions == () or regions == []:
        return ()
    if not isinstance(regions, (list, tuple)):
        raise ValueError("放大检测区域应为矩形列表")
    if len(regions) > MAX_TILES:
        raise ValueError("放大检测区域最多 %d 块" % MAX_TILES)
    parsed = []
    for region in regions:
        try:
            values = [float(value) for value in region]
        except (TypeError, ValueError) as exc:
            raise ValueError("放大检测区域必须是数值") from exc
        if len(values) != 4 or not all(math.isfinite(value) for value in values):
            raise ValueError("放大检测区域应为 4 个有限数值")
        x, y, w, h = values
        if w <= .02 or h <= .02 or x < -1e-6 or y < -1e-6 or x + w > 1. + 1e-6 or y + h > 1. + 1e-6:
            raise ValueError("放大检测区域超出画面：%s" % (tuple(values),))
        parsed.append((x, y, w, h))
    return tuple(parsed)


def _iou(first, second):
    ax, ay, aw, ah = first
    bx, by, bw, bh = second
    x1, y1 = max(ax, bx), max(ay, by)
    x2, y2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    if x2 <= x1 or y2 <= y1:
        return 0.
    inter = (x2 - x1) * (y2 - y1)
    return inter / float(aw * ah + bw * bh - inter)


class VisionDetector:
    """One YOLOv8 model, one frame in, plain objects out."""

    def __init__(self, backend=None, *, model_path=None, device="auto", imgsz=640,
                 confidence=.25, tiles=(), tile_confidence=None, merge_iou=.45,
                 infer=None, verbose=False):
        confidence = float(confidence)
        if not math.isfinite(confidence) or not 0. < confidence <= 1.:
            raise ValueError("视觉置信度应在 0–1 之间")
        tile_confidence = confidence if tile_confidence is None else float(tile_confidence)
        if not math.isfinite(tile_confidence) or not 0. < tile_confidence <= 1.:
            raise ValueError("放大区域置信度应在 0–1 之间")
        if not math.isfinite(float(merge_iou)) or not 0. <= float(merge_iou) <= 1.:
            raise ValueError("重叠阈值应在 0–1 之间")
        self.confidence = confidence
        self.tile_confidence = tile_confidence
        self.tiles = parse_tiles(tiles)
        self.merge_iou = float(merge_iou)
        self._infer = infer
        self._backend = None if infer is not None else (
            backend or UltralyticsBackend(model_path or DEFAULT_MODEL, device=device,
                                          imgsz=imgsz, verbose=verbose))
        self._lock = threading.Lock()

    # ---------------- 构造 ----------------
    @classmethod
    def from_config(cls, settings, **overrides):
        """Build from the ``vision`` settings block; unknown keys are ignored."""
        settings = settings or {}
        model = settings.get("model") or str(DEFAULT_MODEL)
        path = Path(model)
        if not path.is_absolute():
            path = APP_DIR / path
        options = dict(
            model_path=path,
            device=settings.get("device", "auto"),
            imgsz=int(settings.get("imgsz", 640)),
            confidence=settings.get("confidence", .25),
            tiles=settings.get("tiles") or (),
            tile_confidence=settings.get("tile_confidence"),
            merge_iou=settings.get("merge_iou", .45),
            verbose=bool(settings.get("verbose", False)),
        )
        options.update(overrides)
        return cls(**options)

    # ---------------- 可用性 ----------------
    def ensure_available(self):
        """Fast check for callers that enable a feature; never loads the model."""
        if self._infer is not None:
            return
        from importlib.util import find_spec
        for name in ("torch", "ultralytics"):
            if find_spec(name) is None:
                raise ImportError(
                    "视觉推理缺少依赖 %s；请先执行 pip install -r app/requirements.txt" % name)
        path = self._backend.model_path
        if not path.is_file() and path.name not in OFFICIAL_WEIGHTS:
            raise FileNotFoundError("视觉权重缺失：%s" % path)

    def describe(self):
        return "注入推理" if self._infer is not None else self._backend.describe()

    def class_names(self):
        """Model class table as {id: name}; empty when inference is injected."""
        return {} if self._backend is None else self._backend.class_names()

    # ---------------- 推理 ----------------
    def _decode(self, payload, mode):
        from PIL import Image, UnidentifiedImageError
        if mode not in ("jpeg", "jpg", "png"):
            raise ValueError("视觉识别不支持该图像格式")
        try:
            with Image.open(io.BytesIO(payload)) as source:
                if source.width < 8 or source.height < 8:
                    raise ValueError("视觉识别画面尺寸过小")
                return source.convert("RGB")
        except (UnidentifiedImageError, OSError, TypeError, Image.DecompressionBombError) as exc:
            raise ValueError("视觉识别无法解码画面") from exc

    def _run(self, image, confidence):
        if self._infer is not None:
            return self._infer(image, confidence)
        return self._backend.infer(image, confidence)

    def _objects_in(self, image, region, confidence):
        """Infer one (cropped) region; return boxes in source-frame pixels."""
        width, height = image.size
        left, top = round(region[0] * width), round(region[1] * height)
        right = round((region[0] + region[2]) * width)
        bottom = round((region[1] + region[3]) * height)
        crop = image if (left, top, right, bottom) == (0, 0, width, height) else \
            image.crop((left, top, right, bottom))
        if crop.width < 8 or crop.height < 8:
            return []
        found = []
        for x1, y1, x2, y2, score, label in self._run(crop, confidence):
            x1, x2 = [max(0., min(width, float(value) + left)) for value in (x1, x2)]
            y1, y2 = [max(0., min(height, float(value) + top)) for value in (y1, y2)]
            if x2 <= x1 or y2 <= y1:
                continue
            x, y = round(x1), round(y1)
            w, h = round(x2) - x, round(y2) - y
            if w > 0 and h > 0:
                found.append(VisionObject(str(label), float(score), (x, y, w, h)))
        return found

    def _merge(self, objects):
        """Drop duplicate boxes of the same label across passes (keep best score)."""
        kept = []
        for item in sorted(objects, key=lambda value: value.score, reverse=True):
            if any(item.label == other.label and _iou(item.box, other.box) > self.merge_iou
                   for other in kept):
                continue
            kept.append(item)
        return kept

    def detect(self, payload, mode="jpeg"):
        image = self._decode(payload, mode)
        with self._lock:
            objects = self._objects_in(image, (0., 0., 1., 1.), self.confidence)
            if self.tiles:
                extra = []
                for region in self.tiles:
                    extra.extend(self._objects_in(image, region, self.tile_confidence))
                objects = self._merge(objects + extra)
        return VisionDetection(tuple(objects), image.size)
