"""Ultralytics YOLOv8 backend: native PyTorch inference on GPU when available.

Imports of torch/ultralytics are deferred to the first inference so the rest of
the app keeps starting (and every other feature keeps working) on a machine
where the optional vision dependencies are not installed.
"""
from __future__ import annotations

import shutil
import threading
from pathlib import Path

# Ultralytics will download these by name; anything else must already be on disk.
OFFICIAL_WEIGHTS = frozenset(
    "yolov8%s.pt" % suffix for suffix in ("n", "s", "m", "l", "x")
)


class BackendUnavailable(RuntimeError):
    """Raised when torch/ultralytics or the weights cannot be used."""


class UltralyticsBackend:
    """One YOLO instance shared by every caller; inference is serialised.

    The model is loaded lazily and reused, so switching features on and off does
    not reload weights and the UI thread never pays the load cost.
    """

    def __init__(self, model_path, device="auto", imgsz=640, verbose=False):
        self.model_path = Path(model_path)
        self.device = device or "auto"
        self.imgsz = int(imgsz)
        self.verbose = bool(verbose)
        self._model = None
        self._names = {}
        self._device = None
        self._lock = threading.Lock()
        self._load_lock = threading.Lock()

    # ---------------- 加载 ----------------
    def _resolve_device(self):
        import torch
        if self.device not in ("", "auto", None):
            return self.device
        return "cuda:0" if torch.cuda.is_available() else "cpu"

    def _open(self, yolo):
        path = self.model_path
        if path.is_file():
            return yolo(str(path))
        if path.name in OFFICIAL_WEIGHTS:
            # Ultralytics downloads official weights next to the working
            # directory; move them into models/vision so the layout stays fixed.
            model = yolo(path.name)
            downloaded = Path(path.name)
            if downloaded.is_file() and not path.exists():
                path.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(downloaded), str(path))
            return model
        raise BackendUnavailable(
            "视觉权重缺失：%s（可放入官方 yolov8n.pt，或改 vision.model 指向已有权重）" % path)

    def load(self):
        with self._load_lock:
            if self._model is not None:
                return self._model
            try:
                from ultralytics import YOLO
            except ImportError as exc:
                raise BackendUnavailable(
                    "缺少推理依赖 %s；请先执行 pip install -r app/requirements.txt" % (exc.name or exc)) from exc
            self._device = self._resolve_device()
            model = self._open(YOLO)
            names = getattr(model, "names", {}) or {}
            if isinstance(names, (list, tuple)):
                names = dict(enumerate(names))
            self._names = {int(key): str(value) for key, value in names.items()}
            self._model = model
            return model

    # ---------------- 信息 ----------------
    @property
    def device_name(self):
        return self._device or self.device

    def class_names(self):
        self.load()
        return dict(self._names)

    def describe(self):
        """Short label for logs; never loads the model."""
        return "YOLOv8 %s · %s · %dpx" % (self.model_path.name, self.device, self.imgsz)

    # ---------------- 推理 ----------------
    def infer(self, image, confidence=.25):
        """Run one image; return [(x1, y1, x2, y2, score, label)] in image pixels."""
        model = self.load()
        with self._lock:
            results = model.predict(source=image, imgsz=self.imgsz, conf=float(confidence),
                                    device=self._device, verbose=self.verbose)
        rows = []
        for result in results or ():
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].tolist())
                score = float(box.conf[0])
                class_id = int(box.cls[0])
                rows.append((x1, y1, x2, y2, score, self._names.get(class_id, str(class_id))))
        return rows
