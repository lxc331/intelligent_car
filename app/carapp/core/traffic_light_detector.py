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
    """Infer full frames, then accept frontal 'red' detections at conf >= .5.

    The model emits end-to-end xyxy/confidence/class_id rows; its red class
    replaces the old RGB/circle heuristic. Only box centres within x15–85%,
    y2–80% of the source frame trigger the existing stop interlock.
    Imports, compilation and inference run lazily in the detection worker.
    """

    def __init__(self, model_dir=None, confidence=.5, *, infer=None):
        confidence = float(confidence)
        if not math.isfinite(confidence) or not 0. < confidence <= 1.:
            raise ValueError("红灯模型置信度应在 0–1 之间")
        self.model_dir = Path(model_dir) if model_dir is not None else MODEL_DIR
        self.confidence = confidence
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

    def detect(self, payload, mode="jpeg"):
        from PIL import Image, UnidentifiedImageError
        import numpy as np

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
            # Ultralytics letterbox: preserve aspect ratio, pad with 114,
            # feed RGB float32 NCHW in [0,1], never stretch to a square.
            gain = min(size / width, size / height)
            resized = (round(width * gain), round(height * gain))
            left = round((size - resized[0]) / 2 - .1)
            top = round((size - resized[1]) / 2 - .1)
            canvas = Image.new("RGB", (size, size), (114, 114, 114))
            canvas.paste(image.resize(resized, Image.Resampling.BILINEAR), (left, top))
            tensor = np.ascontiguousarray(np.asarray(canvas).transpose(2, 0, 1)[None], dtype=np.float32)
            tensor /= 255.
            rows = np.asarray(self._predict(tensor))
            if rows.ndim != 3 or rows.shape[0] != 1 or rows.shape[2] != 6 or rows.shape[1] > 300:
                raise ValueError("红灯模型返回格式错误，应为 [1,N,6]")
            if not np.isfinite(rows).all():
                raise ValueError("红灯模型返回了无效数值")
            boxes, scores, objects = [], [], []
            for x1, y1, x2, y2, score, class_id in rows[0]:
                if not 0. <= score <= 1.:
                    raise ValueError("红灯模型置信度无效")
                if score < self.confidence:
                    continue
                label = names.get(int(class_id))
                if class_id != int(class_id) or label is None:
                    raise ValueError("红灯模型返回了未知类别")
                x1, x2 = [max(0., min(width, (float(x) - left) / gain)) for x in (x1, x2)]
                y1, y2 = [max(0., min(height, (float(y) - top) / gain)) for y in (y1, y2)]
                if x2 <= x1 or y2 <= y1:
                    continue
                x, y = round(x1), round(y1)
                w, h = round(x2) - x, round(y2) - y
                if w > 0 and h > 0:
                    objects.append(TrafficObject(label, float(score), (x, y, w, h)))
                    if (label == "red" and .15 < (x1 + x2) / 2 / width < .85
                            and .02 < (y1 + y2) / 2 / height < .80):
                        boxes.append((x, y, w, h))
                        scores.append(float(score))
        return TrafficLightDetection(bool(boxes), "模型识别到正前方红灯" if boxes else "模型未识别到正前方红灯",
                                     tuple(boxes), tuple(scores), tuple(objects), (width, height))
