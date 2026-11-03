# -*- coding: utf-8 -*-
"""Prototype: full frame + upper-half zoom tiles, merged with NMS.

Measures what the model reports when the traffic lights are zoomed in, so we can
pick zoom regions and red-area thresholds from data instead of guessing.
"""
import importlib.util
import io
import sys
import time
import urllib.request
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "app"))

from carapp.core.traffic_scene import TrafficScenePolicy  # noqa: E402

HOST = "192.168.167.136"
COUNT = int(sys.argv[1]) if len(sys.argv) > 1 else 8
CONF = .5
TILES = ((0., 0., .5, .6), (.5, 0., .5, .6))

spec = importlib.util.spec_from_file_location("probe", ROOT / ".work" / "live_traffic_raw_probe.py")
probe = importlib.util.module_from_spec(spec)
sys.argv = sys.argv[:1]
spec.loader.exec_module(probe)


def letterbox_tensor(image, size):
    import numpy as np
    width, height = image.size
    gain = min(size / width, size / height)
    resized = (round(width * gain), round(height * gain))
    left = round((size - resized[0]) / 2 - .1)
    top = round((size - resized[1]) / 2 - .1)
    canvas = Image.new("RGB", (size, size), (114, 114, 114))
    canvas.paste(image.resize(resized, Image.Resampling.BILINEAR), (left, top))
    tensor = np.ascontiguousarray(np.asarray(canvas).transpose(2, 0, 1)[None], dtype=np.float32)
    tensor /= 255.
    return tensor, gain, left, top


def load():
    import openvino as ov
    import yaml
    from carapp.core.traffic_light_detector import MODEL_DIR
    with (MODEL_DIR / "metadata.yaml").open(encoding="utf-8") as source:
        metadata = yaml.safe_load(source)
    label_names = metadata["names"]
    label_names = dict(enumerate(label_names)) if isinstance(label_names, list) else label_names
    label_names = {int(key): str(value) for key, value in label_names.items()}
    compiled = ov.Core().compile_model(str(MODEL_DIR / "best_traffic_26.xml"), "CPU", {
        "PERFORMANCE_HINT": "LATENCY", "INFERENCE_NUM_THREADS": 2, "NUM_STREAMS": "1",
    })
    return label_names, int(metadata["imgsz"][0]), compiled.create_infer_request()


names, size, request = load()


def infer_region(image, rect, tag):
    import numpy as np
    width, height = image.size
    x0, y0, w0, h0 = rect
    box = (round(x0 * width), round(y0 * height),
           round((x0 + w0) * width), round((y0 + h0) * height))
    crop = image.crop(box)
    tensor, gain, left, top = letterbox_tensor(crop, size)
    started = time.perf_counter()
    rows = np.asarray(request.infer({0: tensor})[0])[0]
    cost = (time.perf_counter() - started) * 1000.
    out = []
    for x1, y1, x2, y2, score, class_id in rows:
        if score < CONF:
            continue
        sx1, sx2 = [(float(v) - left) / gain + box[0] for v in (x1, x2)]
        sy1, sy2 = [(float(v) - top) / gain + box[1] for v in (y1, y2)]
        sx1, sx2 = max(0., min(width, sx1)), max(0., min(width, sx2))
        sy1, sy2 = max(0., min(height, sy1)), max(0., min(height, sy2))
        if sx2 - sx1 < 1 or sy2 - sy1 < 1:
            continue
        out.append(dict(label=names[int(class_id)], score=float(score),
                        box=(round(sx1), round(sy1), round(sx2 - sx1), round(sy2 - sy1)),
                        tag=tag))
    return out, cost


def iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x1, y1 = max(ax, bx), max(ay, by)
    x2, y2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
    if x2 <= x1 or y2 <= y1:
        return 0.
    inter = (x2 - x1) * (y2 - y1)
    return inter / float(aw * ah + bw * bh - inter)


def merge(items):
    kept = []
    for item in sorted(items, key=lambda i: i["score"], reverse=True):
        if all(not (item["label"] == k["label"] and iou(item["box"], k["box"]) > .45) for k in kept):
            kept.append(item)
    return kept


policy = TrafficScenePolicy()
url = "http://%s:8080/snapshot?topic=/image_raw" % HOST
for index in range(COUNT):
    request_url = urllib.request.Request(url, headers={"Cache-Control": "no-store"})
    with urllib.request.urlopen(request_url, timeout=8.) as response:
        payload = response.read()
    received = time.monotonic()
    with Image.open(io.BytesIO(payload)) as source:
        image = source.convert("RGB")
    full, c0 = infer_region(image, (0., 0., 1., 1.), "full")
    tiles, costs = [], []
    for rect in TILES:
        part, cost = infer_region(image, rect, "tile")
        tiles.extend(part)
        costs.append(cost)
    objects = merge(full + tiles)
    required = policy.observe([type("O", (), {"label": o["label"], "score": o["score"], "box": o["box"]})()
                               for o in objects], image.size, received)
    text = " | ".join("%s %.2f %s a=%d" % (o["label"], o["score"], o["tag"], o["box"][2] * o["box"][3])
                      for o in objects) or "-"
    state = policy.snapshot()
    print("%2d ms full=%.0f tile=%s required=%-5s active=%-5s red_area=%.0f" %
          (index, c0, "/".join("%.0f" % c for c in costs), required,
           state["crosswalk_active"], state["scene_red_area"]))
    print("     %s" % text)
