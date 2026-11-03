# -*- coding: utf-8 -*-
"""Live traffic probe with RAW model output (no confidence/frontal filtering).

Prints every model row above a low threshold so we can tell the difference between
"the traffic light is not in view" and "the model sees it weakly / too small".
Also replays the App's TrafficScenePolicy on the same frames to show the decision.

Read-only: fetches /snapshot?topic=/image_raw from the car's web_video_server.
"""
import io
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "app"))

from carapp.core.traffic_light_detector import MODEL_DIR, TrafficObject  # noqa: E402
from carapp.core.traffic_scene import TrafficScenePolicy  # noqa: E402

HOST = "192.168.167.136"
TOPIC = "/image_raw"
COUNT = int(sys.argv[1]) if len(sys.argv) > 1 else 10
RAW_THRESHOLD = .05
OUT = ROOT / ".work" / "traffic_light_live"
URL = "http://%s:8080/snapshot?topic=%s" % (HOST, TOPIC)


def load_runner():
    import openvino as ov
    import yaml

    with (MODEL_DIR / "metadata.yaml").open(encoding="utf-8") as source:
        metadata = yaml.safe_load(source)
    names = metadata["names"]
    names = dict(enumerate(names)) if isinstance(names, list) else names
    names = {int(key): str(value) for key, value in names.items()}
    size = int(metadata["imgsz"][0])

    core = ov.Core()
    compiled = core.compile_model(str(MODEL_DIR / "best_traffic_26.xml"), "CPU", {
        "PERFORMANCE_HINT": "LATENCY", "INFERENCE_NUM_THREADS": 2, "NUM_STREAMS": "1",
    })
    request = compiled.create_infer_request()

    def run(payload):
        with Image.open(io.BytesIO(payload)) as source:
            image = source.convert("RGB")
        width, height = image.size
        gain = min(size / width, size / height)
        resized = (round(width * gain), round(height * gain))
        left = round((size - resized[0]) / 2 - .1)
        top = round((size - resized[1]) / 2 - .1)
        canvas = Image.new("RGB", (size, size), (114, 114, 114))
        canvas.paste(image.resize(resized, Image.Resampling.BILINEAR), (left, top))
        tensor = np.ascontiguousarray(np.asarray(canvas).transpose(2, 0, 1)[None], dtype=np.float32)
        tensor /= 255.
        rows = np.asarray(request.infer({0: tensor})[0])[0]
        items = []
        for x1, y1, x2, y2, score, class_id in rows:
            if score < RAW_THRESHOLD:
                continue
            label = names.get(int(class_id), "?")
            sx1, sx2 = [max(0., min(width, (float(value) - left) / gain)) for value in (x1, x2)]
            sy1, sy2 = [max(0., min(height, (float(value) - top) / gain)) for value in (y1, y2)]
            if sx2 <= sx1 or sy2 <= sy1:
                continue
            x, y = round(sx1), round(sy1)
            w, h = round(sx2) - x, round(sy2) - y
            items.append(TrafficObject(label, float(score), (x, y, w, h)))
        return items, (width, height)

    return run


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    run = load_runner()
    policy = TrafficScenePolicy()
    print("url:", URL, "| raw threshold:", RAW_THRESHOLD)
    for index in range(COUNT):
        try:
            request = urllib.request.Request(URL, headers={"Cache-Control": "no-store"})
            with urllib.request.urlopen(request, timeout=5.) as response:
                payload = response.read()
        except Exception as exc:  # noqa: BLE001
            print("%2d fetch failed: %s" % (index, exc))
            time.sleep(.5)
            continue
        received = time.monotonic()
        (OUT / ("raw_%02d.jpg" % index)).write_bytes(payload)
        items, size = run(payload)
        required = policy.observe(items, size, received)
        state = policy.snapshot()
        text = ", ".join("%s %.2f @(%d,%d,%d,%d) a=%d"
                         % (i.label, i.score, *i.box, i.box[2] * i.box[3]) for i in items) or "-"
        print("%2d %dx%d required=%-5s active=%-5s xwalk_y=%.0f red_area=%.0f | %s"
              % (index, size[0], size[1], required, state["crosswalk_active"],
                 state["crosswalk_y"], state["scene_red_area"], text))
        print("     scene: %s" % state["scene"])
        time.sleep(.4)
    print("frames in", OUT)


if __name__ == "__main__":
    main()
