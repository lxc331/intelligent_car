# -*- coding: utf-8 -*-
"""Size-sensitivity test: does the detector see the red light when it is bigger?

Fetches /image_raw snapshots (optionally resized by web_video_server), plus a
cropped+upscaled patch around the traffic light, and runs the same raw detector.
"""
import importlib.util
import io
import sys
import urllib.request
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "app"))

spec = importlib.util.spec_from_file_location("probe", ROOT / ".work" / "live_traffic_raw_probe.py")
probe = importlib.util.module_from_spec(spec)
sys.argv = sys.argv[:1]
spec.loader.exec_module(probe)
run = probe.load_runner()
OUT = ROOT / ".work" / "traffic_light_live"
HOST = "192.168.167.136"


def fetch(url):
    request = urllib.request.Request(url, headers={"Cache-Control": "no-store"})
    with urllib.request.urlopen(request, timeout=8.) as response:
        return response.read()


def show(tag, payload):
    with Image.open(io.BytesIO(payload)) as source:
        size = source.size
    items, _ = run(payload)
    print("%-22s %dx%d" % (tag, size[0], size[1]))
    for item in items:
        print("   %-9s %.3f box=%s area=%d" % (item.label, item.score, item.box,
                                               item.box[2] * item.box[3]))
    if not items:
        print("   (nothing above 0.05)")
    return payload


base = "http://%s:8080/snapshot?topic=/image_raw" % HOST
small = show("native", fetch(base))
(OUT / "size_native.jpg").write_bytes(small)

for width, height in ((1280, 720), (1280, 960), (960, 720)):
    try:
        payload = fetch("%s&width=%d&height=%d" % (base, width, height))
    except Exception as exc:  # noqa: BLE001
        print("request %dx%d failed: %s" % (width, height, exc))
        continue
    show("stream %dx%d" % (width, height), payload)
    (OUT / ("size_%dx%d.jpg" % (width, height))).write_bytes(payload)

# Crop the traffic-light band and upscale it 3x, as a "zoom lens" simulation.
with Image.open(io.BytesIO(small)) as source:
    image = source.convert("RGB")
box = (300, 150, 640, 330)
for factor in (2, 3, 4):
    crop = image.crop(box)
    crop = crop.resize((crop.width * factor, crop.height * factor), Image.Resampling.BICUBIC)
    buffer = io.BytesIO()
    crop.save(buffer, "JPEG", quality=92)
    show("crop %dx" % factor, buffer.getvalue())
    (OUT / ("crop_%dx.jpg" % factor)).write_bytes(buffer.getvalue())
