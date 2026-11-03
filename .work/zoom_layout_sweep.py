"""Sweep candidate zoom layouts over saved frames; pick the most robust one.

For every layout the detector runs one full-frame pass plus each region, and we
print the strongest red detection (score, area) per saved frame. A layout is
good when every frame that clearly shows the lit red lamp reaches score >= .5
and area >= 450 px^2.
"""
import sys
from pathlib import Path

ROOT = Path(r"D:\项目\intelligent_car-main")
sys.path.insert(0, str(ROOT / "app"))

from carapp.core.traffic_light_detector import TrafficLightDetector

LIVE = ROOT / ".work" / "traffic_light_live"
FRAMES = ["pitch_1600.jpg", "zoom_check_now.jpg", "probe_now.jpg"]

LAYOUTS = {
    "A 2x.55 full": ((0., 0., .55, .6), (.45, 0., .55, .6)),
    "B 2x.55 y.28-.68": ((0., .28, .55, .4), (.45, .28, .55, .4)),
    "C 3x.3 y.28-.68": ((0., .28, .3, .4), (.35, .28, .3, .4), (.7, .28, .3, .4)),
    "D 4x.28 y.28-.68": ((0., .28, .28, .4), (.24, .28, .28, .4), (.48, .28, .28, .4), (.72, .28, .28, .4)),
    "E 2x.45 center": ((.25, .28, .45, .4), (.45, .28, .45, .4)),
    "F 4x.3 y.2-.6": ((0., .2, .3, .4), (.233, .2, .3, .4), (.466, .2, .3, .4), (.7, .2, .3, .4)),
    "G 3x.4 y.25-.65": ((0., .25, .4, .4), (.3, .25, .4, .4), (.6, .25, .4, .4)),
}


def best_red(detector, payload):
    result = detector.detect(payload, "jpeg")
    reds = [(item.score, item.box[2] * item.box[3]) for item in result.objects if item.label == "red"]
    return max(reds) if reds else (0., 0)


for name, regions in LAYOUTS.items():
    detector = TrafficLightDetector(confidence=.05, zoom=regions, zoom_confidence=.05)
    cells = []
    for frame in FRAMES:
        payload = (LIVE / frame).read_bytes()
        score, area = best_red(detector, payload)
        cells.append("%s=%.2f/%d" % (frame.split(".")[0][:12], score, area))
    print("%-22s %s" % (name, "  ".join(cells)))
