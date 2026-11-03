# -*- coding: utf-8 -*-
"""Capture detector samples at the stop line for the three calibrated views.

Usage: py -3 .work\sample_lights.py [frames_per_view] [host]
"""
import json
import sys
import time
import urllib.request
from pathlib import Path

HOST = sys.argv[2] if len(sys.argv) > 2 else "192.168.167.136"
COUNT = int(sys.argv[1]) if len(sys.argv) > 1 else 12
OUT = Path(r"D:\项目\intelligent_car-main\.work\light_samples")
sys.path.insert(0, r"D:\项目\intelligent_car-main\app")

from carapp.config import load_config  # noqa: E402
from carapp.core.ws import WebSocket  # noqa: E402
from carapp.features.traffic_light import _build  # noqa: E402

VIEWS = (("lift", 1720), ("dip", 1450), ("forward", 1500))


def move(joint_id, pos):
    ws = WebSocket(HOST, 9090, timeout=5.0)
    ws.send_text(json.dumps({"op": "publish", "topic": "/ros_robot_controller/pwm_servo/set_state",
        "msg": {"duration": .8, "state": [{"id": [joint_id], "position": [pos], "offset": [0]}]}}))
    ws.close()


def grab():
    url = "http://%s:8080/snapshot?topic=/image_raw" % HOST
    req = urllib.request.Request(url, headers={"Cache-Control": "no-store"})
    return urllib.request.urlopen(req, timeout=6.0).read()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    cfg = load_config()
    detector = _build(cfg)[0]
    move(2, 1500)
    time.sleep(1.6)
    summary = {}
    for name, pitch in VIEWS:
        move(1, pitch)
        time.sleep(1.8)
        rows = []
        for index in range(COUNT):
            frame = grab()
            (OUT / ("%s_%02d.jpg" % (name, index))).write_bytes(frame)
            result = detector.detect(frame, "jpeg")
            reds = [(round(o.score, 3), o.box) for o in result.objects if o.label == "red"]
            greens = [(round(o.score, 3), o.box) for o in result.objects if o.label == "green"]
            area = max((box[2] * box[3] for _, box in reds), default=0)
            rows.append({"index": index, "red": result.red, "reds": reds, "greens": greens,
                         "red_area": area, "objects": [(o.label, round(o.score, 2)) for o in result.objects]})
            time.sleep(0.15)
        hits = [row for row in rows if row["reds"]]
        scores = [box[0] for row in hits for box in row["reds"]]
        areas = [row["red_area"] for row in hits]
        green_hits = [box for row in rows for box in row["greens"]]
        summary[name] = {
            "pitch": pitch,
            "frames": COUNT,
            "red_frames": len(hits),
            "red_score_min": min(scores) if scores else 0.0,
            "red_score_max": max(scores) if scores else 0.0,
            "red_area_min": min(areas) if areas else 0,
            "red_area_max": max(areas) if areas else 0,
            "green_hits": len(green_hits),
            "green_best": max((box[0] for box in green_hits), default=0.0),
            "sample": rows[0],
        }
        print("%-8s pitch=%d  red %d/%d  score %.2f-%.2f  area %d-%d  green hits %d  best %.2f"
              % (name, pitch, len(hits), COUNT,
                 summary[name]["red_score_min"], summary[name]["red_score_max"],
                 summary[name]["red_area_min"], summary[name]["red_area_max"],
                 len(green_hits), summary[name]["green_best"]))
    (OUT / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print("summary ->", OUT / "summary.json")


if __name__ == "__main__":
    main()
