# -*- coding: utf-8 -*-
"""Live traffic-light probe: pull camera snapshots from the car and run the App's detector.

Read-only: fetches /snapshot?topic=/image_raw from web_video_server, runs
carapp.core.traffic_light_detector on the same bytes the App would receive, and
prints what the model sees (labels, scores, boxes, frontal-red gate result).
"""
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "app"))

from carapp.core.traffic_light_detector import TrafficLightDetector  # noqa: E402

HOST = "192.168.167.136"
TOPIC = "/image_raw"
COUNT = 12
OUT = ROOT / ".work" / "traffic_light_live"
URL = "http://%s:8080/snapshot?topic=%s" % (HOST, TOPIC)


def grab(timeout=5.):
    request = urllib.request.Request(URL, headers={"Cache-Control": "no-store"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return response.read()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    detector = TrafficLightDetector()
    detector.ensure_available()
    print("url:", URL)
    seen = {}
    started = time.perf_counter()
    for index in range(COUNT):
        try:
            payload = grab()
        except Exception as exc:  # noqa: BLE001 - report and continue
            print("%2d fetch failed: %s" % (index, exc))
            time.sleep(1.)
            continue
        (OUT / ("frame_%02d.jpg" % index)).write_bytes(payload)
        before = time.perf_counter()
        result = detector.detect(payload)
        elapsed = (time.perf_counter() - before) * 1000.
        description = ", ".join("%s %.2f @%s" % (item.label, item.score, item.box) for item in result.objects) or "-"
        print("%2d %4d B %6.1f ms red=%-5s objects: %s" % (index, len(payload), elapsed, result.red, description))
        for item in result.objects:
            seen[item.label] = max(seen.get(item.label, 0.), item.score)
        time.sleep(1.)
    print("total %.1f s, labels: %s" % (time.perf_counter() - started, seen or "-"))

    best = OUT / "frame_best.jpg"
    frames = sorted(OUT.glob("frame_*.jpg"))
    if frames:
        best.write_bytes(frames[-1].read_bytes())
        print("frames saved in", OUT)


if __name__ == "__main__":
    main()
