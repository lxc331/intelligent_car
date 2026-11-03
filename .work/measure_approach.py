# -*- coding: utf-8 -*-
"""Forward approach measurement: odometry distance vs crosswalk position in frame.

Usage:
  py -3 .work\measure_approach.py status
  py -3 .work\measure_approach.py run [steps] [speed] [seconds]

Each step drives forward at a fixed command speed, then records
  - odom pose (queue drained, latest message wins)
  - one JPEG frame
  - crosswalk/red detections on that frame
Steps stop early when the near crosswalk reaches the bottom edge (we are at the
stop line) or when the travel budget is used up.
"""
import json
import math
import sys
import time
import urllib.request
from pathlib import Path

HOST = "192.168.167.136"
OUT = Path(r"D:\项目\intelligent_car-main\.work\approach_calib")
TOPIC_CMD = "/controller/cmd_vel"
SAFE_MAX = 0.12

sys.path.insert(0, r"D:\项目\intelligent_car-main\app")

from carapp.config import load_config  # noqa: E402
from carapp.core.ws import WebSocket  # noqa: E402
from carapp.features.traffic_light import _build  # noqa: E402


class Node:
    def __init__(self):
        self.ws = WebSocket(HOST, 9090, timeout=3.0)
        self.ws.send_text(json.dumps({"op": "subscribe", "topic": "/odom", "id": "odom"}))

    def drain_pose(self, seconds=1.2):
        latest = None
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                raw = self.ws.recv_text()
            except Exception:
                continue
            try:
                data = json.loads(raw)
            except ValueError:
                continue
            if data.get("op") != "publish":
                continue
            msg = data.get("msg") or {}
            position = (((msg.get("pose") or {}).get("pose")) or {}).get("position") or {}
            if "x" in position:
                latest = (float(position["x"]), float(position["y"]))
        return latest

    def twist(self, vx):
        self.ws.send_text(json.dumps({"op": "publish", "topic": TOPIC_CMD,
            "msg": {"linear": {"x": vx, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}}))

    def drive(self, speed, seconds):
        speed = max(-SAFE_MAX, min(SAFE_MAX, abs(float(speed))))
        started = time.monotonic()
        while time.monotonic() - started < seconds:
            self.twist(speed)
            time.sleep(0.1)
        self.stop()

    def stop(self, repeat=10):
        for _ in range(repeat):
            self.twist(0.0)
            time.sleep(0.05)

    def close(self):
        try:
            self.stop()
        finally:
            self.ws.close()


def grab(name):
    OUT.mkdir(parents=True, exist_ok=True)
    url = "http://%s:8080/snapshot?topic=/image_raw" % HOST
    req = urllib.request.Request(url, headers={"Cache-Control": "no-store"})
    data = urllib.request.urlopen(req, timeout=6.0).read()
    path = OUT / ("%s.jpg" % name)
    path.write_bytes(data)
    return path, data


def crosswalk_rows(objects):
    rows = []
    for obj in objects:
        if obj.label != "crosswalk":
            continue
        x, y, w, h = obj.box
        rows.append({"score": round(float(obj.score), 3), "box": [x, y, w, h],
                     "cx": x + w / 2.0, "cy": y + h / 2.0, "bottom": y + h})
    return rows


def main():
    command = sys.argv[1] if len(sys.argv) > 1 else "status"
    steps = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    speed = float(sys.argv[3]) if len(sys.argv) > 3 else 0.10
    seconds = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
    detector = _build(load_config())[0]
    node = Node()
    records = []
    try:
        start = node.drain_pose()
        path, data = grab("approach_00")
        result = detector.detect(data, "jpeg")
        record = {"step": 0, "travel": 0.0, "pose": start,
                  "crosswalk": crosswalk_rows(result.objects),
                  "red": result.red}
        records.append(record)
        print("step 0 travel=0.000 pose=%s crosswalk=%s red=%s"
              % (start, record["crosswalk"], result.red))
        for index in range(1, steps + 1):
            node.drive(speed, seconds)
            time.sleep(0.8)
            pose = node.drain_pose()
            path, data = grab("approach_%02d" % index)
            result = detector.detect(data, "jpeg")
            travel = math.hypot(pose[0] - start[0], pose[1] - start[1]) if pose and start else 0.0
            record = {"step": index, "travel": round(travel, 3), "pose": pose,
                      "crosswalk": crosswalk_rows(result.objects), "red": result.red}
            records.append(record)
            near = max((row["bottom"] for row in record["crosswalk"]), default=0)
            print("step %d travel=%.3f m  speed=%.3f m/s  crosswalk=%s  red=%s"
                  % (index, travel, travel / (index * seconds), record["crosswalk"], result.red))
            if near >= 470:
                print("near crosswalk reached the bottom edge -> assume stop line reached")
                break
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT / "records.json").write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
        print("records ->", OUT / "records.json")
    finally:
        node.close()
        print("stopped")


if __name__ == "__main__":
    main()
