# -*- coding: utf-8 -*-
"""Calibrate crosswalk pixel rows against lidar-referenced distance.

The obstacle placed across the road is only a temporary measuring reference:
the recorded table is static, so the runtime code never needs the lidar.

Usage: py -3 .work\calibrate_by_lidar.py [steps] [step_m] [host]
"""
import json
import math
import sys
import time
import urllib.request
from pathlib import Path

OUT = Path(r"D:\项目\intelligent_car-main\.work\distance_calib")
PITCHES = (1500, 1400)
TOPIC_CMD = "/controller/cmd_vel"

sys.path.insert(0, r"D:\项目\intelligent_car-main\app")

from carapp.config import load_config  # noqa: E402
from carapp.core.ws import WebSocket  # noqa: E402
from carapp.features.traffic_light import _build  # noqa: E402


class Car:
    def __init__(self, host):
        self.ws = WebSocket(host, 9090, timeout=3.0)
        self.ws.send_text(json.dumps({"op": "subscribe", "topic": "/odom", "id": "odom"}))
        self.ws.send_text(json.dumps({"op": "subscribe", "topic": "/scan_raw", "id": "scan"}))
        self.host = host

    def drain(self, seconds=1.0):
        pose = None
        scan = None
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
            topic = data.get("topic")
            msg = data.get("msg") or {}
            if topic == "/odom":
                position = (((msg.get("pose") or {}).get("pose")) or {}).get("position") or {}
                if "x" in position:
                    pose = (float(position["x"]), float(position["y"]))
            elif topic == "/scan_raw":
                scan = msg
        return pose, scan

    def twist(self, vx):
        self.ws.send_text(json.dumps({"op": "publish", "topic": TOPIC_CMD,
            "msg": {"linear": {"x": vx, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}}))

    def step(self, meters, speed=0.08):
        direction = 1.0 if meters >= 0 else -1.0
        seconds = abs(meters) / max(speed * .98, .02)
        started = time.monotonic()
        while time.monotonic() - started < seconds:
            self.twist(direction * speed)
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


def forward_ranges(scan, half_deg=12.0):
    angle_min = float(scan.get("angle_min", 0.0))
    increment = float(scan.get("angle_increment", 0.0))
    rmin = float(scan.get("range_min", 0.0))
    rmax = float(scan.get("range_max", 0.0))
    half = math.radians(half_deg)
    found = []
    for index, value in enumerate(scan.get("ranges") or []):
        if not isinstance(value, (int, float)):
            continue
        distance = float(value)
        if not math.isfinite(distance) or distance < rmin or distance > rmax:
            continue
        angle = math.atan2(math.sin(angle_min + index * increment), math.cos(angle_min + index * increment))
        if abs(angle) <= half:
            found.append(distance)
    if not found:
        return None, None
    found.sort()
    return found[0], found[len(found) // 2]


def servo(host, joint, position):
    ws = WebSocket(host, 9090, timeout=3.0)
    ws.send_text(json.dumps({"op": "publish", "topic": "/ros_robot_controller/pwm_servo/set_state",
        "msg": {"duration": .8, "state": [{"id": [joint], "position": [position], "offset": [0]}]}}))
    ws.close()


def grab(host, name):
    OUT.mkdir(parents=True, exist_ok=True)
    req = urllib.request.Request("http://%s:8080/snapshot?topic=/image_raw" % host,
                                 headers={"Cache-Control": "no-store"})
    data = urllib.request.urlopen(req, timeout=6.0).read()
    (OUT / ("%s.jpg" % name)).write_bytes(data)
    return data


def crosswalk_rows(result):
    rows = []
    for obj in result.objects:
        if obj.label != "crosswalk":
            continue
        x, y, w, h = obj.box
        rows.append({"score": round(float(obj.score), 3), "box": [x, y, w, h],
                     "bottom": y + h, "top": y,
                     "aspect": round(w / float(h), 2) if h else 0.})
    rows = [row for row in rows if row["aspect"] >= 1.8]
    rows.sort(key=lambda row: row["bottom"], reverse=True)
    return rows


def main():
    steps = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    step_m = float(sys.argv[2]) if len(sys.argv) > 2 else 0.2
    host = sys.argv[3] if len(sys.argv) > 3 else "192.168.167.136"
    detector = _build(load_config())[0]
    car = Car(host)
    servo(host, 2, 1500)
    records = []
    try:
        pose, scan = car.drain(1.2)
        reference, _ = forward_ranges(scan)
        print("stop line reference: lidar forward %.3f m | odom %s" % (reference, pose))
        started = pose
        for index in range(steps + 1):
            if index:
                car.step(-step_m)
                time.sleep(0.8)
            pose, scan = car.drain(1.0)
            nearest, median = forward_ranges(scan)
            travel = math.hypot(pose[0] - started[0], pose[1] - started[1]) if pose and started else None
            distance = (nearest - reference) if (nearest and reference) else None
            entry = {"index": index, "lidar_forward": None if nearest is None else round(nearest, 3),
                     "distance_to_line": None if distance is None else round(distance, 3),
                     "odom_travel": None if travel is None else round(travel, 3),
                     "views": {}}
            for pitch in PITCHES:
                servo(host, 1, pitch)
                time.sleep(1.5)
                frame = grab(host, "lidar_%02d_%d" % (index, pitch))
                rows = crosswalk_rows(detector.detect(frame, "jpeg"))
                entry["views"][str(pitch)] = rows[:3]
            records.append(entry)
            print("step %2d  lidar %.2f m  to-line %.2f m  odom %.2f m"
                  % (index,
                     nearest if nearest is not None else float("nan"),
                     distance if distance is not None else float("nan"),
                     travel if travel is not None else float("nan")))
            for pitch in PITCHES:
                print("        dip %d rows: %s" % (pitch, entry["views"][str(pitch)]))
        (OUT / "lidar_samples.json").write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
        print("samples ->", OUT / "lidar_samples.json")
    finally:
        servo(host, 1, 1500)
        car.close()
        print("stopped")


if __name__ == "__main__":
    main()
