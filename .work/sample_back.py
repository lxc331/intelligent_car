# -*- coding: utf-8 -*-
"""Reverse in measured steps and record near-crosswalk rows for distance fitting.

Usage: py -3 .work\sample_back.py [steps] [step_m] [dip_pitch] [start_distance_m]
"""
import json
import math
import sys
import time
import urllib.request
from pathlib import Path

HOST = "192.168.167.136"
OUT = Path(r"D:\项目\intelligent_car-main\.work\distance_calib")
TOPIC_CMD = "/controller/cmd_vel"

sys.path.insert(0, r"D:\项目\intelligent_car-main\app")

from carapp.config import load_config  # noqa: E402
from carapp.core.ws import WebSocket  # noqa: E402
from carapp.features.traffic_light import _build  # noqa: E402


class Car:
    def __init__(self):
        self.ws = WebSocket(HOST, 9090, timeout=3.0)
        self.ws.send_text(json.dumps({"op": "subscribe", "topic": "/odom", "id": "odom"}))

    def pose(self, seconds=0.9):
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


def servo(joint, position):
    ws = WebSocket(HOST, 9090, timeout=3.0)
    ws.send_text(json.dumps({"op": "publish", "topic": "/ros_robot_controller/pwm_servo/set_state",
        "msg": {"duration": .8, "state": [{"id": [joint], "position": [position], "offset": [0]}]}}))
    ws.close()


def grab(name):
    OUT.mkdir(parents=True, exist_ok=True)
    req = urllib.request.Request("http://%s:8080/snapshot?topic=/image_raw" % HOST,
                                 headers={"Cache-Control": "no-store"})
    data = urllib.request.urlopen(req, timeout=6.0).read()
    (OUT / ("%s.jpg" % name)).write_bytes(data)
    return data


def pick_crosswalk(result):
    rows = []
    for obj in result.objects:
        if obj.label != "crosswalk":
            continue
        x, y, w, h = obj.box
        rows.append({"score": round(float(obj.score), 3), "box": [x, y, w, h],
                     "bottom": y + h, "width": w,
                     "aspect": round(w / float(h), 2) if h else 0.})
    rows = [row for row in rows if row["aspect"] >= 1.8]
    rows.sort(key=lambda row: row["bottom"], reverse=True)
    return rows


def main():
    steps = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    step_m = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
    dip = int(sys.argv[3]) if len(sys.argv) > 3 else 1400
    start_distance = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
    detector = _build(load_config())[0]
    servo(2, 1500)
    servo(1, dip)
    time.sleep(1.6)
    car = Car()
    records = []
    try:
        origin = car.pose()
        for index in range(steps + 1):
            if index:
                car.step(-step_m)
                time.sleep(0.7)
            pose = car.pose()
            travel = math.hypot(pose[0] - origin[0], pose[1] - origin[1]) if pose and origin else 0.
            frame = grab("back_%02d" % index)
            rows = pick_crosswalk(detector.detect(frame, "jpeg"))
            near = rows[0] if rows else None
            distance = start_distance + travel
            records.append({"index": index, "travel": round(travel, 3), "distance": round(distance, 3),
                            "rows": rows[:3]})
            print("step %2d  distance ~%.2f m  %s"
                  % (index, distance, rows[:2] if rows else "no crosswalk"))
        (OUT / "back_samples.json").write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
        print("samples ->", OUT / "back_samples.json")
    finally:
        car.close()
        print("stopped")


if __name__ == "__main__":
    main()
