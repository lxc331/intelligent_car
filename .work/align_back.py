# -*- coding: utf-8 -*-
"""Back off in tiny bursts until the crosswalk row falls below a target.

Used to put the car back before a stop-line trial without asking the operator to
push it by hand every time.  Safety: rear lidar sectors must stay clear, movement
is a few centimetres per burst, and the loop stops on any doubt.
"""
import json
import math
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(r"D:\项目\intelligent_car-main")
sys.path.insert(0, str(ROOT / "app"))
sys.path.insert(0, str(ROOT / ".work"))

import battery  # noqa: E402
from carapp.config import load_config  # noqa: E402
from carapp.core.traffic_light_detector import TrafficLightDetector  # noqa: E402
from carapp.core.ws import WebSocket  # noqa: E402

HOST = "192.168.167.118"
REAR_MIN = 0.30


class Rig:
    def __init__(self):
        self.ws = WebSocket(HOST, 9090, timeout=3.0)
        self.ws.send_text(json.dumps({"op": "subscribe", "topic": "/scan_raw", "id": "scan"}))
        self.scan = None

    def twist(self, vx):
        self.ws.send_text(json.dumps({"op": "publish", "topic": "/controller/cmd_vel",
            "msg": {"linear": {"x": float(vx), "y": 0., "z": 0.},
                    "angular": {"x": 0., "y": 0., "z": 0.}}}))

    def pump(self, seconds=.3):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                raw = self.ws.recv_text()
            except Exception:
                return
            try:
                data = json.loads(raw)
            except ValueError:
                continue
            if data.get("topic") == "/scan_raw":
                self.scan = data.get("msg")

    def stop(self):
        for _ in range(10):
            self.twist(0.)
            time.sleep(0.03)

    def rear(self):
        scan = self.scan or {}
        angle_min = float(scan.get("angle_min", 0.))
        inc = float(scan.get("angle_increment", 0.))
        best = None
        for index, value in enumerate(scan.get("ranges") or []):
            try:
                distance = float(value)
            except (TypeError, ValueError):
                continue
            if not math.isfinite(distance) or distance <= 0.02:
                continue
            angle = math.degrees(angle_min + index * inc) % 360.
            if 140. <= angle <= 220.:
                best = distance if best is None else min(best, distance)
        return best


def servo(pitch):
    ws = WebSocket(HOST, 9090, timeout=3.0)
    ws.send_text(json.dumps({"op": "publish", "topic": "/ros_robot_controller/pwm_servo/set_state",
        "msg": {"duration": .8, "state": [{"id": [1], "position": [pitch], "offset": [0]}]}}))
    ws.close()
    time.sleep(1.5)


def row_now(detector):
    req = urllib.request.Request("http://%s:8080/snapshot?topic=/image_raw" % HOST,
                                 headers={"Cache-Control": "no-store"})
    data = urllib.request.urlopen(req, timeout=5.).read()
    result = detector.detect(data, "jpeg")
    near = []
    for obj in result.objects:
        if obj.label != "crosswalk":
            continue
        x, y, w, h = obj.box
        if not h or w / float(h) < 1.8 or y + h < 300.:
            continue
        near.append((y + h, x + w / 2.))
    if not near:
        return None, None
    bottom, centre = max(near, key=lambda item: item[0])
    return bottom, centre


def main():
    target = float(sys.argv[2]) if len(sys.argv) > 2 else 398.
    settings = load_config()["traffic_light"]
    detector = TrafficLightDetector(confidence=settings["confidence"], zoom=settings["zoom"],
                                    zoom_confidence=settings["zoom_confidence"],
                                    candidate_confidence=settings["candidate_confidence"],
                                    red_aspect=settings["red_aspect"])
    rig = Rig()
    try:
        battery.ensure_ok(rig.ws)
        servo(1450)
        rig.pump(.5)
        for attempt in range(8):
            row, centre = row_now(detector)
            print("attempt %d row=%s rear=%s" % (attempt, row, rig.rear()))
            if row is None or row <= target:
                print("done:", row)
                return
            rear = rig.rear()
            if rear is not None and rear < REAR_MIN:
                print("后方 %.2f m 太近，停止倒车" % rear)
                return
            rig.twist(-0.04)
            time.sleep(0.30)
            rig.stop()
            rig.pump(.4)
            time.sleep(0.3)
    finally:
        rig.stop()
        rig.ws.close()


if __name__ == "__main__":
    main()
