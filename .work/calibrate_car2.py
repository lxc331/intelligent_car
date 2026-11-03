# -*- coding: utf-8 -*-
"""Car-2 calibration helpers: three-view sanity check and stop-line sampling.

Usage:
  py -3 .work/calibrate_car2.py views [host]
  py -3 .work/calibrate_car2.py speed <mps> <seconds> [host]
  py -3 .work/calibrate_car2.py steps <count> <step_m> [host]

Every command reads the battery first and refuses to move the wheels when the
pack is low or silent. Forward only: the sandbox has a tree right behind the car
and reversing already caused a collision on 2026-10-29.
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
from carapp.core.traffic_scene import TrafficScenePolicy  # noqa: E402
from carapp.core.ws import WebSocket  # noqa: E402

OUT = ROOT / ".work" / "car2_calib"
OUT.mkdir(parents=True, exist_ok=True)
FRONT_MIN = 0.35


class Car:
    def __init__(self, host):
        self.host = host
        self.ws = WebSocket(host, 9090, timeout=3.0)
        self.ws.send_text(json.dumps({"op": "subscribe", "topic": "/odom", "id": "odom"}))
        self.ws.send_text(json.dumps({"op": "subscribe", "topic": "/scan_raw", "id": "scan"}))

    def close(self):
        self.stop()
        self.ws.close()

    def twist(self, vx, wz=0.):
        self.ws.send_text(json.dumps({"op": "publish", "topic": "/controller/cmd_vel",
                                      "msg": {"linear": {"x": float(vx), "y": 0., "z": 0.},
                                              "angular": {"x": 0., "y": 0., "z": float(wz)}}}))

    def stop(self, times=8):
        for _ in range(times):
            self.twist(0., 0.)
            time.sleep(0.03)

    def read(self, seconds=1.2):
        """Drain the socket and return the freshest odom pose and scan."""
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
                try:
                    pos = msg["pose"]["pose"]["position"]
                    ori = msg["pose"]["pose"]["orientation"]
                    yaw = math.atan2(2 * (ori["w"] * ori["z"] + ori["x"] * ori["y"]),
                                     1 - 2 * (ori["y"] ** 2 + ori["z"] ** 2))
                    pose = (float(pos["x"]), float(pos["y"]), yaw)
                except Exception:
                    pass
            elif topic == "/scan_raw":
                scan = msg
        return pose, scan

    def front_range(self, scan, span=6.):
        if not scan:
            return None
        angle_min = float(scan.get("angle_min", 0.))
        step = float(scan.get("angle_increment", 0.))
        best = None
        for index, value in enumerate(scan.get("ranges") or []):
            try:
                distance = float(value)
            except (TypeError, ValueError):
                continue
            if not math.isfinite(distance) or distance <= 0.02:
                continue
            angle = math.degrees(angle_min + step * index)
            if abs(angle) <= span:
                best = distance if best is None else min(best, distance)
        return best

    def step(self, metres, speed, scale=1.87):
        seconds = metres / max(speed * scale, 1e-6)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.twist(speed, 0.)
            time.sleep(0.1)
        self.stop()
        time.sleep(0.6)


def servo(car, pitch, yaw=1500):
    car.ws.send_text(json.dumps({"op": "publish", "topic": "/ros_robot_controller/pwm_servo/set_state",
                                 "msg": {"duration": .8, "state": [{"id": [1], "position": [pitch], "offset": [0]},
                                                                  {"id": [2], "position": [yaw], "offset": [0]}]}}))
    time.sleep(1.6)


def snapshot(host):
    req = urllib.request.Request("http://%s:8080/snapshot?topic=/image_raw" % host,
                                 headers={"Cache-Control": "no-store"})
    return urllib.request.urlopen(req, timeout=5.).read()


def detect(detector, frame):
    result = detector.detect(frame, "jpeg")
    crosswalks = []
    for item in result.objects:
        if item.label != "crosswalk":
            continue
        x, y, w, h = item.box
        crosswalks.append({"box": [int(x), int(y), int(w), int(h)], "score": round(item.score, 3),
                           "bottom": int(y + h), "aspect": round(w / max(h, 1), 2)})
    crosswalks.sort(key=lambda item: item["bottom"])
    reds = [{"box": [int(o.box[0]), int(o.box[1]), int(o.box[2]), int(o.box[3])], "score": round(o.score, 3)}
            for o in result.objects if o.label == "red"]
    return {"red": result.red, "reds": reds, "crosswalks": crosswalks, "size": list(result.image_size)}


def views(host):
    config = load_config()
    settings = config["traffic_light"]
    detector = TrafficLightDetector(confidence=settings["confidence"], zoom=settings["zoom"],
                                    zoom_confidence=settings["zoom_confidence"],
                                    candidate_confidence=settings["candidate_confidence"],
                                    red_aspect=settings["red_aspect"])
    car = Car(host)
    records = {}
    try:
        battery.ensure_ok(car.ws)
        for name, pitch in (("forward", 1450), ("dip", 1600), ("lift", 1380)):
            servo(car, pitch)
            frame = snapshot(host)
            (OUT / ("view_%s.jpg" % name)).write_bytes(frame)
            info = detect(detector, frame)
            records[name] = {"pitch": pitch, **info}
            print("%-8s pitch=%d red=%s reds=%s crosswalks_bottom=%s" % (
                name, pitch, info["red"], info["reds"], [c["bottom"] for c in info["crosswalks"]]))
    finally:
        car.close()
    (OUT / "views.json").write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
    print("wrote", OUT / "views.json")


def speed(host, mps, seconds):
    car = Car(host)
    try:
        battery.ensure_ok(car.ws)
        _, scan = car.read(0.6)
        before = car.front_range(scan)
        print("front before:", before)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            car.twist(mps, 0.)
            time.sleep(0.1)
        car.stop()
        time.sleep(0.7)
        _, scan = car.read(0.6)
        after = car.front_range(scan)
        print("front after:", after)
        if before and after:
            moved = before - after
            print("commanded %.2f m, lidar moved %.3f m -> real speed %.3f m/s, ratio %.2f"
                  % (mps * seconds, moved, moved / seconds, (moved / seconds) / mps))
    finally:
        car.close()


def steps(host, count, step_m):
    settings = load_config()["traffic_light"]
    detector = TrafficLightDetector(confidence=settings["confidence"], zoom=settings["zoom"],
                                    zoom_confidence=settings["zoom_confidence"],
                                    candidate_confidence=settings["candidate_confidence"],
                                    red_aspect=settings["red_aspect"])
    car = Car(host)
    rows = []
    try:
        battery.ensure_ok(car.ws)
        for index in range(count):
            servo(car, 1600)
            pose, scan = car.read(0.8)
            front = car.front_range(scan)
            if front is not None and front < FRONT_MIN:
                print("前方 %.2f m 过近，停止" % front)
                break
            frame = snapshot(host)
            (OUT / ("step_%02d.jpg" % index)).write_bytes(frame)
            info = detect(detector, frame)
            near = [c for c in info["crosswalks"] if c["aspect"] >= 1.8 and c["bottom"] >= 300]
            row = max((c["bottom"] for c in near), default=None)
            rows.append({"index": index, "front": front, "row": row,
                         "crosswalks": [c["bottom"] for c in info["crosswalks"]], "red": info["red"]})
            print("step %d front=%s row=%s all=%s red=%s" % (
                index, front, row, [c["bottom"] for c in info["crosswalks"]], info["red"]))
            if index == count - 1:
                break
            car.step(step_m, 0.06)
        current = rows[-1]["row"] if rows else None
        if current is not None and current >= 470:
            print("row 已到 %s，停止采样" % current)
    finally:
        car.close()
    (OUT / "steps.json").write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")
    print("wrote", OUT / "steps.json")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    command = sys.argv[1]
    if command == "views":
        views(sys.argv[2] if len(sys.argv) > 2 else "192.168.167.118")
    elif command == "speed":
        speed(sys.argv[4] if len(sys.argv) > 4 else "192.168.167.118",
              float(sys.argv[2]), float(sys.argv[3]))
    elif command == "steps":
        steps(sys.argv[4] if len(sys.argv) > 4 else "192.168.167.118",
              int(sys.argv[2]), float(sys.argv[3]))
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
