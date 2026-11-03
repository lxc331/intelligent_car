# -*- coding: utf-8 -*-
"""Position the car with odometry and calibrate the crosswalk distance model.

Subcommands (all motion is small-step, twist published at 10 Hz, odom drained):
  status
  toward <step_m> <speed> <max_steps>   drive forward until the stop line is reached
  back <meters> <speed>                 reverse a measured distance
  sample <steps> <step_m> <pitch>       forward steps capturing distance samples
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
TOPIC_ODOM = "/odom"

sys.path.insert(0, r"D:\项目\intelligent_car-main\app")

from carapp.config import load_config  # noqa: E402
from carapp.core.ws import WebSocket  # noqa: E402
from carapp.features.traffic_light import _build  # noqa: E402


class Car:
    def __init__(self):
        self.ws = WebSocket(HOST, 9090, timeout=3.0)
        self.ws.send_text(json.dumps({"op": "subscribe", "topic": TOPIC_ODOM, "id": "odom"}))

    def pose(self, seconds=1.0):
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
        started = time.monotonic()
        while time.monotonic() - started < seconds:
            self.twist(speed)
            time.sleep(0.1)
        self.stop()

    def step(self, meters, speed=0.10):
        """Drive ``meters`` (negative = reverse) using the measured 0.98 ratio."""
        direction = 1.0 if meters >= 0 else -1.0
        seconds = abs(meters) / max(abs(speed) * .98, .02)
        self.drive(direction * abs(speed), seconds)

    def stop(self, repeat=10):
        for _ in range(repeat):
            self.twist(0.0)
            time.sleep(0.05)

    def close(self):
        try:
            self.stop()
        finally:
            self.ws.close()


def servo(joint_id, position):
    ws = WebSocket(HOST, 9090, timeout=3.0)
    ws.send_text(json.dumps({"op": "publish", "topic": "/ros_robot_controller/pwm_servo/set_state",
        "msg": {"duration": .8, "state": [{"id": [joint_id], "position": [position], "offset": [0]}]}}))
    ws.close()


def grab(name, detector):
    OUT.mkdir(parents=True, exist_ok=True)
    url = "http://%s:8080/snapshot?topic=/image_raw" % HOST
    req = urllib.request.Request(url, headers={"Cache-Control": "no-store"})
    data = urllib.request.urlopen(req, timeout=6.0).read()
    (OUT / ("%s.jpg" % name)).write_bytes(data)
    result = detector.detect(data, "jpeg")
    return result


def nearest_crosswalk(result):
    rows = []
    for obj in result.objects:
        if obj.label != "crosswalk":
            continue
        x, y, w, h = obj.box
        rows.append({"score": round(float(obj.score), 3), "box": [x, y, w, h],
                     "bottom": y + h, "center": y + h / 2.})
    rows.sort(key=lambda row: row["bottom"], reverse=True)
    return rows[0] if rows else None


def command_toward(car, detector, step_m, speed, max_steps):
    miss_streak = 0
    start = car.pose()
    for index in range(max_steps):
        car.step(step_m, speed)
        time.sleep(0.7)
        result = grab("toward_%02d" % index, detector)
        near = nearest_crosswalk(result)
        pose = car.pose()
        travel = math.hypot(pose[0] - start[0], pose[1] - start[1]) if pose and start else 0.
        print("step %2d travel %.2f m  crosswalk %s  red %s"
              % (index, travel, "-" if near is None else "bottom=%d" % near["bottom"], result.red))
        if near is None:
            miss_streak += 1
            if miss_streak >= 2:
                print("crosswalk left the view twice -> assume the stop line is reached")
                return
        else:
            miss_streak = 0
            if near["bottom"] >= 462:
                print("near crosswalk reached the bottom edge -> stop line reached")
                return
    print("max steps reached without reaching the line")


def command_back(car, meters, speed):
    pose = car.pose()
    car.step(-meters, speed)
    time.sleep(0.8)
    after = car.pose()
    moved = math.hypot(after[0] - pose[0], after[1] - pose[1]) if pose and after else 0.
    print("reverse requested %.2f m, odom moved %.2f m" % (meters, moved))


def command_sample(car, detector, steps, step_m, pitch):
    servo(1, pitch)
    time.sleep(1.8)
    start = car.pose()
    rows = []
    for index in range(steps + 1):
        if index:
            car.step(step_m, 0.10)
            time.sleep(0.7)
        pose = car.pose()
        travel = math.hypot(pose[0] - start[0], pose[1] - start[1]) if pose and start else 0.
        result = grab("sample_%02d" % index, detector)
        near = nearest_crosswalk(result)
        rows.append({"index": index, "travel": round(travel, 3),
                     "bottom": None if near is None else near["bottom"],
                     "box": None if near is None else near["box"],
                     "red": result.red})
        print("sample %2d travel %.3f m  %s  red=%s"
              % (index, travel, "bottom=None" if near is None else "bottom=%d" % near["bottom"], result.red))
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "samples.json").write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")
    print("samples ->", OUT / "samples.json")


def main():
    command = sys.argv[1] if len(sys.argv) > 1 else "status"
    detector = _build(load_config())[0]
    car = Car()
    try:
        if command == "status":
            print("odom", car.pose(), "| line to stop at: see toward")
        elif command == "toward":
            step_m = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
            speed = float(sys.argv[3]) if len(sys.argv) > 3 else 0.10
            max_steps = int(sys.argv[4]) if len(sys.argv) > 4 else 20
            command_toward(car, detector, step_m, speed, max_steps)
        elif command == "back":
            meters = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
            speed = float(sys.argv[3]) if len(sys.argv) > 3 else 0.08
            command_back(car, meters, speed)
        elif command == "sample":
            steps = int(sys.argv[2]) if len(sys.argv) > 2 else 10
            step_m = float(sys.argv[3]) if len(sys.argv) > 3 else 0.3
            pitch = int(sys.argv[4]) if len(sys.argv) > 4 else 1650
            command_sample(car, detector, steps, step_m, pitch)
        else:
            raise SystemExit("unknown command: %s" % command)
    finally:
        car.close()
        print("stopped")


if __name__ == "__main__":
    main()
