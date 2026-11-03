# -*- coding: utf-8 -*-
"""Drive-side measurements for the sandbox car.

Subcommands:
  status                          print current /odom pose
  speed <seconds> [reverse]       drive at config speeds, report measured m/s
  approach <steps> [reverse] [s]  crawl in reverse steps, capture a frame per step

Twist is published at 10 Hz while the car moves; odometry is only read while the
car is standing still, because reading blocks until the next message arrives.
"""
import json
import math
import sys
import time
import urllib.request
from pathlib import Path

HOST = "192.168.167.136"
PORT = 9090
OUT = Path(r"D:\项目\intelligent_car-main\.work\drive_calib")
SPEEDS = (0.06, 0.10, 0.12)
SAFE_MAX = 0.15
TOPIC_CMD = "/controller/cmd_vel"
TOPIC_ODOM = "/odom"

sys.path.insert(0, r"D:\项目\intelligent_car-main\app")

from carapp.core.ws import WebSocket  # noqa: E402


class Node:
    def __init__(self):
        self.ws = WebSocket(HOST, PORT, timeout=4.0)
        self.ws.send_text(json.dumps({"op": "subscribe", "topic": TOPIC_ODOM, "id": "odom"}))
        self.pose = None

    def twist(self, vx, wz=0.0):
        self.ws.send_text(json.dumps({
            "op": "publish",
            "topic": TOPIC_CMD,
            "msg": {"linear": {"x": vx, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": wz}},
        }))

    def read_pose(self, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
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
                self.pose = (float(position["x"]), float(position["y"]))
                return self.pose
        if self.pose is None:
            raise RuntimeError("没有收到 /odom 位姿")
        return self.pose

    def stop(self, repeat=10):
        for _ in range(repeat):
            self.twist(0.0, 0.0)
            time.sleep(0.05)

    def drive(self, speed, seconds):
        speed = max(-SAFE_MAX, min(SAFE_MAX, float(speed)))
        started = time.monotonic()
        while time.monotonic() - started < seconds:
            self.twist(speed, 0.0)
            time.sleep(0.1)
        self.stop()

    def close(self):
        try:
            self.stop()
        finally:
            self.ws.close()


def snapshot(name):
    OUT.mkdir(parents=True, exist_ok=True)
    url = "http://%s:8080/snapshot?topic=/image_raw" % HOST
    req = urllib.request.Request(url, headers={"Cache-Control": "no-store"})
    data = urllib.request.urlopen(req, timeout=6.0).read()
    path = OUT / ("%s.jpg" % name)
    path.write_bytes(data)
    return path, len(data)


def command_speed(node, seconds):
    seconds = float(seconds) if len(sys.argv) > 2 else 3.0
    sign = -1.0 if (len(sys.argv) > 3 and sys.argv[3] == "reverse") else 1.0
    for speed in SPEEDS:
        start = node.read_pose()
        node.drive(sign * speed, seconds)
        time.sleep(0.8)
        end = node.read_pose()
        travelled = math.hypot(end[0] - start[0], end[1] - start[1])
        measured = travelled / seconds
        print("cmd=%+.3f m/s  %.1f s  travelled=%.3f m  measured=%.3f m/s  (ratio %.2f)"
              % (sign * speed, seconds, travelled, measured, measured / speed if speed else 0.0))


def command_approach(node, steps):
    steps = int(steps) if len(sys.argv) > 2 else 4
    sign = -1.0 if (len(sys.argv) > 3 and sys.argv[3] == "reverse") else 1.0
    step_seconds = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0
    total = 0.0
    start = node.read_pose()
    path, size = snapshot("approach_00")
    print("step 0: distance 0.000 m, frame %s (%d B)" % (path.name, size))
    for index in range(steps):
        node.drive(sign * 0.10, step_seconds)
        time.sleep(0.8)
        end = node.read_pose()
        moved = math.hypot(end[0] - start[0], end[1] - start[1])
        total += moved
        path, size = snapshot("approach_%02d" % (index + 1))
        print("step %d: distance %.3f m (moved %.3f m), frame %s (%d B)"
              % (index + 1, total, moved, path.name, size))
    print("total travel %.3f m" % total)


def main():
    command = sys.argv[1] if len(sys.argv) > 1 else "status"
    node = Node()
    try:
        if command == "status":
            print("odom", node.read_pose())
        elif command == "speed":
            command_speed(node, sys.argv[2] if len(sys.argv) > 2 else 3.0)
        elif command == "approach":
            command_approach(node, sys.argv[2] if len(sys.argv) > 2 else 4)
        else:
            raise SystemExit("unknown command: %s" % command)
    finally:
        node.close()
        print("stopped")


if __name__ == "__main__":
    main()
