# -*- coding: utf-8 -*-
"""Closed-loop stop-line trial: crosswalk pixel row + odometry finish.

Usage: py -3 .work\auto_stop.py [max_seconds] [host] [--no-assume-red]

Reads the forward camera at ~4 Hz, tracks the crosswalk bottom row, integrates the
odometry travel, asks ``CrosswalkApproach`` for a speed limit and publishes it to
``/controller/cmd_vel``.  The black bollard is only used *afterwards* to report how
far the final position is from the calibrated stop line (1.078 m of lidar range).
"""
import io
import json
import math
import sys
import time
import urllib.request

import numpy as np
from PIL import Image
from pathlib import Path

OUT = Path(r"D:\项目\intelligent_car-main\.work\stop_trials")
TOPIC_CMD = "/controller/cmd_vel"
STOP_LINE_RANGE = 1.078

sys.path.insert(0, r"D:\项目\intelligent_car-main\app")
sys.path.insert(0, str(Path(__file__).resolve().parent))

import battery  # noqa: E402

from carapp.config import load_config  # noqa: E402
from carapp.core.junction_drive import CrosswalkApproach  # noqa: E402
from carapp.core.traffic_light_detector import TrafficLightDetector  # noqa: E402
from carapp.core.ws import WebSocket  # noqa: E402


class Car:
    def __init__(self, host):
        self.host = host
        self.ws = WebSocket(host, 9090, timeout=3.0)
        self.ws.send_text(json.dumps({"op": "subscribe", "topic": "/odom", "id": "odom"}))
        self.ws.send_text(json.dumps({"op": "subscribe", "topic": "/scan_raw", "id": "scan"}))
        self.pose = None
        self.scan = None
        self.yaw = None

    def pump(self, seconds=0.05):
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
                inner = (msg.get("pose") or {}).get("pose") or {}
                position = inner.get("position") or {}
                orientation = inner.get("orientation") or {}
                if "x" in position:
                    self.pose = (float(position["x"]), float(position["y"]))
                if orientation:
                    qx = float(orientation.get("x", 0.))
                    qy = float(orientation.get("y", 0.))
                    qz = float(orientation.get("z", 0.))
                    qw = float(orientation.get("w", 1.))
                    self.yaw = math.atan2(2. * (qw * qz + qx * qy),
                                          1. - 2. * (qy * qy + qz * qz))
            elif topic == "/scan_raw":
                self.scan = msg

    def twist(self, vx, wz=0.):
        self.ws.send_text(json.dumps({"op": "publish", "topic": TOPIC_CMD,
            "msg": {"linear": {"x": vx, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": float(wz)}}}))

    def stop(self, repeat=12):
        for _ in range(repeat):
            self.twist(0.0, 0.0)
            time.sleep(0.05)

    def forward_range(self):
        scan = self.scan or {}
        angle_min = float(scan.get("angle_min", 0.0))
        increment = float(scan.get("angle_increment", 0.0))
        rmin = float(scan.get("range_min", 0.0))
        rmax = float(scan.get("range_max", 0.0))
        half = math.radians(12.0)
        found = []
        for index, value in enumerate(scan.get("ranges") or []):
            if not isinstance(value, (int, float)):
                continue
            distance = float(value)
            if not math.isfinite(distance) or distance < rmin or distance > rmax:
                continue
            angle = math.atan2(math.sin(angle_min + index * increment),
                               math.cos(angle_min + index * increment))
            if abs(angle) <= half:
                found.append(distance)
        return min(found) if found else None

    def close(self):
        try:
            self.stop()
        finally:
            self.ws.close()


def servo(host, joint, position):
    ws = WebSocket(host, 9090, timeout=3.0)
    ws.send_text(json.dumps({"op": "publish", "topic": "/ros_robot_controller/pwm_servo/set_state",
        "msg": {"duration": .8, "state": [{"id": [joint], "position": [position], "offset": [0]}]}}))
    ws.close()


def grab(host, name=None):
    req = urllib.request.Request("http://%s:8080/snapshot?topic=/image_raw" % host,
                                 headers={"Cache-Control": "no-store"})
    data = urllib.request.urlopen(req, timeout=6.0).read()
    if name:
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT / ("%s.jpg" % name)).write_bytes(data)
    return data


def near_crosswalk(result, image=None):
    """Return (bottom_row, centre_x) of the nearest crosswalk band.

    The sandbox has curved white lane markings that the detector sometimes calls a
    crosswalk; a real zebra band at 0.3–0.6 m is flat (its bright centroid row does
    not drift across the box), while those markings are slanted, so the row spread
    is used as a cheap shape check.
    """
    near = []
    for obj in result.objects:
        if obj.label != "crosswalk":
            continue
        x, y, w, h = obj.box
        if not h or w / float(h) < 1.8:
            continue
        bottom = y + h
        if bottom < 250. or w < 120. or h < 6.:
            continue
        # 形状校验只对贴近画面下沿的大框生效：那正是弯道白线的位置；
        # 远处的真实斑马线（bottom < 400）受透视影响也会"斜"，不能一刀切。
        if image is not None and bottom >= 400. and w >= 200.:
            spread = _row_spread(image, obj.box)
            if spread is not None and spread > 3.0:
                continue
        near.append((bottom, x + w / 2.0, w))
    if not near:
        return None, None
    bottom, centre, width = max(near, key=lambda item: item[0])
    return bottom, centre


def _row_spread(image, box):
    x, y, w, h = [int(v) for v in box]
    crop = image.crop((x, y, x + w, y + h)).convert("L")
    grey = np.asarray(crop, dtype=np.float32)
    if grey.shape[0] < 4 or grey.shape[1] < 20:
        return None
    bright = grey > (float(grey.min()) + float(grey.max())) / 2.0
    rows = np.arange(grey.shape[0])[:, None]
    weights = bright.astype(np.float32)
    count = weights.sum(axis=0)
    centroid = np.where(count > 0, (rows * weights).sum(axis=0) / np.maximum(count, 1), np.nan)
    ok = ~np.isnan(centroid)
    if ok.sum() < 10:
        return None
    return float(np.std(centroid[ok]))



def near_crosswalk_row(result):
    return near_crosswalk(result)[0]


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 90.
    assume_red = "--no-assume-red" not in sys.argv
    cfg = load_config()
    cfg_host = (cfg.get("car") or {}).get("host") or "192.168.167.136"
    host = sys.argv[2] if len(sys.argv) > 2 and not sys.argv[2].startswith("--") else cfg_host
    approach = CrosswalkApproach.from_config(cfg)
    light = cfg["traffic_light"]
    detector = TrafficLightDetector(confidence=light["confidence"], zoom=light["zoom"],
                                    zoom_confidence=light["zoom_confidence"],
                                    candidate_confidence=light["candidate_confidence"],
                                    red_aspect=light["red_aspect"])
    track_pitch = (cfg.get("junction") or {}).get("track_pitch")
    dip_pitch = int(track_pitch if track_pitch else cfg["traffic_light"]["dip_pitch"])
    servo(host, 2, 1500)
    servo(host, 1, dip_pitch)
    time.sleep(1.5)
    guard = WebSocket(host, 9090, timeout=3.0)
    try:
        battery.ensure_ok(guard)
    finally:
        guard.close()
    car = Car(host)
    travel = 0.
    home_cx = None
    last = None
    started = time.monotonic()
    history = []
    final = None
    try:
        car.pump(1.0)
        last = car.pose
        home_yaw = car.yaw
        while time.monotonic() - started < seconds:
            car.pump(0.03)
            pose = car.pose
            if pose and last:
                travel += math.hypot(pose[0] - last[0], pose[1] - last[1])
            if pose:
                last = pose
            frame = grab(host)
            result = detector.detect(frame, "jpeg")
            image = Image.open(io.BytesIO(frame))
            row, centre = near_crosswalk(result, image)
            red = True if assume_red else bool(result.red)
            front = car.forward_range()
            if front is not None and front < 0.35:
                command = approach.hold_command("前方 %.2f m 有障碍，停车" % front)
            elif row is None:
                # 远处接近：还没看到近处斑马线，用正常接近速度，而不是低速试探
                command = approach.approach_command(approach.approach_speed,
                                                   "远处接近，等待斑马线进入画面")
            elif row >= 330. and not approach.triggered:
                # 进入 0.6 m 内先降到爬行，减少"一帧晚触发"造成的过冲
                command = approach.approach_command(approach.crawl_speed,
                                                   "接近停止线，降到爬行")
            else:
                command = approach.step(row, red, travel, time.monotonic())
            heading = 0.
            if car.yaw is not None and home_yaw is not None and command.speed:
                error = math.atan2(math.sin(home_yaw - car.yaw), math.cos(home_yaw - car.yaw))
                heading = max(-0.2, min(0.2, 1.5 * error))
            if centre is not None:
                if home_cx is None:
                    home_cx = centre
                lateral = (centre - home_cx) / 320.0
                if command.speed:
                    # 真实航向：保持近处斑马线的横向中心不动（/odom 的 yaw 只是指令积分，
                    # 相机自身也可能有固定偏置，所以用"相对起点的偏移"而不是绝对中线）
                    heading = max(-0.25, min(0.25, -0.8 * lateral))
            car.twist(command.speed or 0., heading)
            history.append({"travel": round(travel, 3), "row": row, "red": bool(result.red),
                            "speed": command.speed, "phase": command.phase, "note": command.note})
            print("travel %.2f m | row %s | red %s | %s %.2f | %s"
                  % (travel, row, result.red, command.phase, command.speed or 0., command.note))
            if command.phase == approach.HOLD:
                final = dict(command=command.phase, travel=round(travel, 3), row=row,
                             lidar_forward=None)
                break
            time.sleep(0.05)
    finally:
        car.stop()
        time.sleep(0.4)
        car.pump(0.3)
        gap = None
        lidar = car.forward_range()
        if lidar is not None:
            gap = round(lidar - STOP_LINE_RANGE, 3)
        grab(host, "final_%s" % time.strftime("%H%M%S"))
        car.close()
        OUT.mkdir(parents=True, exist_ok=True)
        straw = {"finished": final is not None, "travel": round(travel, 3),
                 "lidar_forward": None if lidar is None else round(lidar, 3),
                 "gap_to_stop_line": gap, "history": history[-40:]}
        (OUT / ("trial_%s.json" % time.strftime("%H%M%S"))).write_text(
            json.dumps(straw, ensure_ascii=False, indent=2), encoding="utf-8")
        print("\nRESULT finished=%s travel=%.2f m lidar=%.2f m gap=%s m"
              % (final is not None, travel,
                 lidar if lidar is not None else float("nan"), gap))


if __name__ == "__main__":
    main()
