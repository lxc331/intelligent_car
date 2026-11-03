# -*- coding: utf-8 -*-
"""Raise the camera gimbal over rosbridge, then snapshot + detect.

Usage: py -3 .work/gimbal.py <pitch> [yaw]
Publishes /ros_robot_controller/pwm_servo/set_state exactly like the App
(features/base.py publish_servo) and saves a frame tagged with the target pitch.
"""
import importlib.util
import json
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "app"))

from carapp.core.ws import WebSocket  # noqa: E402

HOST = "192.168.167.136"
PITCH = int(sys.argv[1]) if len(sys.argv) > 1 else 1600
YAW = int(sys.argv[2]) if len(sys.argv) > 2 else None
OUT = ROOT / ".work" / "traffic_light_live"
OUT.mkdir(parents=True, exist_ok=True)

servo_topic = "/ros_robot_controller/pwm_servo/set_state"
state = [{"id": [1], "position": [PITCH], "offset": [0]}]
if YAW is not None:
    state.append({"id": [2], "position": [YAW], "offset": [0]})
message = {"op": "publish", "topic": servo_topic,
           "msg": {"duration": .8, "state": state}}

sock = WebSocket(HOST, 9090)
try:
    sock.send_text(json.dumps(message))
    print("sent pitch=%d%s" % (PITCH, "" if YAW is None else " yaw=%d" % YAW))
finally:
    time.sleep(1.6)
    sock.close()

spec = importlib.util.spec_from_file_location("probe", ROOT / ".work" / "live_traffic_raw_probe.py")
probe = importlib.util.module_from_spec(spec)
_argv = sys.argv[1:]
sys.argv = sys.argv[:1]
spec.loader.exec_module(probe)
sys.argv = sys.argv[:1] + _argv
run = probe.load_runner()

url = "http://%s:8080/snapshot?topic=/image_raw" % HOST
request = urllib.request.Request(url, headers={"Cache-Control": "no-store"})
with urllib.request.urlopen(request, timeout=5.) as response:
    payload = response.read()
path = OUT / ("pitch_%d.jpg" % PITCH)
path.write_bytes(payload)
items, size = run(payload)
print("saved %s (%dx%d)" % (path, size[0], size[1]))
for item in items:
    print("   %-9s %.3f box=%s area=%d" % (item.label, item.score, item.box,
                                           item.box[2] * item.box[3]))
if not items:
    print("   (nothing above 0.05)")
