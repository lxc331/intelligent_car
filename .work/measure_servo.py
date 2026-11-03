# -*- coding: utf-8 -*-
"""Sweep one servo joint and record what the camera sees, for re-calibration.

Usage:
    py -3 .work\\measure_servo.py <joint> <pos1,pos2,...> [host]

joint: pitch | yaw | arm | gripper  (ids come from carapp.config DEFAULTS)
Frames are written to .work/servo_calib/<joint>_<position>.jpg and a montage
.work/servo_calib/<joint>_sheet.jpg is built when all frames are captured.
"""
import json
import sys
import time
import urllib.request
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

APP = Path(__file__).resolve().parents[1] / "app"
sys.path.insert(0, str(APP))
from carapp.config import DEFAULTS  # noqa: E402
from carapp.core.ws import WebSocket  # noqa: E402

OUT = Path(__file__).resolve().parent / "servo_calib"
FONT = ImageFont.truetype(r"C:\Windows\Fonts\msyh.ttc", 20)


def grab(host):
    request = urllib.request.Request("http://%s:8080/snapshot?topic=/image_raw" % host,
                                     headers={"Cache-Control": "no-store"})
    return urllib.request.urlopen(request, timeout=5.).read()


def move(host, servo_id, position):
    ws = WebSocket(host, 9090)
    ws.send_text(json.dumps({
        "op": "publish",
        "topic": "/ros_robot_controller/pwm_servo/set_state",
        "msg": {"duration": .8, "state": [{"id": [servo_id], "position": [position], "offset": [0]}]},
    }))


def sheet(joint, frames):
    tiles = []
    for position, path in frames:
        image = Image.open(path).convert("RGB").resize((480, 360))
        tile = Image.new("RGB", (480, 380), (16, 20, 24))
        tile.paste(image, (0, 0))
        ImageDraw.Draw(tile).text((8, 356), "%s = %d" % (joint, position), font=FONT, fill=(235, 240, 245))
        tiles.append(tile)
    width = 488 * len(tiles)
    montage = Image.new("RGB", (width, 380), (8, 10, 12))
    for index, tile in enumerate(tiles):
        montage.paste(tile, (index * 488, 0))
    out = OUT / ("%s_sheet.jpg" % joint)
    montage.save(out, quality=90)
    return out


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    joint = sys.argv[1]
    positions = [int(value) for value in sys.argv[2].replace("，", ",").split(",") if value.strip()]
    host = sys.argv[3] if len(sys.argv) > 3 else "192.168.167.118"
    servo_id = DEFAULTS["servo"]["ids"][joint]
    OUT.mkdir(parents=True, exist_ok=True)
    frames = []
    for position in positions:
        move(host, servo_id, position)
        time.sleep(1.8)
        path = OUT / ("%s_%d.jpg" % (joint, position))
        path.write_bytes(grab(host))
        frames.append((position, path))
        print("captured %s=%d" % (joint, position))
    print("sheet:", sheet(joint, frames))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
