# -*- coding: utf-8 -*-
"""Install and start the pose recorder in the car container (detached)."""
import base64
import json
import subprocess

RECORDER = r'''
import csv, math, os, time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

OUT = "/tmp/drive_track.csv"
STOP = "/tmp/stop_track"


def yaw_of(q):
    return math.atan2(2. * (q.w * q.z + q.x * q.y), 1. - 2. * (q.y * q.y + q.z * q.z))


rclpy.init()
node = Node("drive_track_recorder")
buffer = Buffer()
TransformListener(buffer, node)
latest = {"t": 0., "x": 0., "y": 0., "yaw": 0.}


def on_pose(message):
    p = message.pose.pose.position
    latest.update(t=time.monotonic(), x=p.x, y=p.y, yaw=yaw_of(message.pose.pose.orientation))


node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", on_pose, 10)
if os.path.exists(STOP):
    os.remove(STOP)
start = time.monotonic()
rows = []
with open(OUT, "w", newline="") as handle:
    import csv
    writer = csv.writer(handle)
    writer.writerow(["time", "x", "y", "yaw"])
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=.2)
    while time.monotonic() - start < 1800.:
        if os.path.exists(STOP):
            break
        rclpy.spin_once(node, timeout_sec=.02)
        record = None
        for frame in ("base_footprint", "base_link"):
            try:
                transform = buffer.lookup_transform("map", frame, rclpy.time.Time()).transform
                record = (time.monotonic(), transform.translation.x, transform.translation.y,
                          yaw_of(transform.rotation))
                break
            except Exception:
                pass
        if record is None and time.monotonic() - latest["t"] < 2.:
            record = (latest["t"], latest["x"], latest["y"], latest["yaw"])
        if record and (not rows or math.dist(record[1:3], rows[-1][1:3]) > .01
                       or record[0] - rows[-1][0] > .5):
            rows.append(record)
            writer.writerow([round(record[0], 3), round(record[1], 4),
                             round(record[2], 4), round(record[3], 4)])
            if len(rows) % 10 == 0:
                handle.flush()
        time.sleep(.1)
print("samples", len(rows), flush=True)
node.destroy_node()
rclpy.shutdown()
'''


def main():
    payload = base64.b64encode(RECORDER.encode("utf-8")).decode("ascii")
    script = ("echo %s | base64 -d > /tmp/record_track.py; chmod 644 /tmp/record_track.py; "
              "rm -f /tmp/stop_track /tmp/record_track.log; "
              "setsid nohup python3 /tmp/record_track.py > /tmp/record_track.log 2>&1 & "
              "sleep 1; pgrep -af 'record_track.py' | head -3; ls -l /tmp/drive_track.csv 2>/dev/null || echo 'no csv yet'"
              % payload)
    out = subprocess.run(["bash", "-lc", script], capture_output=True, text=True)
    print(out.stdout.strip() or "(no output)")
    print(json.dumps(dict(stderr=out.stderr.strip()[-300:])))


if __name__ == "__main__":
    main()
