# -*- coding: utf-8 -*-
"""Record the car pose in the map frame while the user drives one lap.

    python3 record_track.py 900      # hard stop after N seconds
    touch /tmp/stop_track            # or stop from another shell

Writes /tmp/drive_track.csv with time,x,y,yaw (map frame, base_footprint).
"""
import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

OUT = "/tmp/drive_track.csv"
STOP = "/tmp/stop_track"


def yaw_of(quaternion):
    return math.atan2(2. * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1. - 2. * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def main():
    limit = float(sys.argv[1]) if len(sys.argv) > 1 else 900.
    if os.path.exists(STOP):
        os.remove(STOP)
    rclpy.init()
    node = Node("drive_track_recorder")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    latest = {"t": 0., "x": 0., "y": 0., "yaw": 0., "src": ""}

    def on_pose(message):
        position = message.pose.pose.position
        latest.update(t=time.monotonic(), x=position.x, y=position.y,
                      yaw=yaw_of(message.pose.pose.orientation), src="amcl")

    node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", on_pose, 10)
    start = time.monotonic()
    rows, frames = [], 0
    with open(OUT, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time", "x", "y", "yaw"])
        while time.monotonic() - start < limit:
            if os.path.exists(STOP):
                break
            rclpy.spin_once(node, timeout_sec=.02)
            record = None
            try:
                transform = buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                record = (time.monotonic(), translation.x, translation.y, yaw_of(rotation), "tf")
            except Exception:  # transform not there yet, or base_footprint not published
                try:
                    transform = buffer.lookup_transform("map", "base_link", rclpy.time.Time())
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    record = (time.monotonic(), translation.x, translation.y, yaw_of(rotation), "tf_base")
                except Exception:
                    if time.monotonic() - latest["t"] < 2.:
                        record = (latest["t"], latest["x"], latest["y"], latest["yaw"], latest["src"])
            if record and (not rows or math.dist(record[1:3], rows[-1][1:3]) > .01
                           or record[0] - rows[-1][0] > .5):
                rows.append(record)
                writer.writerow([round(record[0], 3), round(record[1], 4),
                                 round(record[2], 4), round(record[3], 4)])
                if len(rows) % 10 == 0:
                    handle.flush()
            frames += 1
            time.sleep(.1)
    print("samples", len(rows), "loops", frames, "sources",
          sorted({row[4] for row in rows}) if rows else "-", flush=True)
    if rows:
        print("start", rows[0][1:4], "end", rows[-1][1:4], flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
