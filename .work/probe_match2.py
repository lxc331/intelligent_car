# -*- coding: utf-8 -*-
"""Match probe with a fallback chain so one missing frame cannot hide the result."""
import json
import math
import time

import numpy as np
import rclpy
import yaml
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

SESSION = "/tmp/carapp_nav_f01a468f18324c0d9d4c7d375995067e"

rclpy.init()
node = Node("match_probe2")
buffer = Buffer()
TransformListener(buffer, node)
scan_box = {}


def on_scan(message):
    scan_box["scan"] = message


node.create_subscription(LaserScan, "/scan_raw", on_scan, 10)
end = time.monotonic() + 8.
while time.monotonic() < end and "scan" not in scan_box:
    rclpy.spin_once(node, timeout_sec=.05)
# Discovery needs a few seconds; keep spinning so /tf and /tf_static arrive.
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=.05)
scan = scan_box.get("scan")
frames = buffer.all_frames_as_yaml()
print("scan frame", scan.header.frame_id if scan else None,
      "has frame", scan is not None and (scan.header.frame_id + ":") in frames)
print("frames with lidar:", [line.split(":")[0] for line in frames.splitlines() if "lidar" in line and line.endswith(":")])
if scan is not None:
    for candidate in ("lidar_frame", "lidar_link", "laser_frame", "base_link", "base_footprint"):
        try:
            transform = buffer.lookup_transform("map", candidate, rclpy.time.Time()).transform
            print("ok", candidate, round(transform.translation.x, 3), round(transform.translation.y, 3))
        except Exception as exc:
            print("fail", candidate, type(exc).__name__, str(exc)[:80])
node.destroy_node()
rclpy.shutdown()
