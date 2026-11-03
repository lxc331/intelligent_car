# -*- coding: utf-8 -*-
"""Just the frames we need: scan header plus any lidar/laser transform."""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node("frames_probe2")
buffer = Buffer()
TransformListener(buffer, node)
scan_frame = {}
node.create_subscription(LaserScan, "/scan_raw",
                         lambda message: scan_frame.setdefault("frame", message.header.frame_id), 10)
end = time.monotonic() + 5.
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=.05)
text = buffer.all_frames_as_yaml()
lines = text.splitlines()
print("scan frame:", scan_frame.get("frame"))
for index, line in enumerate(lines):
    if "lidar" in line or "laser" in line or "base_footprint:" in line:
        print("\n".join(lines[index:index + 4]))
node.destroy_node()
rclpy.shutdown()
