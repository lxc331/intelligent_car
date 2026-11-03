# -*- coding: utf-8 -*-
"""List TF frames and the scan header frame."""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node("frames_probe")
buffer = Buffer()
TransformListener(buffer, node)
scan_info = {}
node.create_subscription(LaserScan, "/scan_raw",
                        lambda message: scan_info.setdefault("frame", message.header.frame_id), 10)
end = time.monotonic() + 6.
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=.05)
print("scan frame:", scan_info.get("frame"))
print(buffer.all_frames_as_yaml()[:1200])
node.destroy_node()
rclpy.shutdown()
