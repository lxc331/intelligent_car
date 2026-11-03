# -*- coding: utf-8 -*-
"""Debug why the lane follower does not receive /image_raw."""
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan


class Probe(Node):
    def __init__(self):
        super().__init__("probe_inputs")
        self.counts = {"odom": 0, "scan": 0, "image": 0}
        self.meta = {}
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(LaserScan, "/scan_raw", self.on_scan, 20)
        self.create_subscription(Image, "/image_raw", self.on_image, 2)

    def on_odom(self, m):
        self.counts["odom"] += 1

    def on_scan(self, m):
        self.counts["scan"] += 1

    def on_image(self, m):
        self.counts["image"] += 1
        self.meta = {"h": m.height, "w": m.width, "encoding": m.encoding,
                     "step": m.step, "len": len(m.data)}


rclpy.init()
node = Probe()
end = time.time() + 5
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
print(node.counts, node.meta)
node.destroy_node()
rclpy.shutdown()
