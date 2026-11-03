# -*- coding: utf-8 -*-
"""Set the initial pose between node 1 and node 2, then report how well the scan fits."""
import json
import math
import time

import numpy as np
import rclpy
import yaml
from PIL import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

SESSION = "/tmp/carapp_nav_d512c9dfe68a4b4caec7a3063d0ae4d8"
NODE1 = (-1.0869, -1.376)
NODE2 = (-1.0277, -1.5079)
X = (NODE1[0] + NODE2[0]) / 2.
Y = (NODE1[1] + NODE2[1]) / 2.
YAW = math.atan2(NODE2[1] - NODE1[1], NODE2[0] - NODE1[0])


def yaw_of(q):
    return math.atan2(2. * (q.w * q.z + q.x * q.y), 1. - 2. * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    node = Node("initial_pose_between_nodes")
    publisher = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.pose.pose.position.x = X
    message.pose.pose.position.y = Y
    message.pose.pose.orientation.z = math.sin(YAW / 2.)
    message.pose.pose.orientation.w = math.cos(YAW / 2.)
    message.pose.covariance = [0.09, 0., 0., 0., 0., 0.,
                               0., 0.09, 0., 0., 0., 0.,
                               0., 0., 0., 0., 0., 0.,
                               0., 0., 0., 0., 0., 0.,
                               0., 0., 0., 0., 0., 0.,
                               0., 0., 0., 0., 0., 0.81]
    for _ in range(8):
        message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)
        time.sleep(.2)
    print(json.dumps(dict(initial_pose=[round(X, 3), round(Y, 3), round(math.degrees(YAW), 1)])),
          flush=True)

    buffer = Buffer()
    TransformListener(buffer, node)
    scan_box = {}

    def on_scan(msg):
        scan_box["scan"] = msg

    node.create_subscription(LaserScan, "/scan_raw", on_scan, 10)
    end = time.monotonic() + 9.
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
    scan = scan_box.get("scan")
    if scan is None:
        print(json.dumps(dict(error="no scan")), flush=True)
        return
    meta = yaml.safe_load(open(SESSION + "/map.yaml", encoding="utf-8"))
    grid = np.array(Image.open(SESSION + "/" + meta["image"]).convert("L"))
    resolution, (ox, oy, _) = float(meta["resolution"]), meta["origin"]
    transform = buffer.lookup_transform("map", scan.header.frame_id, rclpy.time.Time()).transform
    tx, ty = transform.translation.x, transform.translation.y
    yaw = yaw_of(transform.rotation)
    hits = occupied = 0
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        px = tx + distance * math.cos(yaw + angle)
        py = ty + distance * math.sin(yaw + angle)
        column, row = int((px - ox) / resolution), int((py - oy) / resolution)
        hits += 1
        if 0 <= column < grid.shape[1] and 0 <= row < grid.shape[0]:
            if grid[grid.shape[0] - 1 - row, column] < 60:
                occupied += 1
    print(json.dumps(dict(pose=[round(tx, 3), round(ty, 3), round(math.degrees(yaw), 1)],
                          hits=hits, hit_ratio=round(occupied / max(1, hits), 3))), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
