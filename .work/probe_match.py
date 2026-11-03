# -*- coding: utf-8 -*-
"""Sanity-check AMCL: how many lidar hits land on mapped obstacles."""
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

MAP = "/tmp/carapp_nav_f01a468f18324c0d9d4c7d375995067e/map.yaml"


def main():
    meta = yaml.safe_load(open(MAP, encoding="utf-8"))
    image = Image.open("/tmp/carapp_nav_f01a468f18324c0d9d4c7d375995067e/" + meta["image"])
    grid = np.array(image.convert("L"))
    resolution = float(meta["resolution"])
    ox, oy, _ = meta["origin"]
    rclpy.init()
    node = Node("match_probe")
    buffer = Buffer()
    TransformListener(buffer, node)
    scans = []

    def on_scan(message):
        scans.append(message)

    node.create_subscription(LaserScan, "/scan_raw", on_scan, 10)
    end = time.monotonic() + 6.
    while time.monotonic() < end and not scans:
        rclpy.spin_once(node, timeout_sec=.05)
    time.sleep(.5)
    rclpy.spin_once(node, timeout_sec=.05)
    scan = scans[-1]
    transform = buffer.lookup_transform("map", scan.header.frame_id, rclpy.time.Time()).transform
    q = transform.rotation
    yaw = math.atan2(2. * (q.w * q.z + q.x * q.y), 1. - 2. * (q.y * q.y + q.z * q.z))
    tx, ty = transform.translation.x, transform.translation.y
    hits = occupied = inside = 0
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        px = tx + math.cos(yaw) * distance * math.cos(angle) - math.sin(yaw) * distance * math.sin(angle)
        py = ty + math.sin(yaw) * distance * math.cos(angle) + math.cos(yaw) * distance * math.sin(angle)
        column = int((px - ox) / resolution)
        row = int((py - oy) / resolution)
        hits += 1
        if 0 <= column < grid.shape[1] and 0 <= row < grid.shape[0]:
            inside += 1
            value = grid[grid.shape[0] - 1 - row, column]
            if value < 60:
                occupied += 1
    print(json.dumps(dict(hits=hits, inside_ratio=round(inside / max(1, hits), 3),
                          on_obstacle=round(occupied / max(1, hits), 3),
                          pose=(round(tx, 3), round(ty, 3), round(math.degrees(yaw), 1)))), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
