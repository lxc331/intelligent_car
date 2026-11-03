# -*- coding: utf-8 -*-
"""Runs inside the car container: odom pose plus nearest obstacle per sector."""
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

rclpy.init(args=None)
node = Node('peek')
state = {}
node.create_subscription(Odometry, '/odom', lambda m: state.update(odom=m), 10)
node.create_subscription(LaserScan, '/scan_raw', lambda m: state.update(scan=m), 10)
start = time.time()
while rclpy.ok() and time.time() - start < 6.0:
    rclpy.spin_once(node, timeout_sec=0.1)

odom = state.get('odom')
if odom is None:
    print('no odom')
else:
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    print('odom x=%.3f y=%.3f yaw=%.1f' % (p.x, p.y, yaw))

scan = state.get('scan')
if scan is None:
    print('no scan')
else:
    vals = scan.ranges
    centre = len(vals) // 2

    def sector(low_deg, high_deg):
        lo = centre + int(math.radians(low_deg) / scan.angle_increment)
        hi = centre + int(math.radians(high_deg) / scan.angle_increment)
        seg = [v for v in vals[max(0, lo):max(0, hi)] if math.isfinite(v) and v > 0.02]
        return round(min(seg), 3) if seg else None

    print('front +-15', sector(-15, 15))
    print('front +-35', sector(-35, 35))
    print('left', sector(40, 100), 'right', sector(-100, -40))
