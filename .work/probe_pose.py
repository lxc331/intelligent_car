# -*- coding: utf-8 -*-
"""Read AMCL pose + lifecycle state a few seconds after the initial pose."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2. * (q.w * q.z + q.x * q.y), 1. - 2. * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    node = Node("pose_probe")
    buffer = Buffer()
    TransformListener(buffer, node)
    poses = []

    def on_pose(message):
        p = message.pose.pose.position
        poses.append((p.x, p.y, math.degrees(yaw_of(message.pose.pose.orientation))))

    node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", on_pose, 10)
    end = time.monotonic() + 8.
    tf_ok = 0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
        try:
            buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            tf_ok += 1
        except Exception:
            pass
    print(json.dumps(dict(amcl_samples=len(poses), tf_ok=tf_ok,
                          last=poses[-1] if poses else None)), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
