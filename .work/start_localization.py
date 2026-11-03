# -*- coding: utf-8 -*-
"""Bring up map_server + AMCL only, then park the initial pose near node 1.

Runs inside the car container. Prints JSON lines so the PC side can parse.
No navigation nodes are started, so the remote-control page keeps working.
"""
import json
import math
import os
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

SESSION = "/tmp/carapp_nav_f01a468f18324c0d9d4c7d375995067e"
MAP = SESSION + "/map.yaml"
PARAMS = SESSION + "/nav2.yaml"
NODE1 = (-1.0869, -1.376)
YAW = math.radians(47.7)


def main():
    log = open("/tmp/localization.log", "w")
    cmd = ["ros2", "launch", "nav2_bringup", "localization_launch.py",
           "map:=" + MAP, "params_file:=" + PARAMS, "use_sim_time:=false"]
    process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    print(json.dumps(dict(started=process.pid, cmd=" ".join(cmd))), flush=True)
    time.sleep(12.0)
    if process.poll() is not None:
        print(json.dumps(dict(error="launch exited", code=process.returncode)), flush=True)
        with open("/tmp/localization.log", "r", errors="replace") as handle:
            print(handle.read()[-1200:], flush=True)
        return
    rclpy.init()
    node = Node("initial_pose_sender")
    publisher = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.pose.pose.position.x = NODE1[0]
    message.pose.pose.position.y = NODE1[1]
    message.pose.pose.orientation.z = math.sin(YAW / 2.)
    message.pose.pose.orientation.w = math.cos(YAW / 2.)
    message.pose.covariance = [0.16, 0., 0., 0., 0., 0.,
                               0., 0.16, 0., 0., 0., 0.,
                               0., 0., 0., 0., 0., 0.,
                               0., 0., 0., 0., 0., 0.,
                               0., 0., 0., 0., 0., 0.,
                               0., 0., 0., 0., 0., 1.44]
    for _ in range(6):
        message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)
        time.sleep(.2)
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(dict(initial_pose="sent", x=NODE1[0], y=NODE1[1], yaw_deg=47.7)), flush=True)


if __name__ == "__main__":
    main()
