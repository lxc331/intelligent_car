# -*- coding: utf-8 -*-
"""One-shot look at the car's current odom pose and nearby obstacles."""
import base64

import paramiko

CAR = "192.168.167.118"

_CODE = r'''
import math, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
rclpy.init(args=None)
node = Node('peek')
state = {}
node.create_subscription(Odometry, '/odom', lambda m: state.update(odom=m), 10)
node.create_subscription(LaserScan, '/scan_raw', lambda m: state.update(scan=m), 10)
deadline = 6.0
import time
t0 = time.time()
while rclpy.ok() and time.time() - t0 < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
odom = state.get('odom'); scan = state.get('scan')
if odom is not None:
    p = odom.pose.pose.position; q = odom.pose.pose.orientation
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    print('odom x=%.3f y=%.3f yaw=%.1f' % (p.x, p.y, yaw))
else:
    print('no odom')
if scan is not None:
    vals = scan.ranges
    c = len(vals) // 2
    def sector(deg):
        k = int(math.radians(deg) / scan.angle_increment)
        seg = [v for v in vals[max(0, c - k):c + k] if math.isfinite(v) and v > 0.02]
        return round(min(seg), 3) if seg else None
    print('front +-15', sector(15), 'front +-35', sector(35),
          'left +-90', sector(90), 'right +-90', sector(90))
else:
    print('no scan')
'''


def main():
    payload = base64.b64encode(_CODE.encode()).decode()
    inner = ("source /opt/ros/humble/setup.bash; source /home/ubuntu/ros2_ws/install/setup.bash; "
             "python3 -c \"import base64;exec(base64.b64decode('%s'))\"" % payload)
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(CAR, username="pi", password="raspberrypi", timeout=10)
    channel = client.exec_command("sudo -n docker exec -u ubuntu ClawPi bash -lc %s" % repr(inner), timeout=60)[1]
    print(channel.read().decode("utf-8", "replace"))
    client.close()


if __name__ == "__main__":
    main()
