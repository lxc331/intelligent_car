# -*- coding: utf-8 -*-
"""Read-only probe: lidar/SLAM stack, current pose, container state (host/port via argv)."""
import sys

import paramiko

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.167.136"
BASH = (
    "source /opt/ros/humble/setup.bash; "
    "ros2 topic list | grep -E 'scan_raw|/odom|cmd_vel|/map'; echo '--- hz scan'; "
    "timeout 3 ros2 topic hz /scan_raw 2>/dev/null | tail -2; echo '--- hz odom'; "
    "timeout 3 ros2 topic hz /odom 2>/dev/null | tail -2; echo '--- nodes'; "
    "ros2 node list | grep -iE 'slam|map_server' ; echo '--- map'; "
    "timeout 2 ros2 topic echo /map --once --field info 2>&1 | head -6; echo '--- odom once'; "
    "timeout 3 ros2 topic echo /odom --once --field pose.pose 2>&1 | head -8"
)
CMD = 'sudo -n docker exec -u ubuntu ClawPi bash -lc "%s"' % BASH.replace('"', '\\"')

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(HOST, username="pi", password="raspberrypi", timeout=6)
stdin, stdout, stderr = client.exec_command(CMD, timeout=60)
print(stdout.read().decode("utf-8", "replace"))
err = stderr.read().decode("utf-8", "replace").strip()
if err:
    print("STDERR:", err[:800])
client.close()
