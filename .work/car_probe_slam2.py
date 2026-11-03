# -*- coding: utf-8 -*-
"""Read-only probe: apt availability of cartographer / explore packages, vendor slam config, lidar spec."""
import sys

import paramiko

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.167.136"
BASH = r"""
source /opt/ros/humble/setup.bash
echo '=== apt candidates'
apt-cache policy ros-humble-cartographer-ros ros-humble-explore-lite ros-humble-slam-toolbox 2>&1 | head -20
echo '=== vendor slam config'
ls -l /home/ubuntu/ros2_ws/src/slam/config/ 2>&1
cat /home/ubuntu/ros2_ws/src/slam/config/slam.yaml 2>&1 | head -80
echo '=== lidar driver info'
ps aux | grep -iE 'sclidar|lidar' | grep -v grep | head -5
echo '=== imu topics hz'
timeout 3 ros2 topic hz /imu 2>/dev/null | tail -1
echo '=== disk'
df -h / | tail -1
"""
CMD = 'sudo -n docker exec -u ubuntu ClawPi bash -lc "%s"' % BASH.replace('"', '\\"')

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(HOST, username="pi", password="raspberrypi", timeout=6)
stdin, stdout, stderr = client.exec_command(CMD, timeout=90)
print(stdout.read().decode("utf-8", "replace"))
err = stderr.read().decode("utf-8", "replace").strip()
if err:
    print("STDERR:", err[:600])
client.close()
