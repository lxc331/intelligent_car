# -*- coding: utf-8 -*-
"""Read-only probe: SLAM/exploration packages, sensors, TF, odom/ekf details."""
import sys

import paramiko

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.167.136"
BASH = r"""
source /opt/ros/humble/setup.bash
echo '=== slam packages'
ros2 pkg list | grep -iE 'slam|cartograph|hector|rtab|octomap|explore|nav2_map|amcl|usb_cam|imu' | sort
echo '=== sensor topics'
ros2 topic list | grep -iE 'scan|imu|lidar|camera|image' | sort
echo '=== lidar info'
ros2 topic echo /scan_raw --once --field angle_min --field angle_max --field range_max --field ranges 2>/dev/null | head -3
timeout 3 ros2 topic hz /scan_raw 2>/dev/null | tail -1
echo '=== imu hz'
timeout 3 ros2 topic hz /imu/data_raw 2>/dev/null | tail -1
echo '=== tf frames'
timeout 3 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -5
echo '=== nodes'
ros2 node list
echo '=== ekf params'
ros2 param dump /ekf_node 2>/dev/null | head -30
echo '=== docker image files'
ls /home/pi/hiwonder-toolbox 2>/dev/null | head
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
