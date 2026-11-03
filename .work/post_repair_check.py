# -*- coding: utf-8 -*-
"""Post-repair sanity check: SSH reachability, ROS nodes, scan/odom/velocity health."""
import sys
import time

import paramiko

CAR = sys.argv[1] if len(sys.argv) > 1 else "192.168.167.136"
BEGIN = "source /opt/ros/humble/setup.bash;"


def dexec(client, inner, timeout=40):
    cmd = "sudo -n docker exec -u ubuntu ClawPi bash -lc %s" % repr(BEGIN + " " + inner)
    out = client.exec_command(cmd, timeout=timeout)
    return out[1].read().decode("utf-8", "replace") + out[2].read().decode("utf-8", "replace")


c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(CAR, username="pi", password="raspberrypi", timeout=10)
try:
    print("host:", c.exec_command("hostname; cat /proc/device-tree/serial-number | tr -d '\\0'; echo", timeout=20)[1]
          .read().decode().replace("\n", " "))
    print("uptime:", c.exec_command("uptime", timeout=20)[1].read().decode().strip())
    print("docker:", c.exec_command("sudo -n docker ps --format '{{.Names}} {{.Status}}'", timeout=20)[1].read().decode().strip())
    print("carto leftovers:", dexec(c, "pgrep -af cartographer || echo none").strip()[:300])
    print("nodes:", dexec(c, "timeout 6 ros2 node list 2>/dev/null | tr '\\n' ' '").strip()[:600])
    print("scan hz:", dexec(c, "timeout 6 ros2 topic hz /scan_raw 2>/dev/null | tail -1").strip())
    print("odom hz:", dexec(c, "timeout 6 ros2 topic hz /odom 2>/dev/null | tail -1").strip())
    print("odom pose:", dexec(c, "timeout 4 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null | tr '\\n' ' '").strip())
    print("scan sample:", dexec(c, "timeout 4 ros2 topic echo /scan_raw --once --field ranges 2>/dev/null | head -c 400").strip())
finally:
    c.close()
