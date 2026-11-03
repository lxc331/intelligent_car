# -*- coding: utf-8 -*-
"""Read-only check: car stopped, mapping nodes alive, map topic size."""
import paramiko

BEGIN = "source /opt/ros/humble/setup.bash;"


def dexec(client, inner, timeout=40):
    cmd = "sudo -n docker exec -u ubuntu ClawPi bash -lc %s" % repr(BEGIN + " " + inner)
    out = client.exec_command(cmd, timeout=timeout)
    return out[1].read().decode("utf-8", "replace") + out[2].read().decode("utf-8", "replace")


c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect("192.168.167.136", username="pi", password="raspberrypi", timeout=8)
try:
    print("carto processes:", dexec(c, "pgrep -fc cartographer_node").strip())
    print("grid processes:", dexec(c, "pgrep -fc cartographer_occupancy_grid_node").strip())
    print("map publisher:", dexec(c, "ros2 topic info /map 2>/dev/null | tr '\\n' ' '").strip())
    print("odom:", dexec(c, "timeout 3 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null | tr '\\n' ' '").strip())
    print("cmd_vel:", dexec(c, "timeout 3 ros2 topic echo /controller/cmd_vel --once 2>/dev/null | tr '\\n' ' '").strip())
finally:
    c.close()
