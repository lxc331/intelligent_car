# -*- coding: utf-8 -*-
"""Check that the mapping stack is gone (read-only)."""
import paramiko

CAR = "192.168.167.136"
BEGIN = "source /opt/ros/humble/setup.bash; source /home/ubuntu/ros2_ws/install/setup.bash;"

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(CAR, username="pi", password="raspberrypi", timeout=10)


def run(cmd, timeout=40):
    o = c.exec_command(cmd, timeout=timeout)
    return (o[1].read() + o[2].read()).decode("utf-8", "replace").strip()


inner = BEGIN + " ros2 node list | grep -Ei 'slam|map_saver' || echo NO_SLAM_NODE"
print("nodes:", run("sudo -n docker exec -u ubuntu ClawPi bash -lc %r" % inner))
print("procs:", run("ps -eo pid,args | grep -E 'sync_slam|map_saver' | grep -v grep || echo NO_PROC"))
print("map pub:", run("sudo -n docker exec -u ubuntu ClawPi bash -lc %r" % (BEGIN + " ros2 topic info /map --verbose | head -6")))
c.close()
