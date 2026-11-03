# -*- coding: utf-8 -*-
"""Diagnose the car after the SSH reset: uptime, container, nodes, mapping processes."""
import paramiko

CAR = "192.168.167.136"
BEGIN = "source /opt/ros/humble/setup.bash;"


def sh(c, cmd, timeout=40):
    o = c.exec_command(cmd, timeout=timeout)
    return (o[1].read().decode("utf-8", "replace") + o[2].read().decode("utf-8", "replace")).strip()


def dexec(c, inner, timeout=40):
    return sh(c, "sudo -n docker exec -u ubuntu ClawPi bash -lc %s" % repr(BEGIN + " " + inner), timeout)


c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(CAR, username="pi", password="raspberrypi", timeout=10)
try:
    print("uptime:", sh(c, "uptime"))
    print("docker:", sh(c, "sudo -n docker ps --format '{{.Names}} {{.Status}}'"))
    print("carto procs:", dexec(c, "pgrep -af cartographer | head -5 || echo none"))
    print("nodes:", dexec(c, "timeout 6 ros2 node list 2>/dev/null | tr '\\n' ' '")[:400])
    print("map topic:", dexec(c, "timeout 5 ros2 topic info /map 2>/dev/null | tr '\\n' ' '"))
    print("odom:", dexec(c, "timeout 4 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null | tr '\\n' ' '"))
    print("carto log size:", dexec(c, "ls -l /tmp/carto.log /tmp/carto_grid.log 2>&1"))
    print("pbstreams:", sh(c, "ls -l /home/pi/carapp_carto/maps/ 2>/dev/null; sudo -n docker exec ClawPi sh -c 'ls -l /tmp/carto_*.pbstream 2>/dev/null'"))
finally:
    c.close()
