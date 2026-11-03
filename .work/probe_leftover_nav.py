# -*- coding: utf-8 -*-
"""Look at leftover navigation processes on the car (read-only)."""
import sys

import paramiko

CAR = "192.168.167.136"
BEGIN = "source /opt/ros/humble/setup.bash;"


def run(c, inner, timeout=60):
    command = "sudo -n docker exec -u ubuntu ClawPi bash -lc %s" % repr(BEGIN + " " + inner)
    o = c.exec_command(command, timeout=timeout)
    return (o[1].read().decode("utf-8", "replace"), o[2].read().decode("utf-8", "replace"))


def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(CAR, username="pi", password="raspberrypi", timeout=10)
    try:
        out, err = run(c, "ros2 node list 2>/dev/null | sort")
        nodes = [n for n in out.split() if n.startswith("/")]
        print("nodes:", len(nodes))
        for n in nodes:
            print("  ", n)
        out, err = run(c, "ps -eo pid,etimes,cmd --sort=pid | grep -E 'navigation.launch|carapp_nav|nav2|slam_toolbox|amcl|map_server' | grep -v grep")
        print("-- processes --")
        print(out or "(none)")
    finally:
        c.close()


if __name__ == "__main__":
    main()
