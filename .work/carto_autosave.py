# -*- coding: utf-8 -*-
"""Keep writing Cartographer pbstream snapshots so a power cut loses little.

    python carto_autosave.py 20 360

Runs until the deadline, writing /tmp/carto_auto_<NNN>.pbstream every GAP seconds
and printing the file list each time.
"""
import sys
import time

import paramiko

CAR = "192.168.167.136"
BEGIN = "source /opt/ros/humble/setup.bash;"
GAP = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
LIMIT = float(sys.argv[2]) if len(sys.argv) > 2 else 300.0


def dexec(c, inner, timeout=120):
    o = c.exec_command("sudo -n docker exec -u ubuntu ClawPi bash -lc %s" % repr(BEGIN + " " + inner), timeout=timeout)
    return (o[1].read().decode("utf-8", "replace") + o[2].read().decode("utf-8", "replace")).strip()


def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(CAR, username="pi", password="raspberrypi", timeout=10)
    end = time.time() + LIMIT
    try:
        while time.time() < end:
            name = time.strftime("%H%M%S")
            target = "/tmp/carto_auto_%s.pbstream" % name
            req = '"{filename: \'%s\', include_unfinished_submaps: true}"' % target
            out = dexec(c, "ros2 service call /write_state cartographer_ros_msgs/srv/WriteState %s" % req, 90)
            ok = "State written" in out
            print("%s autosave %s %s" % (name, target, "ok" if ok else out[-160:]), flush=True)
            time.sleep(GAP)
    finally:
        c.close()


if __name__ == "__main__":
    main()
