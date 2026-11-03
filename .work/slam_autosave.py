# -*- coding: utf-8 -*-
"""Periodically save the live map to /tmp so a power cut loses little.

    python slam_autosave.py 30 1200

Each round runs map_saver_cli into /tmp/slam_auto_<HHMMSS>/ and prints the result.
"""
import sys
import time

import paramiko

CAR = "192.168.167.136"
BEGIN = "source /opt/ros/humble/setup.bash;"
GAP = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
LIMIT = float(sys.argv[2]) if len(sys.argv) > 2 else 900.0


def dexec(c, inner, timeout=120):
    outer = "sudo -n docker exec -u ubuntu ClawPi bash -lc %r" % (BEGIN + " " + inner)
    _in, out, err = c.exec_command(outer, timeout=timeout)
    return (out.read() + err.read()).decode("utf-8", "replace").strip()


def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(CAR, username="pi", password="raspberrypi", timeout=10)
    end = time.time() + LIMIT
    try:
        while time.time() < end:
            name = time.strftime("%H%M%S")
            folder = "/tmp/slam_auto_%s" % name
            out = dexec(c, "mkdir -p %s && ros2 run nav2_map_server map_saver_cli -f %s/map --fmt pgm "
                           "--ros-args -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0"
                        % (folder, folder), 90)
            ok = "Map saved" in out or "saved" in out.lower()
            print("%s autosave %s %s" % (name, folder, "ok" if ok else out[-200:]), flush=True)
            time.sleep(GAP)
    finally:
        c.close()


if __name__ == "__main__":
    main()
