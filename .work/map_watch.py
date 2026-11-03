# -*- coding: utf-8 -*-
"""Repeatedly report mapping progress: processes, map size, trajectory pose, carto log tail."""
import json
import sys
import time

import paramiko

CAR = "192.168.167.136"
BEGIN = "source /opt/ros/humble/setup.bash;"
ROUNDS = int(sys.argv[1]) if len(sys.argv) > 1 else 1
GAP = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0


def dexec(client, inner, timeout=40):
    cmd = "sudo -n docker exec -u ubuntu ClawPi bash -lc %s" % repr(BEGIN + " " + inner)
    out = client.exec_command(cmd, timeout=timeout)
    return out[1].read().decode("utf-8", "replace") + out[2].read().decode("utf-8", "replace")


c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(CAR, username="pi", password="raspberrypi", timeout=10)
try:
    for i in range(ROUNDS):
        info = dexec(c, "timeout 6 ros2 topic echo /map --once --field info 2>/dev/null "
                        "| grep -E 'width|height|resolution' | tr '\\n' ' '").strip()
        pose = dexec(c, "timeout 4 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null "
                        "| tr '\\n' ' '").strip()
        tail = dexec(c, "tail -2 /tmp/carto.log 2>/dev/null").strip().splitlines()
        print(json.dumps({"round": i + 1, "map": info, "odom": pose,
                          "log": tail[-1] if tail else ""}, ensure_ascii=False), flush=True)
        if i + 1 < ROUNDS:
            time.sleep(GAP)
finally:
    c.close()
