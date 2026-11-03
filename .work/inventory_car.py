# -*- coding: utf-8 -*-
"""Inventory: container name, ROS topics, leftover nav sessions with map/nav2 config."""
import glob
import os
import subprocess


def sh(command):
    return subprocess.run(["bash", "-lc", command], capture_output=True, text=True).stdout.strip()


print("containers:", sh("sudo -n docker ps --format '{{.Names}}' | tr '\\n' ' '"))
print("sessions:")
for folder in sorted(glob.glob("/tmp/carapp_nav_*")):
    names = [name for name in ("map.yaml", "map.pgm", "nav2.yaml", "state.json") if os.path.exists(os.path.join(folder, name))]
    print("  ", folder, names)
