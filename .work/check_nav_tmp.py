# -*- coding: utf-8 -*-
"""Inventory what a previous navigation session left in /tmp (map + nav2 params)."""
import glob
import os

for folder in sorted(glob.glob("/tmp/carapp_nav_*")):
    print(folder)
    for name in ("map.yaml", "map.pgm", "nav2.yaml", "state.json", "owner.json"):
        path = os.path.join(folder, name)
        if os.path.exists(path):
            print("   %-10s %8d B" % (name, os.path.getsize(path)))
