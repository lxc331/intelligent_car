# -*- coding: utf-8 -*-
"""Newest leftover session that has both the project map and nav2 params."""
import glob
import os

best = None
for folder in glob.glob("/tmp/carapp_nav_*"):
    pgm = os.path.join(folder, "map.pgm")
    nav2 = os.path.join(folder, "nav2.yaml")
    if not (os.path.exists(pgm) and os.path.exists(nav2)):
        continue
    if os.path.getsize(pgm) != 71835:
        continue
    stamp = os.path.getmtime(folder)
    if best is None or stamp > best[0]:
        best = (stamp, folder)
print(best[1] if best else "NONE")
