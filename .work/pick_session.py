# -*- coding: utf-8 -*-
"""Pick a leftover session whose map matches the ready project (71835 B map.pgm)."""
import glob
import os
import subprocess

print("containers:", subprocess.run(["bash", "-lc", "sudo -n docker ps --format '{{.Names}}' | tr '\\n' ' '"],
                                     capture_output=True, text=True).stdout.strip())
rows = []
for folder in glob.glob("/tmp/carapp_nav_*"):
    pgm = os.path.join(folder, "map.pgm")
    if not os.path.exists(pgm):
        continue
    rows.append((os.path.getmtime(folder), folder, os.path.getsize(pgm),
                 os.path.exists(os.path.join(folder, "nav2.yaml"))))
for mtime, folder, size, has_nav2 in sorted(rows, reverse=True):
    print("%s pgm=%d nav2=%s" % (folder, size, has_nav2))
