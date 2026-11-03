# -*- coding: utf-8 -*-
"""Is the localization stack still alive?"""
import subprocess

for command in ("pgrep -af 'localization_launch|amcl|map_server' || true",
                "source /opt/ros/humble/setup.bash >/dev/null 2>&1; ros2 node list 2>/dev/null | head -20"):
    out = subprocess.run(["bash", "-lc", command], capture_output=True, text=True)
    print("$", command)
    print(out.stdout.strip() or "(empty)")
    if out.stderr.strip():
        print("stderr:", out.stderr.strip()[:400])
print("--- log tail ---")
print(open("/tmp/localization.log", errors="replace").read()[-1500:] if __import__("os").path.exists("/tmp/localization.log") else "(no log)")
