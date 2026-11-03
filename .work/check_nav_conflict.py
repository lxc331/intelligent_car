# -*- coding: utf-8 -*-
"""Check that nothing else already owns map_server / amcl on the car."""
import subprocess

out = subprocess.run(["bash", "-lc", "pgrep -af 'map_server|amcl|localization_launch|nav2' || true"],
                     capture_output=True, text=True).stdout.strip()
print("processes:", out or "(none)")
out = subprocess.run(["bash", "-lc", "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
                                    "ros2 node list 2>/dev/null | grep -E 'amcl|map_server|bt_navigator|controller_server' || true"],
                     capture_output=True, text=True).stdout.strip()
print("nodes:", out or "(none)")
