# -*- coding: utf-8 -*-
"""Check what the car's container offers for a standalone localization run."""
from pathlib import Path

for name in ("nav2_bringup", "nav2_amcl", "nav2_map_server", "nav2_lifecycle_manager"):
    print(name, Path("/opt/ros/humble/share", name).is_dir())
launch = Path("/opt/ros/humble/share/nav2_bringup/launch")
print("launch files", sorted(p.name for p in launch.glob("*.py")) if launch.is_dir() else "missing")
