# -*- coding: utf-8 -*-
"""Remote sanity check: rclpy import, cartographer_ros presence and executables."""
import shutil
import subprocess

print("rclpy import:", end=" ")
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402

print(rclpy.__file__, "Node OK:", Node.__name__)

print("cartographer_node:", shutil.which("cartographer_node"))
for name in ("cartographer_node", "cartographer_occupancy_grid_node", "cartographer_pbstream_to_ros_map"):
    found = shutil.which(name)
    print(name, "->", found)
try:
    out = subprocess.run(["ros2", "pkg", "executables", "cartographer_ros"], capture_output=True, text=True, timeout=30)
    print("executables:", out.stdout.strip())
except Exception as exc:  # noqa: BLE001
    print("pkg query failed:", exc)
