# -*- coding: utf-8 -*-
"""Remote check: /map publishers, one message, map->odom TF, cartographer log errors."""
import subprocess


def sh(cmd, timeout=25):
    out = subprocess.run(["bash", "-lc", "source /opt/ros/humble/setup.bash; " + cmd],
                         capture_output=True, text=True, timeout=timeout)
    return (out.stdout + out.stderr).strip()


print("map info:\n", sh("ros2 topic info /map -v 2>&1 | head -20"))
print("map echo:\n", sh("timeout 6 ros2 topic echo /map --once --field info 2>&1 | head -10"))
print("tf map->odom:\n", sh("timeout 4 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -4"))
print("log errors:\n", sh("grep -iE 'error|fatal|fail' /tmp/carto.log | tail -10"))
print("log tail:\n", sh("tail -5 /tmp/carto.log"))
print("grid log:\n", sh("tail -10 /tmp/carto_grid.log"))
print("submaps:\n", sh("timeout 4 ros2 topic echo /submap_list --once --field submap 2>&1 | head -4"))
