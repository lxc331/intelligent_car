# -*- coding: utf-8 -*-
"""Check each node against the keepout grid and the roads drawing."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path("app").resolve()))
from carapp.core.sandbox_project import load_project

P = Path("app/savedMap/sandbox_lidar_20261031")
project = load_project(P / "sandbox.json")
reg = project.registration
roads = project.roads.grid
keep = project.keepout.grid
print("registration scale %.4f yaw %.2f t (%.3f, %.3f)"
      % (reg.scale, __import__("math").degrees(reg.angle), reg.x, reg.y))
print("roads grid %dx%d res %.3f" % (roads["width"], roads["height"], roads["resolution"]))


def raster(grid, x, y):
    gx = int((x - grid["origin"][0]) / grid["resolution"])
    gy = int((y - grid["origin"][1]) / grid["resolution"])
    if not (0 <= gx < grid["width"] and 0 <= gy < grid["height"]):
        return None
    return grid["data"][gy * grid["width"] + gx]


for n in project.network.nodes:
    node = project.network.nodes[n] if isinstance(project.network.nodes, dict) else n
    px, py = reg.inverse(node.x, node.y)
    print("node %s map (%6.2f,%6.2f) plan (%5.2f,%5.2f) roads=%s keepout=%s"
          % (node.id, node.x, node.y, px, py, raster(roads, px, py), raster(keep, node.x, node.y)))
