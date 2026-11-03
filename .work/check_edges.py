# -*- coding: utf-8 -*-
"""Per-edge swept-clear diagnostics for the new sandbox project."""
import sys
import math
from pathlib import Path

sys.path.insert(0, str(Path("app").resolve()))
from carapp.core.sandbox_project import load_project
from carapp.core.navigation_path import swept_clear

project = load_project(Path("app/savedMap/sandbox_lidar_20261031/sandbox.json"))
net = project.network
keepout = project.keepout.grid
observed = project.localization.grid
for node in net:
    for target in node.to:
        other = net.node(target)
        yaw = math.atan2(other.y - node.y, other.x - node.x)
        path = ((node.x, node.y, yaw), (other.x, other.y, yaw))
        print("%d->%d width=%.2f keepout=%s observed=%s"
              % (node.id, target,
                 project.metadata["edge_widths"].get("%d:%d" % (node.id, target), 0),
                 swept_clear(keepout, path), swept_clear(observed, path)))
