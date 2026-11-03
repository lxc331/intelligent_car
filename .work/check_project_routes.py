# -*- coding: utf-8 -*-
"""Offline check: can the new sandbox project route between its junctions?"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path("app").resolve()))
from carapp.core.sandbox_project import load_project
from carapp.core.sandbox_route import plan_route

project = load_project(Path("app/savedMap/sandbox_lidar_20261031/sandbox.json"))
nodes = project.network.nodes
print("nodes:", len(nodes))
ok = fail = 0
lengths = []
for a in sorted(nodes):
    for b in sorted(nodes):
        if a >= b:
            continue
        na, nb = nodes[a], nodes[b]
        start = (na.x, na.y, 0.0)
        goal = b
        try:
            route = plan_route(project, start, goal)
        except ValueError as exc:
            fail += 1
            print("route %d->%d FAIL %s" % (a, b, exc))
            continue
        ok += 1
        lengths.append(route.length)
        if a == 0 and b == 8:
            print("route 0->8 nodes %s length %.2f m" % (route.nodes, route.length))
print("routed %d pairs, failed %d, length %.2f..%.2f m"
      % (ok, fail, min(lengths), max(lengths)))
print("edge widths:", len(project.metadata["edge_widths"]))
print("narrow enabled:", project.metadata.get("narrow_enabled"),
      "narrow nodes:", project.metadata.get("narrow_nodes"))

