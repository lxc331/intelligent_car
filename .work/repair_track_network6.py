# -*- coding: utf-8 -*-
"""Attach skipped nodes back to the chain (or drop them) and renumber."""
import math
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "app"))

from carapp.core.navigation_path import swept_clear
from carapp.core.network_editor import _Clearance
from carapp.core.road_network import RoadNetwork, RoadNode
from carapp.core.sandbox_project import load_project, save_network
from carapp.core.sandbox_route import plan_route

PROJECT = ROOT / "app" / "savedMap" / "sandbox_track_20261102"
FOOTPRINT = (.20, .10, .09)
THRESHOLD = 65
WIDTH_MIN = .20
BACKUP = ROOT / ".work" / "backup_track_repair6"


def main():
    project = load_project(PROJECT / "sandbox.json")
    clearance = _Clearance(project.keepout.grid)
    nodes = project.network.nodes
    widths = dict(project.metadata["edge_widths"])

    def chord(a, b):
        ax, ay = nodes[a].x, nodes[a].y
        bx, by = nodes[b].x, nodes[b].y
        length = math.hypot(bx - ax, by - ay)
        if length < .05:
            return None
        yaw = math.atan2(by - ay, bx - ax)
        samples = max(2, int(length / .025) + 1)
        width = math.inf
        for step in range(samples):
            t = step / (samples - 1)
            width = min(width, 2. * clearance.at(ax + (bx - ax) * t, ay + (by - ay) * t))
        if width < WIDTH_MIN:
            return None
        segment = ((ax, ay, yaw), (bx, by, yaw))
        if not swept_clear(clearance.grid, segment, FOOTPRINT, THRESHOLD):
            return None
        if not swept_clear(project.localization.grid, segment, FOOTPRINT, THRESHOLD):
            return None
        return width

    adjacency = {node_id: set(node.to) for node_id, node in nodes.items()}
    orphans = [node_id for node_id, to in adjacency.items() if not to]
    print("orphans", len(orphans), orphans)
    repaired = []
    for orphan in orphans:
        candidates = sorted((node_id for node_id in nodes if adjacency[node_id]),
                            key=lambda other: math.dist((nodes[orphan].x, nodes[orphan].y),
                                                        (nodes[other].x, nodes[other].y)))
        for other in candidates[:6]:
            width = chord(orphan, other)
            if width:
                adjacency[orphan].add(other)
                adjacency[other].add(orphan)
                widths["%d:%d" % (orphan, other)] = max(width, WIDTH_MIN)
                widths["%d:%d" % (other, orphan)] = max(width, WIDTH_MIN)
                repaired.append((orphan, other))
                break
    print("repaired", repaired)

    dead = [node_id for node_id, to in adjacency.items() if not to]
    print("dropping", dead)
    keep = sorted(node_id for node_id in nodes if node_id not in dead)
    number = {old: new for new, old in enumerate(keep)}
    network = RoadNetwork([], source_path=PROJECT / "road_network.yaml")
    clean_widths = {}
    for old in keep:
        new = number[old]
        to = tuple(sorted(number[other] for other in adjacency[old] if other in number))
        network.nodes[new] = RoadNode(new, nodes[old].x, nodes[old].y, nodes[old].yaw, to,
                                      nodes[old].name or "路口%d" % new)
    for key, width in widths.items():
        a_text, b_text = key.split(":")
        a, b = int(a_text), int(b_text)
        if a in number and b in number:
            clean_widths["%d:%d" % (number[a], number[b])] = width

    BACKUP.mkdir(parents=True, exist_ok=True)
    for name in project.metadata.get("sha256", {}):
        shutil.copy2(PROJECT / name, BACKUP / name)
    shutil.copy2(PROJECT / "sandbox.json", BACKUP / "sandbox.json")
    project = save_network(PROJECT / "sandbox.json", network, clean_widths)
    print("saved nodes", len(project.network), "edges", len(project.metadata["edge_widths"]))

    import random
    ids = sorted(project.network.nodes)
    random.seed(7)
    pairs = [(a, b) for a in ids for b in ids if a < b]
    sample = random.sample(pairs, min(120, len(pairs)))
    ok = fail = 0
    for start, goal in sample:
        try:
            plan_route(project, project.network.node(start).pose, goal)
            ok += 1
        except Exception:
            fail += 1
    print("plan sample ok", ok, "fail", fail)


if __name__ == "__main__":
    main()
