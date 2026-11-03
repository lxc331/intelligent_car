# -*- coding: utf-8 -*-
"""Turn a driven trajectory into a road network inside the custom project.

Run from the repo root with app/ on sys.path:

    python .work/build_from_track.py
"""
import csv
import json
import math
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "app"))

import numpy as np
from PIL import Image, ImageDraw

from carapp.core.navigation_path import pose_clear, swept_clear
from carapp.core.network_editor import _Clearance
from carapp.core.road_network import RoadNetwork, RoadNode
from carapp.core.sandbox_project import load_project, save_network
from carapp.core.sandbox_route import plan_route

TRACK = ROOT / ".work" / "drive_track.csv"
PROJECT = ROOT / "app" / "savedMap" / "sandbox_custom_20261102"
BACKUP = ROOT / ".work" / "backup_track_build"
FOOTPRINT = (.25, .15, .115)
SPACING = .22            # metres between nodes on straight parts
TURN_DEG = 8.            # or split when heading changed this much
MIN_NODE_GAP = .12
WIDTH_MIN = .23
import os
BLOCKED = int(os.environ.get("TRACK_THRESHOLD", "65"))
DRY = os.environ.get("TRACK_DRY") == "1"


def load_track():
    rows = []
    with TRACK.open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rows.append((float(row["time"]), float(row["x"]), float(row["y"]), float(row["yaw"])))
    cleaned = [rows[0]]
    for row in rows[1:]:
        step = math.dist(row[1:3], cleaned[-1][1:3])
        if step > .35:            # AMCL jump; drop the segment start
            continue
        cleaned.append(row)
    return cleaned


def wrap(angle):
    while angle > math.pi:
        angle -= 2. * math.pi
    while angle < -math.pi:
        angle += 2. * math.pi
    return angle


def pick_nodes(track, clearance):
    nodes = []
    anchor = track[0]
    for index, row in enumerate(track):
        if not nodes:
            nodes.append(row)
            anchor = row
            continue
        distance = math.dist(row[1:3], anchor[1:3])
        heading = abs(wrap(row[3] - anchor[3]))
        if distance >= SPACING or (heading >= math.radians(TURN_DEG) and distance >= MIN_NODE_GAP):
            nodes.append(row)
            anchor = row
    if nodes[-1] is not track[-1] and math.dist(nodes[-1][1:3], track[-1][1:3]) > MIN_NODE_GAP:
        nodes.append(track[-1])
    return nodes


def recentre(node, clearance, radius=.12, step=.02):
    """Move a node to the nearby spot with the best clearance from obstacles."""
    best = (clearance.at(node[1], node[2]), node[1], node[2])
    if best[0] < 0:
        best = (-1., node[1], node[2])
    count = int(radius / step)
    for dx in range(-count, count + 1):
        for dy in range(-count, count + 1):
            x, y = node[1] + dx * step, node[2] + dy * step
            value = clearance.at(x, y)
            if value > best[0]:
                best = (value, x, y)
    return best[1], best[2], best[0]


def segment_width(grid, clearance, ax, ay, bx, by):
    length = math.hypot(bx - ax, by - ay)
    yaw = math.atan2(by - ay, bx - ax)
    samples = max(2, int(length / .025) + 1)
    width = math.inf
    for index in range(samples):
        t = index / (samples - 1)
        x, y = ax + (bx - ax) * t, ay + (by - ay) * t
        value = clearance.at(x, y)
        if value < 0:
            return -1.
        width = min(width, 2. * value)
    return width


def main():
    track = load_track()
    project = load_project(PROJECT / "sandbox.json")
    grid = project.keepout.grid
    clearance = _Clearance(grid)
    print("track samples", len(track), "span %.1f s" % (track[-1][0] - track[0][0]),
          "start", [round(v, 2) for v in track[0][1:3]], "end", [round(v, 2) for v in track[-1][1:3]])
    length = sum(math.dist(a[1:3], b[1:3]) for a, b in zip(track, track[1:]))
    print("driven length %.2f m" % length)

    nodes, rejected = [], 0
    for row in pick_nodes(track, clearance):
        x, y, clear = recentre(row, clearance, .15)
        if clear < WIDTH_MIN / 2:
            rejected += 1
            continue
        if nodes and math.dist((x, y), (nodes[-1][1], nodes[-1][2])) < MIN_NODE_GAP:
            continue
        nodes.append((row[0], x, y, row[3]))
    print("nodes", len(nodes), "rejected", rejected)

    links, failures = [], []
    for index, (left, right) in enumerate(zip(nodes, nodes[1:])):
        width = segment_width(grid, clearance, left[1], left[2], right[1], right[2])
        yaw = math.atan2(right[2] - left[2], right[1] - left[1])
        keep_ok = swept_clear(grid, ((left[1], left[2], yaw), (right[1], right[2], yaw)), FOOTPRINT, BLOCKED)
        if width >= WIDTH_MIN and keep_ok:
            links.append((index, index + 1, width))
        else:
            failures.append((index, round(width, 3), keep_ok))
    # close the loop when the drive ended near the start
    if len(nodes) > 3 and math.dist((nodes[0][1], nodes[0][2]), (nodes[-1][1], nodes[-1][2])) < .45:
        width = segment_width(grid, clearance, nodes[-1][1], nodes[-1][2], nodes[0][1], nodes[0][2])
        yaw = math.atan2(nodes[0][2] - nodes[-1][2], nodes[0][1] - nodes[-1][1])
        if width >= WIDTH_MIN and swept_clear(grid, ((nodes[-1][1], nodes[-1][2], yaw),
                                                     (nodes[0][1], nodes[0][2], yaw)), FOOTPRINT, BLOCKED):
            links.append((len(nodes) - 1, 0, width))
        else:
            print("loop closure rejected: width %.3f" % width)
    print("links", len(links), "failures", len(failures))
    for failure in failures[:10]:
        print("  failed edge", failure)

    adjacency = {index: [] for index in range(len(nodes))}
    widths = {}
    for left, right, width in links:
        adjacency[left].append(right)
        adjacency[right].append(left)
        widths["%d:%d" % (left, right)] = max(width, WIDTH_MIN)
        widths["%d:%d" % (right, left)] = max(width, WIDTH_MIN)
    network = RoadNetwork([], source_path=PROJECT / "road_network.yaml")
    for index, node in enumerate(nodes):
        network.nodes[index] = RoadNode(index, node[1], node[2], node[3],
                                        tuple(adjacency[index]), "路口%d" % index)

    if DRY:
        print("dry run: threshold", BLOCKED, "- project left untouched")
        return
    BACKUP.mkdir(parents=True, exist_ok=True)
    for name in ("road_network.yaml", "sandbox.json"):
        shutil.copy2(PROJECT / name, BACKUP / name)
    project = save_network(PROJECT / "sandbox.json", network, widths)
    print("saved project nodes", len(project.network), "edge keys", len(project.metadata["edge_widths"]))

    ids = sorted(project.network.nodes)
    ok = fail = 0
    for index, start in enumerate(ids[:40]):
        for goal in ids[index + 1:index + 8]:
            try:
                plan_route(project, project.network.node(start).pose, goal)
                ok += 1
            except Exception:
                fail += 1
    print("plan_route sample ok", ok, "fail", fail)

    overview(project, nodes, links)


def overview(project, nodes, links):
    meta = json.loads((PROJECT / "sandbox.json").read_text(encoding="utf-8"))
    yaml_text = (PROJECT / "map.yaml").read_text(encoding="utf-8")
    origin = [float(v) for v in yaml_text.split("origin:")[1].split("[")[1].split("]")[0].split(",")]
    resolution = float(yaml_text.split("resolution:")[1].split()[0])
    image = Image.open(PROJECT / "map.pgm").convert("RGB")
    scale = 3
    image = image.resize((image.width * scale, image.height * scale), Image.NEAREST)
    draw = ImageDraw.Draw(image)

    def point(x, y):
        return ((x - origin[0]) / resolution * scale, image.height - (y - origin[1]) / resolution * scale)

    for left, right, width in links:
        draw.line([point(nodes[left][1], nodes[left][2]), point(nodes[right][1], nodes[right][2])],
                  fill=(0, 160, 255), width=2)
    for index, node in enumerate(nodes):
        x, y = point(node[1], node[2])
        draw.ellipse([x - 3, y - 3, x + 3, y + 3], fill=(255, 0, 0))
        draw.text((x + 4, y - 10), str(index), fill=(255, 0, 0))
    out = ROOT / ".work" / "maps" / "track_network.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    image.save(out)
    print("overlay", out)


if __name__ == "__main__":
    main()
