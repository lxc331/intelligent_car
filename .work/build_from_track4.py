# -*- coding: utf-8 -*-
"""Fill gaps in the taught-track network with the recorded points themselves.

Seed nodes are recentred; when a seed-to-seed chord fails, the recorded track
points between them are inserted *verbatim* (no recentring) and the whole chain is
validated with the same footprint check. If the chain passes it replaces the
chord; otherwise the points are dropped and the failure is reported.
"""
import csv
import math
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "app"))

from PIL import Image, ImageDraw

from carapp.core.navigation_path import swept_clear
from carapp.core.network_editor import _Clearance
from carapp.core.road_network import RoadNetwork, RoadNode
from carapp.core.sandbox_project import load_project, save_network
from carapp.core.sandbox_route import plan_route

TRACK = ROOT / ".work" / "drive_track.csv"
PROJECT = ROOT / "app" / "savedMap" / "sandbox_track_20261102"
BACKUP = ROOT / ".work" / "backup_track_build3"
FOOTPRINT = (.22, .12, .095)
SPACING = .22
TURN_DEG = 8.
MIN_NODE_GAP = .12
WIDTH_MIN = .23
BLOCKED = 65


def wrap(angle):
    while angle > math.pi:
        angle -= 2. * math.pi
    while angle < -math.pi:
        angle += 2. * math.pi
    return angle


def load_track():
    rows = []
    with TRACK.open(encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rows.append((float(row["time"]), float(row["x"]), float(row["y"]), float(row["yaw"])))
    cleaned = [rows[0]]
    for row in rows[1:]:
        if math.dist(row[1:3], cleaned[-1][1:3]) > .35:
            continue
        cleaned.append(row)
    return cleaned


class Builder:
    def __init__(self, track, clearance):
        self.track, self.clearance = track, clearance
        self.nodes = {}
        self.links = []
        self.failed = []
        self.next_key = 0

    def add(self, index, recentre):
        if recentre:
            x, y, yaw = self._recentre(index)
        else:
            x, y, yaw = self.track[index][1], self.track[index][2], self.track[index][3]
        key = self.next_key
        self.next_key += 1
        self.nodes[key] = dict(i=index, x=x, y=y, yaw=yaw)
        return key

    def _recentre(self, index, radius=.12, step=.02):
        x, y, yaw = self.track[index][1], self.track[index][2], self.track[index][3]
        best = (self.clearance.at(x, y), x, y)
        count = int(radius / step)
        for dx in range(-count, count + 1):
            for dy in range(-count, count + 1):
                cx, cy = x + dx * step, y + dy * step
                value = self.clearance.at(cx, cy)
                if value > best[0]:
                    best = (value, cx, cy)
        return best[1], best[2], yaw

    def chord(self, a, b):
        ax, ay = self.nodes[a]["x"], self.nodes[a]["y"]
        bx, by = self.nodes[b]["x"], self.nodes[b]["y"]
        length = math.hypot(bx - ax, by - ay)
        if length < MIN_NODE_GAP:
            return None, 0.
        yaw = math.atan2(by - ay, bx - ax)
        samples = max(2, int(length / .025) + 1)
        width = math.inf
        for step in range(samples):
            t = step / (samples - 1)
            width = min(width, 2. * self.clearance.at(ax + (bx - ax) * t, ay + (by - ay) * t))
        if width < WIDTH_MIN:
            return None, width
        if not swept_clear(self.clearance.grid, ((ax, ay, yaw), (bx, by, yaw)), FOOTPRINT, BLOCKED):
            return None, width
        return length, width

    def fill(self, left, right):
        length, width = self.chord(left, right)
        if length:
            self.links.append((left, right, width))
            return
        gap = self.nodes[right]["i"] - self.nodes[left]["i"]
        if gap >= 3:
            step = max(1, gap // 12)
            mids = [index for index in range(self.nodes[left]["i"] + step, self.nodes[right]["i"], step)]
            keys = [left] + [self.add(index, False) for index in mids] + [right]
            trial = []
            for a, b in zip(keys, keys[1:]):
                piece, piece_width = self.chord(a, b)
                if not piece:
                    trial = None
                    break
                trial.append((a, b, piece_width))
            if trial:
                self.links.extend(trial)
                return
            for key in keys[1:-1]:
                self.nodes.pop(key, None)
        self.failed.append((left, right, round(width, 3), gap))


def pick_indices(track):
    picked, anchor = [0], 0
    for index in range(1, len(track)):
        distance = math.dist(track[index][1:3], track[anchor][1:3])
        heading = abs(wrap(track[index][3] - track[anchor][3]))
        if distance >= SPACING or (heading >= math.radians(TURN_DEG) and distance >= MIN_NODE_GAP):
            picked.append(index)
            anchor = index
    if picked[-1] != len(track) - 1:
        picked.append(len(track) - 1)
    return picked


def main():
    track = load_track()
    project = load_project(PROJECT / "sandbox.json")
    builder = Builder(track, _Clearance(project.keepout.grid))
    keys = [builder.add(index, True) for index in pick_indices(track)]
    print("track", len(track), "seed nodes", len(keys))

    for left, right in zip(keys, keys[1:]):
        builder.fill(left, right)
    first, last = keys[0], keys[-1]
    if math.dist((builder.nodes[first]["x"], builder.nodes[first]["y"]),
                 (builder.nodes[last]["x"], builder.nodes[last]["y"])) < .45:
        builder.fill(last, first)

    order = sorted(builder.nodes)
    number = {key: position for position, key in enumerate(order)}
    adjacency = {position: [] for position in number.values()}
    widths = {}
    for left, right, width in builder.links:
        a, b = number[left], number[right]
        adjacency[a].append(b)
        adjacency[b].append(a)
        widths["%d:%d" % (a, b)] = max(width, WIDTH_MIN)
        widths["%d:%d" % (b, a)] = max(width, WIDTH_MIN)
    print("nodes", len(order), "links", len(builder.links), "failed", len(builder.failed))
    for failure in builder.failed[:12]:
        print("  failed", failure)
    linked = {node for pair in builder.links for node in pair}
    orphans = [number[k] for k in order if k not in linked]
    print("orphans", len(orphans), orphans[:12])

    network = RoadNetwork([], source_path=PROJECT / "road_network.yaml")
    for key in order:
        node = builder.nodes[key]
        index = number[key]
        network.nodes[index] = RoadNode(index, node["x"], node["y"], node["yaw"],
                                        tuple(adjacency[index]), "璺彛%d" % index)

    BACKUP.mkdir(parents=True, exist_ok=True)
    for name in ("road_network.yaml", "sandbox.json"):
        shutil.copy2(PROJECT / name, BACKUP / name)
    project = save_network(PROJECT / "sandbox.json", network, widths)
    print("saved nodes", len(project.network), "edges", len(project.metadata["edge_widths"]))

    ids = sorted(project.network.nodes)
    ok = fail = 0
    for position, start in enumerate(ids[:40]):
        for goal in ids[position + 1:position + 8]:
            try:
                plan_route(project, project.network.node(start).pose, goal)
                ok += 1
            except Exception:
                fail += 1
    print("plan_route sample ok", ok, "fail", fail)
    overview(builder, order, number)


def overview(builder, order, number):
    yaml_text = (PROJECT / "map.yaml").read_text(encoding="utf-8")
    origin = [float(v) for v in yaml_text.split("origin:")[1].split("[")[1].split("]")[0].split(",")]
    resolution = float(yaml_text.split("resolution:")[1].split()[0])
    image = Image.open(PROJECT / "map.pgm").convert("RGB")
    scale = 3
    image = image.resize((image.width * scale, image.height * scale), Image.NEAREST)
    draw = ImageDraw.Draw(image)

    def point(x, y):
        return ((x - origin[0]) / resolution * scale,
                image.height - (y - origin[1]) / resolution * scale)

    for left, right, _width in builder.links:
        a, b = builder.nodes[left], builder.nodes[right]
        draw.line([point(a["x"], a["y"]), point(b["x"], b["y"])], fill=(0, 160, 255), width=2)
    for key in order:
        node = builder.nodes[key]
        x, y = point(node["x"], node["y"])
        draw.ellipse([x - 2, y - 2, x + 2, y + 2], fill=(255, 0, 0))
    out = ROOT / ".work" / "maps" / "track_network3.png"
    image.save(out)
    print("overlay", out)


if __name__ == "__main__":
    main()

