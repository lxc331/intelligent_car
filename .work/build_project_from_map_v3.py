# -*- coding: utf-8 -*-
"""v3: waypoint lattice on the drivable surface of the lidar map.

Nodes sit every LATTICE metres where a 0.23 m body fits with margin; neighbouring
nodes are linked when the straight segment stays inside the drivable surface.
This is the graph the sandbox navigation page will plan over.
"""
import hashlib
import json
import math
import shutil
import sys
import uuid
from pathlib import Path

import numpy as np
import scipy.ndimage as ndi
import yaml
from PIL import Image

MAP_DIR = Path("app/savedMap/sandbox_20261031_2025")
OUT = Path("app/savedMap/sandbox_nav_20261031")
LATTICE = 0.25
BODY_HALF = 0.115
MARGIN = 0.02
CLEARANCE = 0.05
DISTRICT_CLOSE = 0.13
DISTRICT_GROW = 0.02
PLATE_CENTRE = (1.297, -0.491)
PLATE_THETA = math.radians(46.97)
PLATE_LONG = 5.400
PLATE_SHORT = 2.960
EDGE_MIN = 0.23

MAP_YAML = yaml.safe_load((MAP_DIR / "map.yaml").read_text(encoding="utf-8"))
IMAGE = np.array(Image.open(MAP_DIR / MAP_YAML["image"]).convert("L"))
RES = float(MAP_YAML["resolution"])
OX, OY = float(MAP_YAML["origin"][0]), float(MAP_YAML["origin"][1])
H, W = IMAGE.shape
free = IMAGE >= 250
occ = IMAGE <= 60
d_occ = ndi.distance_transform_edt(~occ) * RES


def plate_mask():
    yy, xx = np.mgrid[0:H, 0:W]
    x = xx * RES + OX
    y = (H - 1 - yy) * RES + OY
    c, s = math.cos(-PLATE_THETA), math.sin(-PLATE_THETA)
    u = c * (x - PLATE_CENTRE[0]) - s * (y - PLATE_CENTRE[1])
    v = s * (x - PLATE_CENTRE[0]) + c * (y - PLATE_CENTRE[1])
    return (np.abs(u) <= PLATE_LONG / 2) & (np.abs(v) <= PLATE_SHORT / 2)


def drivable():
    closed = ndi.binary_closing(occ, structure=np.ones((3, 3), bool),
                               iterations=max(1, int(round(DISTRICT_CLOSE / RES / 2))))
    grown = ndi.binary_dilation(closed, iterations=max(1, int(round(DISTRICT_GROW / RES))))
    return free & plate_mask() & ~grown


drv = drivable()
lat = ndi.binary_opening(drv & (d_occ >= CLEARANCE), structure=np.ones((2, 2), bool))
dt = ndi.distance_transform_edt(lat) * RES
needed = BODY_HALF + MARGIN


def at(x, y):
    gx, gy = int(round((x - OX) / RES)), int(round(H - 1 - (y - OY) / RES))
    if not (0 <= gx < W and 0 <= gy < H):
        return 0.0
    return dt[gy, gx]


def segment_ok(a, b):
    n = max(2, int(math.ceil(math.dist(a, b) / 0.02)))
    widths = []
    for i in range(n + 1):
        t = i / n
        d = at(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        if d < needed:
            return 0.0
        widths.append(2 * d)
    return min(widths)


def lattice_nodes():
    yy, xx = np.mgrid[0:H, 0:W]
    x = xx * RES + OX
    y = (H - 1 - yy) * RES + OY
    c, s = math.cos(-PLATE_THETA), math.sin(-PLATE_THETA)
    u = c * (x - PLATE_CENTRE[0]) - s * (y - PLATE_CENTRE[1])
    v = s * (x - PLATE_CENTRE[0]) + c * (y - PLATE_CENTRE[1])
    us = np.arange(-PLATE_LONG / 2 + LATTICE / 2, PLATE_LONG / 2, LATTICE)
    vs = np.arange(-PLATE_SHORT / 2 + LATTICE / 2, PLATE_SHORT / 2, LATTICE)
    nodes, index = [], {}
    for i, uu in enumerate(us):
        for j, vv in enumerate(vs):
            wx = PLATE_CENTRE[0] + math.cos(PLATE_THETA) * uu - math.sin(PLATE_THETA) * vv
            wy = PLATE_CENTRE[1] + math.sin(PLATE_THETA) * uu + math.cos(PLATE_THETA) * vv
            if at(wx, wy) >= needed:
                index[(i, j)] = len(nodes)
                nodes.append((wx, wy, (i, j)))
    return nodes, index


def build():
    if OUT.exists():
        shutil.rmtree(OUT)
    nodes, index = lattice_nodes()
    links = []
    for i, j in list(index):
        for di, dj in ((1, 0), (0, 1), (1, 1), (1, -1)):
            other = index.get((i + di, j + dj))
            if other is None:
                continue
            u = index[(i, j)]
            w = segment_ok(nodes[u][:2], nodes[other][:2])
            if w >= EDGE_MIN:
                links.append((u, other, w))
    adj = {}
    for u, v, _ in links:
        adj.setdefault(u, set()).add(v)
        adj.setdefault(v, set()).add(u)
    seen, comps = set(), []
    for start in adj:
        if start in seen:
            continue
        stack, comp = [start], set()
        while stack:
            cur = stack.pop()
            if cur in comp:
                continue
            comp.add(cur)
            seen.add(cur)
            stack.extend(adj[cur] - comp)
        comps.append(comp)
    comps.sort(key=len, reverse=True)
    print("nodes %d links %d components %s" % (len(nodes), len(links), [len(c) for c in comps[:5]]))
    main = comps[0]
    used = sorted(main)
    remap = {old: k for k, old in enumerate(used)}
    wps = [(nodes[i][0], nodes[i][1]) for i in used]
    kept = [(remap[u], remap[v], w) for u, v, w in links if u in main and v in main]

    OUT.mkdir(parents=True)
    shutil.copy(MAP_DIR / MAP_YAML["image"], OUT / "map.pgm")
    map_text = (MAP_DIR / "map.yaml").read_text(encoding="utf-8")
    (OUT / "map.yaml").write_text(map_text, encoding="utf-8")
    pgm = b"P5\n%d %d\n255\n" % (W, H) + np.where(drv, 255, 0).astype(np.uint8).tobytes()
    for name in ("roads", "keepout"):
        (OUT / (name + ".pgm")).write_bytes(pgm)
        (OUT / (name + ".yaml")).write_text(
            map_text.replace("image: " + MAP_YAML["image"], "image: " + name + ".pgm"),
            encoding="utf-8")

    to = {i: set() for i in range(len(wps))}
    for u, v, _ in kept:
        to[u].add(v)
        to[v].add(u)
    lines = ["waypoints:"]
    for i, (x, y) in enumerate(wps):
        lines += ["  - id: %d" % i, "    pose: [%.4f, %.4f]" % (x, y),
                  "    to: [%s]" % ", ".join(str(t) for t in sorted(to[i])),
                  "    name: 璺彛%d" % i]
    (OUT / "road_network.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")

    edge_widths = {}
    for u, v, w in kept:
        edge_widths["%d:%d" % (u, v)] = round(w, 3)
        edge_widths["%d:%d" % (v, u)] = round(w, 3)
    pairs = [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]
    meta = dict(version=1, id=uuid.uuid4().hex, frame="map", calibrated=True, max_rms=0.075,
                calibration=dict(source=pairs, target=pairs,
                                 transform=dict(x=0.0, y=0.0, yaw=0.0, rms=0.0,
                                                maximum=0.0, scale_ratio=1.0)),
                edge_widths=edge_widths, narrow_enabled=True, narrow_nodes=[],
                sources=dict(localization=str(MAP_DIR), roads="lattice on the lidar map"),
                note="graph from the 2026-10-31 lidar map")
    sha = {}
    for path in OUT.iterdir():
        sha[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    meta["sha256"] = sha
    (OUT / "sandbox.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2),
                                      encoding="utf-8")
    print("wrote %s nodes %d edges %d" % (OUT, len(wps), len(kept)))

    sys.path.insert(0, str(Path("app").resolve()))
    from carapp.core.sandbox_project import load_project
    from carapp.core.sandbox_route import plan_route
    project = load_project(OUT / "sandbox.json")
    ids = sorted(project.network.nodes)
    ok = fail = 0
    lengths = []
    for b in ids[1:]:
        try:
            route = plan_route(project, (wps[0][0], wps[0][1], 0.0), b)
            ok += 1
            lengths.append(route.length)
        except ValueError:
            fail += 1
    print("routes from 0: ok %d fail %d" % (ok, fail))
    if lengths:
        print("route length %.2f .. %.2f m" % (min(lengths), max(lengths)))


if __name__ == "__main__":
    build()

