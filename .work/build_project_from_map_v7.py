# -*- coding: utf-8 -*-
"""v7: lattice navigation graph over the drivable surface of the new SLAM map.

Nodes: points of a 0.25 m lattice inside the sandbox plate where the swept
footprint fits.  Edges: neighbouring lattice points (4-neighbour) whose straight
segment passes the same swept-rectangle check.  The largest connected component
becomes the road network; the project is written in the localisation-map frame.
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
SPACING = 0.25
CLOSING_ITERS = 2
CLEAR_MIN = 0.10
PLATE_CENTRE = (1.297, -0.491)
PLATE_THETA = math.radians(46.97)
PLATE_LONG = 5.400
PLATE_SHORT = 2.960
FOOTPRINT = (0.25, 0.15, 0.115)

sys.path.insert(0, str(Path("app").resolve()))
from carapp.core.navigation_path import pose_clear, swept_clear  # noqa: E402

MY = yaml.safe_load((MAP_DIR / "map.yaml").read_text(encoding="utf-8"))
IM = np.array(Image.open(MAP_DIR / MY["image"]).convert("L"))
RES = float(MY["resolution"])
OX, OY = float(MY["origin"][0]), float(MY["origin"][1])
H, W = IM.shape
free = IM >= 250
occ = IM <= 60
d_occ = ndi.distance_transform_edt(~occ) * RES

yy, xx = np.mgrid[0:H, 0:W]
gx = xx * RES + OX
gy = (H - 1 - yy) * RES + OY
_c, _s = math.cos(-PLATE_THETA), math.sin(-PLATE_THETA)
u = _c * (gx - PLATE_CENTRE[0]) - _s * (gy - PLATE_CENTRE[1])
v = _s * (gx - PLATE_CENTRE[0]) + _c * (gy - PLATE_CENTRE[1])
plate = (np.abs(u) <= PLATE_LONG / 2) & (np.abs(v) <= PLATE_SHORT / 2)

closed = ndi.binary_closing(occ, np.ones((3, 3), bool), iterations=CLOSING_ITERS)
keep = free & plate & ~closed
d_edge = ndi.distance_transform_edt(keep) * RES

keep_grid = dict(width=W, height=H, resolution=RES, origin=(OX, OY, 0.0),
                 data=np.flipud(np.where(keep, 0, 100)).astype(np.uint8).ravel().tolist())
# execution uses the raw localisation map as well; the road surface must be free
# there too, which `keep` already guarantees for obstacles.
obs_grid = dict(width=W, height=H, resolution=RES, origin=(OX, OY, 0.0),
                data=np.flipud(np.where(free, 0, 100)).astype(np.uint8).ravel().tolist())


def to_xy(c, r):
    return c * RES + OX, (H - 1 - r) * RES + OY


def plate_uv(x, y):
    c, s = math.cos(-PLATE_THETA), math.sin(-PLATE_THETA)
    return c * (x - PLATE_CENTRE[0]) - s * (y - PLATE_CENTRE[1]), \
           s * (x - PLATE_CENTRE[0]) + c * (y - PLATE_CENTRE[1])


def lattice():
    pts = []
    n_u = int(PLATE_LONG / SPACING)
    n_v = int(PLATE_SHORT / SPACING)
    c, s = math.cos(PLATE_THETA), math.sin(PLATE_THETA)
    for i in range(n_u + 1):
        for j in range(n_v + 1):
            uu = -PLATE_LONG / 2 + i * SPACING
            vv = -PLATE_SHORT / 2 + j * SPACING
            x = PLATE_CENTRE[0] + c * uu - s * vv
            y = PLATE_CENTRE[1] + s * uu + c * vv
            col, row = int(round((x - OX) / RES)), int(round(H - 1 - (y - OY) / RES))
            if 0 <= col < W and 0 <= row < H and d_edge[row, col] >= 0.115:
                pts.append((x, y))
    return pts


def sweep_ok(grid, a, b):
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    return swept_clear(grid, ((a[0], a[1], yaw), (b[0], b[1], yaw)), FOOTPRINT)


def width_along(a, b):
    n = max(2, int(math.dist(a, b) / RES) + 1)
    vals = []
    for i in range(n + 1):
        t = i / n
        x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
        col, row = int(round((x - OX) / RES)), int(round(H - 1 - (y - OY) / RES))
        if 0 <= col < W and 0 <= row < H:
            vals.append(d_edge[row, col])
    return 2.0 * min(vals) if vals else 0.0


def build_graph():
    raw = lattice()
    print("lattice candidates %d" % len(raw))
    yaws = [k * math.pi / 8 for k in range(16)]
    pts = [p for p in raw if any(pose_clear(keep_grid, (p[0], p[1], yw)) for yw in yaws)]
    print("footprint fits at %d" % len(pts))
    index = {(round(x, 3), round(y, 3)): i for i, (x, y) in enumerate(pts)}
    edges = []
    eu = (math.cos(PLATE_THETA), math.sin(PLATE_THETA))
    ev = (-math.sin(PLATE_THETA), math.cos(PLATE_THETA))
    steps = []
    for su, sv in ((1, 0), (0, 1)):
        steps.append((SPACING * (su * eu[0] + sv * ev[0]),
                      SPACING * (su * eu[1] + sv * ev[1])))
    for i, a in enumerate(pts):
        for dx, dy in steps:
            key = (round(a[0] + dx, 3), round(a[1] + dy, 3))
            j = index.get(key)
            if j is None or j <= i:
                continue
            b = pts[j]
            if not (sweep_ok(keep_grid, a, b) and sweep_ok(obs_grid, a, b)):
                continue
            edges.append((i, j, width_along(a, b)))
    print("edges %d" % len(edges))
    return pts, edges


def components(pts, edges):
    parent = list(range(len(pts)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i, j, _ in edges:
        a, b = find(i), find(j)
        if a != b:
            parent[a] = b
    groups = {}
    for i in range(len(pts)):
        groups.setdefault(find(i), []).append(i)

    def weight(ids):
        s = set(ids)
        return sum(1 for e in edges if e[0] in s)

    best = max(groups.values(), key=weight)
    remap = {old: new for new, old in enumerate(sorted(best))}
    nodes = [pts[old] for old in sorted(best)]
    kept = [(remap[i], remap[j], w) for i, j, w in edges if i in remap and j in remap]
    return nodes, kept


def write_project(nodes, edges):
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True)
    shutil.copy(MAP_DIR / MY["image"], OUT / "map.pgm")
    map_text = (MAP_DIR / "map.yaml").read_text(encoding="utf-8")
    (OUT / "map.yaml").write_text(map_text, encoding="utf-8")
    pgm = b"P5\n%d %d\n255\n" % (W, H) + np.where(keep, 255, 0).astype(np.uint8).tobytes()
    for name in ("roads", "keepout"):
        (OUT / (name + ".pgm")).write_bytes(pgm)
        (OUT / (name + ".yaml")).write_text(
            map_text.replace("image: " + MY["image"], "image: " + name + ".pgm"),
            encoding="utf-8")
    adj = {i: [] for i in range(len(nodes))}
    widths = {}
    for i, j, wdt in edges:
        adj[i].append(j)
        adj[j].append(i)
        widths["%d:%d" % (i, j)] = round(wdt, 3)
        widths["%d:%d" % (j, i)] = round(wdt, 3)
    lines = ["waypoints:"]
    for i, p in enumerate(nodes):
        lines += ["  - id: %d" % i, "    pose: [%.4f, %.4f]" % p,
                  "    to: [%s]" % ", ".join(str(x) for x in sorted(adj[i])),
                  "    name: 路口%d" % i]
    (OUT / "road_network.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")
    files = ["map.pgm", "map.yaml", "roads.pgm", "roads.yaml",
             "keepout.pgm", "keepout.yaml", "road_network.yaml"]
    sha = {f: hashlib.sha256((OUT / f).read_bytes()).hexdigest() for f in files}
    meta = {
        "version": 1, "id": uuid.uuid4().hex[:12], "frame": "map",
        "calibrated": True, "max_rms": 0.03,
        "calibration": {
            "source": [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
            "target": [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
            "transform": {"x": 0.0, "y": 0.0, "yaw": 0.0, "rms": 0.0,
                          "maximum": 0.0, "scale_ratio": 1.0},
        },
        "edge_widths": widths, "narrow_enabled": True, "narrow_nodes": [],
        "sources": {"localization": "sandbox_20261031_2025"},
        "sha256": sha,
    }
    (OUT / "sandbox.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2),
                                      encoding="utf-8")
    return OUT


if __name__ == "__main__":
    pts, edges = build_graph()
    nodes, kept = components(pts, edges)
    print("largest component nodes %d edges %d" % (len(nodes), len(kept)))
    write_project(nodes, kept)
    print("wrote", OUT)
