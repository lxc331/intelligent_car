# -*- coding: utf-8 -*-
"""v8: road network = waypoints sampled along the skeleton of the drivable
surface, linked along the skeleton itself (so turns follow the corridor instead
of cutting corners).  Every edge is validated with the same swept-rectangle
test the navigation page uses, and the project is written in the map frame.
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
SPACING = 0.15
LINK = 0.30
CLOSING_ITERS = 2
CLEAR_MIN = 0.12
PLATE_CENTRE = (1.297, -0.491)
PLATE_THETA = math.radians(46.97)
PLATE_LONG = 5.400
PLATE_SHORT = 2.960
FOOTPRINT = (0.25, 0.15, 0.115)
PRUNE = 12

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
lat = ndi.binary_opening(keep & (d_occ >= CLEAR_MIN), np.ones((2, 2), bool))
d_edge = ndi.distance_transform_edt(keep) * RES

keep_grid = dict(width=W, height=H, resolution=RES, origin=(OX, OY, 0.0),
                 data=np.flipud(np.where(keep, 0, 100)).astype(np.uint8).ravel().tolist())
obs_grid = dict(width=W, height=H, resolution=RES, origin=(OX, OY, 0.0),
                data=np.flipud(np.where(free, 0, 100)).astype(np.uint8).ravel().tolist())


def to_xy(c, r):
    return c * RES + OX, (H - 1 - r) * RES + OY


def to_grid(x, y):
    return int(round((x - OX) / RES)), int(round(H - 1 - (y - OY) / RES))


def thin(mask):
    img = mask.copy().astype(np.uint8)
    changed = True
    while changed:
        changed = False
        for step in (0, 1):
            p = np.pad(img, 1)
            p2, p3 = p[:-2, 1:-1], p[:-2, 2:]
            p4, p5 = p[1:-1, 2:], p[2:, 2:]
            p6, p7 = p[2:, 1:-1], p[2:, :-2]
            p8, p9 = p[1:-1, :-2], p[:-2, :-2]
            b = (p2.astype(np.int16) + p3 + p4 + p5 + p6 + p7 + p8 + p9)
            a = (((p2 == 0) & (p3 == 1)).astype(np.int16)
                 + ((p3 == 0) & (p4 == 1)).astype(np.int16)
                 + ((p4 == 0) & (p5 == 1)).astype(np.int16)
                 + ((p5 == 0) & (p6 == 1)).astype(np.int16)
                 + ((p6 == 0) & (p7 == 1)).astype(np.int16)
                 + ((p7 == 0) & (p8 == 1)).astype(np.int16)
                 + ((p8 == 0) & (p9 == 1)).astype(np.int16)
                 + ((p9 == 0) & (p2 == 1)).astype(np.int16))
            cond = ((p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)) if step == 0 else \
                   ((p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0))
            rm = (img == 1) & (b >= 2) & (b <= 6) & (a == 1) & cond
            if rm.any():
                img[rm] = 0
                changed = True
    return img.astype(bool)


def deg(skel):
    k = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]])
    return ndi.convolve(skel.astype(np.uint8), k, mode="constant")


def prune(skel, iters):
    for _ in range(iters):
        ends = skel & (deg(skel) == 1)
        if not ends.any():
            return skel
        skel = skel & ~ends
    return skel


def largest(skel):
    lab, n = ndi.label(skel, structure=np.ones((3, 3), bool))
    if not n:
        return skel
    return lab == int(np.argmax(ndi.sum(skel, lab, range(1, n + 1)))) + 1


def neighbours(r, c, skel):
    out = []
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == dc == 0:
                continue
            rr, cc = r + dr, c + dc
            if 0 <= rr < H and 0 <= cc < W and skel[rr, cc]:
                out.append((rr, cc))
    return out


def waypoints(skel):
    pts = sorted((int(r), int(c)) for r, c in zip(*np.nonzero(skel)))
    chosen = []
    for r, c in pts:
        x, y = to_xy(c, r)
        if all(math.dist((x, y), p) >= SPACING for p in chosen):
            chosen.append((x, y))
    return chosen, {(round(x, 3), round(y, 3)): i for i, (x, y) in enumerate(chosen)}


def nearest_wp(skel, r, c, chosen_idx, origin):
    """BFS along the skeleton; return (index, distance_m) of the first waypoint."""
    seen = {(r, c): 0.0}
    queue = [(r, c)]
    head = 0
    while head < len(queue):
        cur = queue[head]
        head += 1
        d = seen[cur]
        if d > 0.05:
            x, y = to_xy(cur[1], cur[0])
            key = (round(x, 3), round(y, 3))
            if key in chosen_idx and (x, y) != origin:
                return chosen_idx[key], d
        for q in neighbours(*cur, skel):
            if q in seen:
                continue
            step = math.dist(to_xy(cur[1], cur[0]), to_xy(q[1], q[0]))
            nd = d + step
            if nd > LINK:
                continue
            seen[q] = nd
            queue.append(q)
    return None, None


def sweep_ok(a, b):
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    pts = ((a[0], a[1], yaw), (b[0], b[1], yaw))
    return swept_clear(keep_grid, pts, FOOTPRINT) and swept_clear(obs_grid, pts, FOOTPRINT)


def width_along(a, b):
    n = max(2, int(math.dist(a, b) / RES) + 1)
    vals = []
    for i in range(n + 1):
        t = i / n
        c, r = to_grid(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        if 0 <= c < W and 0 <= r < H:
            vals.append(d_edge[r, c])
    return 2.0 * min(vals) if vals else 0.0


def build():
    skel = largest(prune(thin(lat), PRUNE))
    print("skeleton px %d (%.1f m)" % (int(skel.sum()), skel.sum() * RES))
    wps, index = waypoints(skel)
    print("waypoints %d" % len(wps))
    edges = []
    for i, (x, y) in enumerate(wps):
        c, r = to_grid(x, y)
        # walk each skeleton neighbour direction to find the next waypoint
        for start in neighbours(r, c, skel):
            j, dist = nearest_wp(skel, start[0], start[1], index, (x, y))
            if j is None or j == i:
                continue
            a, b = wps[i], wps[j]
            if not sweep_ok(a, b):
                continue
            edges.append((i, j, width_along(a, b)))
    # de-duplicate
    uniq = {}
    for i, j, wdt in edges:
        uniq[(min(i, j), max(i, j))] = max(wdt, uniq.get((min(i, j), max(i, j)), 0.0))
    edges = [(i, j, w) for (i, j), w in uniq.items()]
    print("edges %d" % len(edges))
    return wps, edges


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
    best = max(groups.values(), key=len)
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
    pts, edges = build()
    nodes, kept = components(pts, edges)
    print("largest component nodes %d edges %d" % (len(nodes), len(kept)))
    write_project(nodes, kept)
    print("wrote", OUT)
