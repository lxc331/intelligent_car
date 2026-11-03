# -*- coding: utf-8 -*-
"""Build a connected road network on the widest corridors of the lidar map.

Restricted to the sandbox plate; a waypoint only exists where the real
footprint (0.40 x 0.23 m incl. gripper) fits at some heading, and every edge is
validated with the same swept clear test the navigation page uses.
"""
import hashlib
import json
import math
import sys
import uuid
import shutil
from collections import deque
from pathlib import Path

import numpy as np
import scipy.ndimage as ndi
import yaml
from PIL import Image

sys.path.insert(0, str(Path("app").resolve()))
from carapp.core.navigation_path import pose_clear, swept_clear  # noqa: E402

SRC = Path("app/savedMap/sandbox_nav_20261031")
OUT = Path("app/savedMap/sandbox_auto_20261101")
MASK_FILE = Path(".work/maps/drivable_mask.npy")
SPACING = 0.15
FOOT = (0.25, 0.15, 0.115)
PLATE_C = (1.297, -0.491)
PLATE_HALF = (2.70, 1.48)
PLATE_ANGLE = math.radians(46.97)

MY = yaml.safe_load((SRC / "map.yaml").read_text(encoding="utf-8"))
IM = np.array(Image.open(SRC / MY["image"]).convert("L"))
RES = float(MY["resolution"])
OX, OY = float(MY["origin"][0]), float(MY["origin"][1])
H, W = IM.shape
free = IM >= 250
keep = np.array(Image.open(SRC / "keepout.pgm").convert("L")) >= 250
d_edge = ndi.distance_transform_edt(keep) * RES

plate = np.zeros((H, W), bool)
for r in range(H):
    y = OY + (r + 0.5) * RES
    for c in range(W):
        x = OX + (c + 0.5) * RES
        dx, dy = x - PLATE_C[0], y - PLATE_C[1]
        u = dx * math.cos(PLATE_ANGLE) + dy * math.sin(PLATE_ANGLE)
        v = -dx * math.sin(PLATE_ANGLE) + dy * math.cos(PLATE_ANGLE)
        plate[r, c] = abs(u) <= PLATE_HALF[0] and abs(v) <= PLATE_HALF[1]


def grid_of(mask):
    return dict(width=W, height=H, resolution=RES, origin=(OX, OY, 0.0),
                data=np.flipud(np.where(mask, 0, 100)).astype(np.uint8).ravel().tolist())


keep_grid = grid_of(keep)
obs_grid = grid_of(free)


def to_grid(x, y):
    return int(round((x - OX) / RES)), int(round(H - 1 - (y - OY) / RES))


def px_to_xy(r, c):
    return OX + (c + 0.5) * RES, OY + (H - 1 - r + 0.5) * RES


def thin(mask):
    img = mask.copy().astype(np.uint8)
    changed = True
    while changed:
        changed = False
        for step in (0, 1):
            p = np.pad(img, 1)
            p2, p3, p4, p5 = p[:-2, 1:-1], p[:-2, 2:], p[1:-1, 2:], p[2:, 2:]
            p6, p7, p8, p9 = p[2:, 1:-1], p[2:, :-2], p[1:-1, :-2], p[:-2, :-2]
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


def pix_neighbours(r, c, skel):
    out = []
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == dc == 0:
                continue
            rr, cc = r + dr, c + dc
            if 0 <= rr < H and 0 <= cc < W and skel[rr, cc]:
                out.append((rr, cc))
    return out


def main():
    drv = np.load(MASK_FILE)
    base = drv & plate
    print("drivable & plate %.1f%%" % (100.0 * base.mean()))
    skel = thin(base)
    lab, count = ndi.label(skel, structure=np.ones((3, 3)))
    sizes = ndi.sum(skel, lab, range(1, count + 1))
    skel = lab == (int(np.argmax(sizes)) + 1)
    pixels = [(int(r), int(c)) for r, c in zip(*np.nonzero(skel))]
    print("skeleton px %d" % len(pixels))

    # furthest point first, then BFS so the sampling follows the corridor
    start = pixels[0]
    far, dist = start, -1
    q = deque([(start, 0)])
    seen = {start}
    while q:
        (r, c), d = q.popleft()
        if d > dist:
            far, dist = (r, c), d
        for p in pix_neighbours(r, c, skel):
            if p not in seen:
                seen.add(p)
                q.append((p, d + 1))

    order = []
    seen = {far}
    q = deque([far])
    while q:
        p = q.popleft()
        order.append(p)
        for q2 in pix_neighbours(p[0], p[1], skel):
            if q2 not in seen:
                seen.add(q2)
                q.append(q2)
    print("bfs pixels %d" % len(order))

    nodes = []

    def node_at(p):
        x, y = px_to_xy(*p)
        for i, (nx, ny) in enumerate(nodes):
            if math.dist((x, y), (nx, ny)) < 0.11:
                return i
        c, r = to_grid(x, y)
        if not (0 <= c < W and 0 <= r < H) or d_edge[r, c] < 0.14:
            return None
        yaw = 0.0
        nb = pix_neighbours(p[0], p[1], skel)
        if nb:
            t = nb[0]
            yaw = math.atan2(px_to_xy(*t)[1] - y, px_to_xy(*t)[0] - x)
        if not (pose_clear(keep_grid, (x, y, yaw)) and pose_clear(obs_grid, (x, y, yaw))):
            return None
        nodes.append((x, y))
        return len(nodes) - 1

    owner = {}
    for p in order:
        i = node_at(p)
        if i is not None:
            owner[p] = i

    edges = {}
    for p, i in owner.items():
        for qq in pix_neighbours(p[0], p[1], skel):
            j = owner.get(qq)
            if j is None or j == i:
                continue
            key = (min(i, j), max(i, j))
            if key in edges:
                continue
            a, b = nodes[i], nodes[j]
            eyaw = math.atan2(b[1] - a[1], b[0] - a[0])
            seg = ((a[0], a[1], eyaw), (b[0], b[1], eyaw))
            if not (swept_clear(keep_grid, seg, FOOT) and swept_clear(obs_grid, seg, FOOT)):
                continue
            n = max(2, int(math.dist(a, b) / RES) + 1)
            vals = []
            for t in range(n + 1):
                f = t / n
                c, r = to_grid(a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)
                if 0 <= c < W and 0 <= r < H:
                    vals.append(d_edge[r, c])
            edges[key] = 2.0 * min(vals) if vals else 0.0

    print("nodes %d edges %d" % (len(nodes), len(edges)))
    kept = {e: w for e, w in edges.items() if w >= 0.23}
    g = {i: [] for i in range(len(nodes))}
    for i, j in kept:
        g[i].append(j)
        g[j].append(i)
    seen, comps = set(), []
    for s in g:
        if s in seen:
            continue
        stack, comp = [s], []
        seen.add(s)
        while stack:
            u = stack.pop()
            comp.append(u)
            for v in g[u]:
                if v not in seen:
                    seen.add(v)
                    stack.append(v)
        comps.append(comp)
    comps.sort(key=len, reverse=True)
    print("components", [len(c) for c in comps][:6])
    main_set = set(comps[0])
    nodes2 = [nodes[i] for i in sorted(main_set)]
    remap = {old: new for new, old in enumerate(sorted(main_set))}
    edges2 = {(remap[i], remap[j]): w for (i, j), w in kept.items()
              if i in main_set and j in main_set}
    print("main network nodes %d edges %d" % (len(nodes2), len(edges2)))
    write_project(nodes2, edges2)


def write_project(nodes, edges):
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True)
    for name in ("map.pgm", "map.yaml", "roads.pgm", "roads.yaml", "keepout.pgm", "keepout.yaml"):
        shutil.copy(SRC / name, OUT / name)
    adj = {i: [] for i in range(len(nodes))}
    widths = {}
    for (i, j), wdt in sorted(edges.items()):
        adj[i].append(j)
        adj[j].append(i)
        widths["%d:%d" % (i, j)] = round(wdt, 3)
        widths["%d:%d" % (j, i)] = round(wdt, 3)
    lines = ["waypoints:"]
    for i, p in enumerate(nodes):
        lines += ["  - id: %d" % i, "    pose: [%.4f, %.4f]" % (p[0], p[1]),
                  "    to: [%s]" % ", ".join(str(x) for x in sorted(adj[i])),
                  "    name: 路口%d" % i]
    (OUT / "road_network.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")
    files = ["map.pgm", "map.yaml", "roads.pgm", "roads.yaml",
             "keepout.pgm", "keepout.yaml", "road_network.yaml"]
    sha = {f: hashlib.sha256((OUT / f).read_bytes()).hexdigest() for f in files}
    meta = {
        "version": 1, "id": uuid.uuid4().hex[:12], "frame": "map", "calibrated": True,
        "max_rms": 0.03,
        "calibration": {"source": [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
                        "target": [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
                        "transform": {"x": 0.0, "y": 0.0, "yaw": 0.0, "rms": 0.0,
                                      "maximum": 0.0, "scale_ratio": 1.0}},
        "edge_widths": widths, "narrow_enabled": True, "narrow_nodes": [],
        "sources": {"localization": "sandbox_20261031_2025", "drawing": "auto"},
        "sha256": sha,
    }
    (OUT / "sandbox.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2),
                                      encoding="utf-8")
    print("wrote", OUT)


if __name__ == "__main__":
    main()
