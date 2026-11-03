# -*- coding: utf-8 -*-
"""Turn the hand-drawn route on .work/maps/map_draw_canvas_marked.png into a
road network.

Red pixels  = main loop (closed stroke).
Blue pixels = extra corridors (each stroke becomes a polyline).
Every waypoint is nudged to the widest spot nearby and every edge has to pass
the same swept-footprint test the navigation page uses.
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

SRC = Path("app/savedMap/sandbox_nav_20261031")
OUT = Path("app/savedMap/sandbox_drawn_20261031")
CANVAS = Path(".work/maps/map_draw_canvas_marked.png")
META = json.loads(Path(".work/maps/map_draw_canvas.json").read_text(encoding="utf-8"))
SPACING = 0.07
FOOTPRINT = (0.25, 0.15, 0.115)
NUDGE = 0.30
PLATE_C = (1.297, -0.491)

sys.path.insert(0, str(Path("app").resolve()))
from carapp.core.navigation_path import pose_clear, swept_clear  # noqa: E402

MY = yaml.safe_load((SRC / "map.yaml").read_text(encoding="utf-8"))
IM = np.array(Image.open(SRC / MY["image"]).convert("L"))
RES = float(MY["resolution"])
OX, OY = float(MY["origin"][0]), float(MY["origin"][1])
H, W = IM.shape
S = int(META["scale"])
free = IM >= 250
occ = IM <= 60
d_free = ndi.distance_transform_edt(~occ) * RES

keep = np.array(Image.open(SRC / "keepout.pgm").convert("L")) >= 250
d_edge = ndi.distance_transform_edt(keep) * RES
keep_grid = dict(width=W, height=H, resolution=RES, origin=(OX, OY, 0.0),
                 data=np.flipud(np.where(keep, 0, 100)).astype(np.uint8).ravel().tolist())
obs_grid = dict(width=W, height=H, resolution=RES, origin=(OX, OY, 0.0),
                data=np.flipud(np.where(free, 0, 100)).astype(np.uint8).ravel().tolist())


def to_xy(px, py):
    return OX + (px + 0.5) / S * RES, OY + (H - (py + 0.5) / S) * RES


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


def neighbours(r, c, mask):
    out = []
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == dc == 0:
                continue
            rr, cc = r + dr, c + dc
            if 0 <= rr < mask.shape[0] and 0 <= cc < mask.shape[1] and mask[rr, cc]:
                out.append((rr, cc))
    return out


def strokes(mask):
    """Split a thin mask into ordered pixel chains (loop -> one closed chain)."""
    skel = thin(mask)
    left = {(int(r), int(c)) for r, c in zip(*np.nonzero(skel))}
    chains = []
    while left:
        deg = {p: len([q for q in neighbours(*p, skel) if q in left]) for p in left}
        starts = [p for p in left if deg[p] <= 1]
        start = starts[0] if starts else min(left)
        chain = [start]
        left.discard(start)
        cur = start
        while True:
            nxt = [q for q in neighbours(*cur, skel) if q in left]
            if not nxt:
                break
            if len(nxt) > 1:
                # prefer the straightest continuation
                prev = chain[-2] if len(chain) > 1 else None
                def straight(q):
                    if prev is None:
                        return 0.0
                    a = (cur[0] - prev[0], cur[1] - prev[1])
                    b = (q[0] - cur[0], q[1] - cur[1])
                    na, nb = math.hypot(*a), math.hypot(*b)
                    return (a[0] * b[0] + a[1] * b[1]) / (na * nb) if na and nb else 0.0
                nxt.sort(key=straight, reverse=True)
            cur = nxt[0]
            chain.append(cur)
            left.discard(cur)
        chains.append(chain)
    return chains


def resample(chain, px_to_xy):
    pts = [px_to_xy(r, c) for r, c in chain]
    if len(pts) < 2:
        return pts
    out = [pts[0]]
    acc = 0.0
    for a, b in zip(pts, pts[1:]):
        acc += math.dist(a, b)
        if acc >= SPACING:
            out.append(b)
            acc = 0.0
    if math.dist(out[-1], pts[-1]) > SPACING * 0.4:
        out.append(pts[-1])
    return out


def nudge(x, y, yaw):
    """Slide sideways (or inwards toward the plate centre) to the widest spot."""
    ok0 = pose_clear(keep_grid, (x, y, yaw)) and pose_clear(obs_grid, (x, y, yaw))
    c0, r0 = to_grid(x, y)
    start_clear = d_edge[r0, c0] if 0 <= c0 < W and 0 <= r0 < H else -1.0
    best, best_score = (x, y), start_clear if ok0 else -1.0
    inward = math.atan2(PLATE_C[1] - y, PLATE_C[0] - x)
    dirs = ((-math.sin(yaw), math.cos(yaw)), (math.cos(inward), math.sin(inward)))
    for dx, dy in dirs:
        for off in np.arange(0.0, NUDGE + 1e-9, 0.025):
            for sign in ((1,) if off == 0 else (1, -1)):
                cand = (x + sign * off * dx, y + sign * off * dy)
                c, r = to_grid(*cand)
                if not (0 <= c < W and 0 <= r < H):
                    continue
                if pose_clear(keep_grid, (cand[0], cand[1], yaw)) and \
                   pose_clear(obs_grid, (cand[0], cand[1], yaw)):
                    score = d_edge[r, c] - 0.20 * abs(off)
                    if score > best_score:
                        best, best_score = cand, score
    return best[0], best[1], best_score >= 0.10


def width_along(a, b):
    n = max(2, int(math.dist(a, b) / RES) + 1)
    vals = []
    for i in range(n + 1):
        t = i / n
        c, r = to_grid(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        if 0 <= c < W and 0 <= r < H:
            vals.append(d_edge[r, c])
    return 2.0 * min(vals) if vals else 0.0


def main():
    rgb = np.array(Image.open(CANVAS).convert("RGB")).astype(int)
    masks = {
        "red": (rgb[:, :, 0] > 170) & (rgb[:, :, 1] < 110) & (rgb[:, :, 2] < 110),
        "blue": (rgb[:, :, 2] > 170) & (rgb[:, :, 0] < 110) & (rgb[:, :, 1] < 110),
    }
    nodes, edges = [], {}
    dropped = []

    def add_node(p):
        for i, q in enumerate(nodes):
            if math.dist(p, q) < 0.10:
                return i
        nodes.append(p)
        return len(nodes) - 1

    for name, mask in masks.items():
        if not mask.any():
            continue
        for chain in strokes(mask):
            pts = resample(chain, to_xy)
            if len(pts) < 2:
                continue
            idx = []
            for k, p in enumerate(pts):
                prev = pts[max(0, k - 1)]
                nxt = pts[min(len(pts) - 1, k + 1)]
                yaw = math.atan2(nxt[1] - prev[1], nxt[0] - prev[0])
                x, y, ok = nudge(p[0], p[1], yaw)
                if not ok:
                    dropped.append((name, round(x, 2), round(y, 2)))
                idx.append(add_node((x, y)))
            for i, j in zip(idx, idx[1:]):
                if i == j:
                    continue
                eyaw = math.atan2(nodes[j][1] - nodes[i][1], nodes[j][0] - nodes[i][0])
                seg = ((nodes[i][0], nodes[i][1], eyaw), (nodes[j][0], nodes[j][1], eyaw))
                if not (swept_clear(keep_grid, seg, FOOTPRINT) and swept_clear(obs_grid, seg, FOOTPRINT)):
                    wdt = 0.0
                else:
                    wdt = width_along(nodes[i], nodes[j])
                edges[(min(i, j), max(i, j))] = max(wdt, edges.get((min(i, j), max(i, j)), 0.0))

    print("nodes %d  links %d  dropped %d" % (len(nodes), len(edges), len(dropped)))
    for d in dropped[:12]:
        print("  dropped", d)
    write_project(nodes, edges)


def write_project(nodes, edges):
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True)
    shutil.copy(SRC / "map.pgm", OUT / "map.pgm")
    shutil.copy(SRC / "map.yaml", OUT / "map.yaml")
    shutil.copy(SRC / "roads.pgm", OUT / "roads.pgm")
    shutil.copy(SRC / "roads.yaml", OUT / "roads.yaml")
    shutil.copy(SRC / "keepout.pgm", OUT / "keepout.pgm")
    shutil.copy(SRC / "keepout.yaml", OUT / "keepout.yaml")
    adj = {i: [] for i in range(len(nodes))}
    widths = {}
    for (i, j), wdt in sorted(edges.items()):
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
        "sources": {"localization": "sandbox_20261031_2025",
                    "drawing": "map_draw_canvas_marked.png"},
        "sha256": sha,
    }
    (OUT / "sandbox.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2),
                                      encoding="utf-8")
    print("wrote", OUT)


if __name__ == "__main__":
    main()
