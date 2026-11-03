# -*- coding: utf-8 -*-
"""v6: build a navigation project straight from the new SLAM map.

* keepout = free inside the sandbox plate, with the model districts closed
  (lidar sees through tree gaps, so the districts have to be sealed).
* roads   = same map, so plan/exec see one consistent drivable surface.
* network = waypoints sampled along the skeleton of the wide part of the
  corridor, with every edge validated by the same swept-rectangle check the
  navigation page runs.
* The project lives in the localisation-map frame, registration is identity.
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
LINK = 0.45
CLOSING_ITERS = 2
DISTRICT_CLEAR = 0.05
CLEAR_MIN = 0.10
PLATE_CENTRE = (1.297, -0.491)
PLATE_THETA = math.radians(46.97)
PLATE_LONG = 5.400
PLATE_SHORT = 2.960
FOOTPRINT = (0.25, 0.15, 0.115)

sys.path.insert(0, str(Path("app").resolve()))
from carapp.core.navigation_path import swept_clear  # noqa: E402

MY = yaml.safe_load((MAP_DIR / "map.yaml").read_text(encoding="utf-8"))
IM = np.array(Image.open(MAP_DIR / MY["image"]).convert("L"))
RES = float(MY["resolution"])
OX, OY = float(MY["origin"][0]), float(MY["origin"][1])
H, W = IM.shape
free = IM >= 250
occ = IM <= 60
d_free = ndi.distance_transform_edt(free) * RES
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


def to_xy(col, row):
    return col * RES + OX, (H - 1 - row) * RES + OY


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
            if step == 0:
                cond = (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)
            else:
                cond = (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
            rm = (img == 1) & (b >= 2) & (b <= 6) & (a == 1) & cond
            if rm.any():
                img[rm] = 0
                changed = True
    return img.astype(bool)


def deg(skel):
    k = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]])
    return ndi.convolve(skel.astype(np.uint8), k, mode="constant")


def largest(skel):
    lab, n = ndi.label(skel, structure=np.ones((3, 3), bool))
    if not n:
        return skel
    sizes = ndi.sum(skel, lab, range(1, n + 1))
    return lab == int(np.argmax(sizes)) + 1


def sample_waypoints(skel):
    pts = [to_xy(c, r) for r, c in zip(*np.nonzero(skel))]
    chosen, chosen_idx = [], []
    for i, p in enumerate(pts):
        if all(math.dist(p, q) >= SPACING for q in chosen):
            chosen.append(p)
            chosen_idx.append(i)
    return chosen


def sweep_ok(grid, a, b):
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    return swept_clear(grid, ((a[0], a[1], yaw), (b[0], b[1], yaw)), FOOTPRINT)


def width_along(a, b, d_edge):
    n = max(2, int(math.dist(a, b) / RES) + 1)
    vals = []
    for i in range(n + 1):
        t = i / n
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        c, r = to_grid(x, y)
        vals.append(d_edge[r, c])
    return 2.0 * min(vals)


def write_project(keep, nodes, edges):
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True)
    shutil.copy(MAP_DIR / MY["image"], OUT / "map.pgm")
    map_text = (MAP_DIR / "map.yaml").read_text(encoding="utf-8")
    (OUT / "map.yaml").write_text(map_text, encoding="utf-8")
    pgm = b"P5\n%d %d\n255\n" % (W, H)
    pgm += np.where(keep, 255, 0).astype(np.uint8).tobytes()
    for name in ("roads", "keepout"):
        (OUT / (name + ".pgm")).write_bytes(pgm)
        (OUT / (name + ".yaml")).write_text(
            map_text.replace("image: " + MY["image"], "image: " + name + ".pgm"),
            encoding="utf-8")
    adj = {i: [] for i in range(len(nodes))}
    widths = {}
    for i, j, wdt in edges:
        adj[i].append(j)
        widths["%d:%d" % (i, j)] = round(wdt, 3)
    lines = ["waypoints:"]
    for i, p in enumerate(nodes):
        lines += ["  - id: %d" % i, "    pose: [%.4f, %.4f]" % p,
                  "    to: [%s]" % ", ".join(str(x) for x in sorted(adj[i])),
                  "    name: 璺彛%d" % i]
    (OUT / "road_network.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")

    files = ["map.pgm", "map.yaml", "roads.pgm", "roads.yaml",
             "keepout.pgm", "keepout.yaml", "road_network.yaml"]
    sha = {f: hashlib.sha256((OUT / f).read_bytes()).hexdigest() for f in files}
    meta = {
        "version": 1,
        "id": uuid.uuid4().hex[:12],
        "frame": "map",
        "calibrated": True,
        "max_rms": 0.03,
        "calibration": {
            "source": [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
            "target": [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
            "transform": {"x": 0.0, "y": 0.0, "yaw": 0.0,
                          "scale_ratio": 1.0, "rms": 0.0, "maximum": 0.0},
        },
        "edge_widths": widths,
        "narrow_enabled": True,
        "narrow_nodes": [],
        "sources": {"localization": "sandbox_20261031_2025"},
        "sha256": sha,
    }
    (OUT / "sandbox.json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    return OUT


def build():
    skel = largest(thin(lat))
    nodes = sample_waypoints(skel)
    print("skeleton px %d  waypoints %d" % (int(skel.sum()), len(nodes)))

    keep_grid = dict(width=W, height=H, resolution=RES, origin=(OX, OY, 0.0),
                     data=np.flipud(np.where(keep, 0, 100)).astype(np.uint8).ravel().tolist())
    d_edge = ndi.distance_transform_edt(keep) * RES

    edges = []
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            a, b = nodes[i], nodes[j]
            dist = math.dist(a, b)
            if dist > LINK or dist < 0.05:
                continue
            if not sweep_ok(keep_grid, a, b):
                continue
            wdt = width_along(a, b, d_edge)
            if wdt < 0.23:
                print("  narrow link %d-%d width %.2f" % (i, j, wdt))
            edges.append((i, j, wdt))
    print("edges %d" % len(edges))
    write_project(keep, nodes, edges)
    print("wrote", OUT)


if __name__ == "__main__":
    build()

