# -*- coding: utf-8 -*-
"""v5: graph = skeleton (medial axis) of the drivable surface, sampled every
SPACING metres, with every edge validated by the same swept-rectangle check the
navigation page will run.  Project is written in the localization-map frame.
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
CLEARANCE = 0.05
DISTRICT_CLOSE = 0.13
DISTRICT_GROW = 0.02
PLATE_CENTRE = (1.297, -0.491)
PLATE_THETA = math.radians(46.97)
PLATE_LONG = 5.400
PLATE_SHORT = 2.960
EDGE_MIN = 0.23
FOOTPRINT = (0.25, 0.15, 0.115)
SPUR_ITERS = 8

MAP_YAML = yaml.safe_load((MAP_DIR / "map.yaml").read_text(encoding="utf-8"))
IMAGE = np.array(Image.open(MAP_DIR / MAP_YAML["image"]).convert("L"))
RES = float(MAP_YAML["resolution"])
OX, OY = float(MAP_YAML["origin"][0]), float(MAP_YAML["origin"][1])
H, W = IMAGE.shape
free = IMAGE >= 250
occ = IMAGE <= 60
d_occ = ndi.distance_transform_edt(~occ) * RES

sys.path.insert(0, str(Path("app").resolve()))
from carapp.core.navigation_path import swept_clear  # noqa: E402


def plate_mask():
    yy, xx = np.mgrid[0:H, 0:W]
    x = xx * RES + OX
    y = (H - 1 - yy) * RES + OY
    c, s = math.cos(-PLATE_THETA), math.sin(-PLATE_THETA)
    u = c * (x - PLATE_CENTRE[0]) - s * (y - PLATE_CENTRE[1])
    v = s * (x - PLATE_CENTRE[0]) + c * (y - PLATE_CENTRE[1])
    return (np.abs(u) <= PLATE_LONG / 2) & (np.abs(v) <= PLATE_SHORT / 2)


def surfaces():
    closed = ndi.binary_closing(occ, structure=np.ones((3, 3), bool),
                               iterations=max(1, int(round(DISTRICT_CLOSE / RES / 2))))
    grown = ndi.binary_dilation(closed, iterations=max(1, int(round(DISTRICT_GROW / RES))))
    keep = free & plate_mask() & ~grown
    lat = ndi.binary_opening(keep & (d_occ >= CLEARANCE), structure=np.ones((2, 2), bool))
    return keep, lat


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
            b = (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9)
            a = (((p2 == 0) & (p3 == 1)).astype(np.uint8)
                 + ((p3 == 0) & (p4 == 1)).astype(np.uint8)
                 + ((p4 == 0) & (p5 == 1)).astype(np.uint8)
                 + ((p5 == 0) & (p6 == 1)).astype(np.uint8)
                 + ((p6 == 0) & (p7 == 1)).astype(np.uint8)
                 + ((p7 == 0) & (p8 == 1)).astype(np.uint8)
                 + ((p8 == 0) & (p9 == 1)).astype(np.uint8)
                 + ((p9 == 0) & (p2 == 1)).astype(np.uint8))
            if step == 0:
                cond = (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)
            else:
                cond = (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
            remove = (img == 1) & (b >= 2) & (b <= 6) & (a == 1) & cond
            if remove.any():
                img[remove] = 0
                changed = True
    return img.astype(bool)


def deg_map(skel):
    kern = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]])
    return ndi.convolve(skel.astype(np.uint8), kern, mode="constant")


def prune(skel, iters):
    for _ in range(iters):
        ends = skel & (deg_map(skel) == 1)
        if not ends.any():
            break
        skel = skel & ~ends
    return skel


def components(skel):
    lab, n = ndi.label(skel, structure=np.ones((3, 3), bool))
    if not n:
        return skel
    sizes = ndi.sum(skel, lab, range(1, n + 1))
    return lab == int(np.argmax(sizes)) + 1


def main_path(skel):
    """Longest geodesic path through the skeleton, as an ordered pixel list."""
    cells = [(int(r), int(c)) for r, c in zip(*np.nonzero(skel))]
    cells_set = set(cells)
    adj = {p: [] for p in cells}
    for (r, c) in cells:
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == dc == 0:
                    continue
                q = (r + dr, c + dc)
                if q in cells_set:
                    adj[(r, c)].append(q)

    def bfs(start):
        seen = {start: None}
        order = [start]
        head = 0
        while head < len(order):
            cur = order[head]
            head += 1
            for q in adj[cur]:
                if q not in seen:
                    seen[q] = cur
                    order.append(q)
        return order[-1], seen

    a, _ = bfs(cells[0])
    b, parents = bfs(a)
    path = [b]
    while parents[path[-1]] is not None:
        path.append(parents[path[-1]])
    return path[::-1]


def waypoints_along(path):
    wps = [to_xy(*path[0])]
    acc = 0.0
    for prev, cur in zip(path, path[1:]):
        acc += math.dist(to_xy(*prev), to_xy(*cur))
        if acc >= SPACING:
            wps.append(to_xy(*cur))
            acc = 0.0
    if math.dist(wps[-1], to_xy(*path[-1])) > 0.05:
        wps.append(to_xy(*path[-1]))
    return wps


def write_project(keep, wps):
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True)
    shutil.copy(MAP_DIR / MAP_YAML["image"], OUT / "map.pgm")
    map_text = (MAP_DIR / "map.yaml").read_text(encoding="utf-8")
    (OUT / "map.yaml").write_text(map_text, encoding="utf-8")
    pgm = b"P5\n%d %d\n255\n" % (W, H) + np.where(keep, 255, 0).astype(np.uint8).tobytes()
    for name in ("roads", "keepout"):
        (OUT / (name + ".pgm")).write_bytes(pgm)
        (OUT / (name + ".yaml")).write_text(
            map_text.replace("image: " + MAP_YAML["image"], "image: " + name + ".pgm"),
            encoding="utf-8")
    lines = ["waypoints:"]
    for i, (x, y) in enumerate(wps):
        lines += ["  - id: %d" % i, "    pose: [%.4f, %.4f]" % (x, y),
                  "    to: []", "    name: 璺彛%d" % i]
    (OUT / "road_network.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return OUT


def build():
    keep, lat = surfaces()
    dt = ndi.distance_transform_edt(lat) * RES
    skel = components(prune(thin(lat), SPUR_ITERS))
    print("skeleton px %d, max clearance %.2f m" % (int(skel.sum()), dt.max()))
    path = main_path(skel)
    print("main corridor path %d px, %.1f m" % (len(path), len(path) * RES))
    wps = waypoints_along(path)
    print("waypoints %d" % len(wps))

    keep_grid = dict(width=W, height=H, resolution=RES, origin=(OX, OY, 0.0),
                     data=np.flipud(np.where(keep, 0, 100)).astype(np.uint8).ravel().tolist())
    obs_grid = dict(width=W, height=H, resolution=RES, origin=(OX, OY, 0.0),
                    data=np.flipud(np.where(free, 0, 100)).astype(np.uint8).ravel().tolist())

    edges = []
    for u in range(len(wps) - 1):
        a, b = wps[u], wps[u + 1]
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        pts = ((a[0], a[1], yaw), (b[0], b[1], yaw))
        if len(pts) == 2:
            ok_keep = swept_clear(keep_grid, pts, FOOTPRINT)
            ok_obs = swept_clear(obs_grid, pts, FOOTPRINT)
        if ok_keep and ok_obs:
            edges.append((u, u + 1, 0.0))
        else:
            print("  edge %d->%d rejected (keep=%s obs=%s)" % (u, u + 1, ok_keep, ok_obs))
    print("kept edges %d of %d" % (len(edges), max(0, len(wps) - 1)))
    write_project(keep, wps)
    print("wrote", OUT)


if __name__ == "__main__":
    build()


