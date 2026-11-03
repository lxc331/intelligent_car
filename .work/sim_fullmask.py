"""Would a fuller keepout mask make the drawn network pass the costmap test?

The current keepout erodes the road area by a couple of cells before the
costmap inflation takes another 0.105 m, so the band left for the car is
narrower than the 0.23 m body. This script rebuilds the keepout from the lidar
map (plate minus obstacles, no extra erosion), simulates the costmap that would
be produced from it, and reports how many drawn edges pass.
"""
import json
import math

import numpy as np
from PIL import Image
from scipy import ndimage as ndi

RES = 0.05
ORIGIN = (-5.04, -7.53)
FOOT = (0.25, 0.15, 0.115)
HALF_CELL = RES / 2
PLATE = [(2.708, 6.453), (0.604, 4.386), (4.564, 0.356), (6.668, 2.424)]

real = np.array(Image.open(r".work\maps\map_20261031_2025\map.pgm").convert("L"))
occ = real <= 60
h, w = real.shape
free = real >= 250


def plate_mask():
    from matplotlib.path import Path  # only used for a point-in-polygon test

    cols, rows = np.meshgrid(np.arange(w), np.arange(h))
    xs = cols * RES + RES / 2 + ORIGIN[0]
    ys = rows * RES + RES / 2 + ORIGIN[1]
    pts = np.column_stack([xs.ravel(), ys.ravel()])
    return Path(PLATE).contains_points(pts).reshape(h, w)


inside = plate_mask()
mask = inside & free & ~ndi.binary_dilation(occ, iterations=1)
lethal = (~mask) | occ
d = ndi.distance_transform_edt(~lethal) * RES
band = d >= 0.105 + 0.004         # inscribed radius + one cell of slack

print("mask area m2 %.2f  free band m2 %.2f" % (mask.sum() * RES * RES, band.sum() * RES * RES))

nodes = json.load(open(r".work\net_nodes.json", encoding="utf-8"))
pos = {int(i): tuple(p) for i, p, _ in nodes}
links = {int(i): [int(t) for t in to] for i, _, to in nodes}
edges = sorted({(min(x, y), max(x, y)) for x, ts in links.items() for y in ts})


def pose_clear(x, y, yaw, grid):
    front, rear, half = FOOT
    a_ = (front + rear) / 2
    cx = x + (front - rear) / 2 * math.cos(yaw)
    cy = y + (front - rear) / 2 * math.sin(yaw)
    ex = abs(math.cos(yaw)) * a_ + abs(math.sin(yaw)) * half
    ey = abs(math.sin(yaw)) * a_ + abs(math.cos(yaw)) * half
    c0 = int(math.floor((cx - ex - ORIGIN[0]) / RES))
    c1 = int(math.floor((cx + ex - ORIGIN[0]) / RES))
    r0 = int(math.floor((cy - ey - ORIGIN[1]) / RES))
    r1 = int(math.floor((cy + ey - ORIGIN[1]) / RES))
    for r in range(r0, r1 + 1):
        for c in range(c0, c1 + 1):
            if not (0 <= r < h and 0 <= c < w) or grid[r, c]:
                if not (0 <= r < h and 0 <= c < w):
                    return False
                if grid[r, c] and (0 <= r < h and 0 <= c < w):
                    dx = (c * RES + RES / 2 + ORIGIN[0]) - cx
                    dy = (r * RES + RES / 2 + ORIGIN[1]) - cy
                    cc, ss = math.cos(yaw), math.sin(yaw)
                    if abs(cc * dx + ss * dy) <= a_ + HALF_CELL and abs(-ss * dx + cc * dy) <= half + HALF_CELL:
                        return False
    return True


def edge_ok(p, q, grid):
    yaw = math.atan2(q[1] - p[1], q[0] - p[0])
    distance = math.dist(p, q)
    n = max(1, int(math.ceil(distance / 0.02)))
    return all(pose_clear(p[0] + (q[0] - p[0]) * i / n, p[1] + (q[1] - p[1]) * i / n, yaw, grid) for i in range(n + 1))


blocked = ~band
bad = [e for e in edges if not edge_ok(pos[e[0]], pos[e[1]], blocked)]
print("edges %d blocked %d" % (len(edges), len(bad)))
print("blocked", ["%d-%d" % e for e in bad])
json.dump({"blocked": [list(e) for e in bad]}, open(r".work\net_fullmask_check.json", "w", encoding="utf-8"), indent=1)
