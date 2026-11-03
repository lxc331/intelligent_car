"""Widen the keepout corridor mask to the true road width and re-test the network.

The shipped keepout was carved from an already-eroded drivable mask, so after
costmap inflation only ~0.20 m of free band was left for a 0.23 m wide car -
every edge failed. This rebuilds keepout as: existing mask grown by N cells,
limited to the sandbox plate, minus lidar obstacles. It then simulates the
costmap (253 = within 0.105 m of lethal) and counts blocked edges.
"""
import json
import math
import sys

import numpy as np
from PIL import Image
from scipy import ndimage as ndi

RES = 0.05
ORIGIN = (-5.04, -7.53)
FOOT = (0.25, 0.15, 0.115)
HALF_CELL = RES / 2
GROW = int(sys.argv[1]) if len(sys.argv) > 1 else 2

PROJECT = r"app/savedMap/sandbox_nav_ready_20261101"
keep = np.array(Image.open(PROJECT + "/keepout.pgm").convert("L")) >= 250
real = np.array(Image.open(r".work\maps\map_20261031_2025\map.pgm").convert("L"))
occ = real <= 60
h, w = keep.shape

# Plate rectangle in map frame (centre, yaw, size measured by redraw_plan_from_lidar_v2)
CX, CY, THETA, HU, HV = 1.297, -0.491, math.radians(46.97), 5.400, 2.960
c, s = math.cos(THETA), math.sin(THETA)
cols, rows = np.meshgrid(np.arange(w), np.arange(h))
xs = cols * RES + RES / 2 + ORIGIN[0]
ys = rows * RES + RES / 2 + ORIGIN[1]
du = (xs - CX) * c + (ys - CY) * s
dv = -(xs - CX) * s + (ys - CY) * c
plate = (np.abs(du) <= HU / 2) & (np.abs(dv) <= HV / 2)

grown = ndi.binary_dilation(keep, iterations=GROW)
new_keep = grown & ~ndi.binary_dilation(occ, iterations=1)
print("keepout white cells: %d -> %d (%.2f -> %.2f m2)" % (keep.sum(), new_keep.sum(), keep.sum() * .0025, new_keep.sum() * .0025))

lethal = ~new_keep
d = ndi.distance_transform_edt(~lethal) * RES
band = d >= 0.105 + 0.004
print("simulated costmap free band %.2f m2" % (band.sum() * .0025))

nodes = json.load(open(r".work\net_nodes.json", encoding="utf-8"))
pos = {int(i): tuple(p) for i, p, _ in nodes}
links = {int(i): [int(t) for t in to] for i, _, to in nodes}
edges = sorted({(min(x, y), max(x, y)) for x, ts in links.items() for y in ts})
blocked = ~band


def pose_clear(x, y, yaw):
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
    if not (0 <= r0 < h and 0 <= c0 < w and 0 <= r1 < h and 0 <= c1 < w):
        return False
    for r in range(r0, r1 + 1):
        for c in range(c0, c1 + 1):
            if blocked[r, c]:
                dx = (c * RES + RES / 2 + ORIGIN[0]) - cx
                dy = (r * RES + RES / 2 + ORIGIN[1]) - cy
                cc, ss = math.cos(yaw), math.sin(yaw)
                if abs(cc * dx + ss * dy) <= a_ + HALF_CELL and abs(-ss * dx + cc * dy) <= half + HALF_CELL:
                    return False
    return True


def edge_ok(p, q):
    yaw = math.atan2(q[1] - p[1], q[0] - p[0])
    distance = math.dist(p, q)
    n = max(1, int(math.ceil(distance / 0.02)))
    return all(pose_clear(p[0] + (q[0] - p[0]) * i / n, p[1] + (q[1] - p[1]) * i / n, yaw) for i in range(n + 1))


bad = [e for e in edges if not edge_ok(pos[e[0]], pos[e[1]])]
print("edges %d blocked %d" % (len(edges), len(bad)))
print("blocked", ["%d-%d" % e for e in bad][:60])
Image.fromarray(np.where(new_keep, 255, 0).astype("uint8")).save(r".work\maps\keepout_grown.pgm")
