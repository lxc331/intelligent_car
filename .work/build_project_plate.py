"""Rebuild the project so keepout matches the physical road width.

Measured problem: the shipped keepout was carved from an eroded drivable mask,
so corridors are ~0.40 m wide; costmap inflation then leaves only ~0.17 m for a
0.23 m body and every runtime swept check fails. This rebuilds keepout as
plate-hull minus lidar obstacles (no extra erosion), so corridors get their
real ~0.50 m width, then reports:
  * how many edges plan_route keeps,
  * how many edges would pass the runtime costmap check.
"""
import hashlib
import json
import math
import os

import numpy as np
from PIL import Image, ImageFilter
from scipy import ndimage as ndi


RES = 0.05
ORIGIN = [-5.04, -7.53]
SRC = r"app/savedMap/sandbox_nav_ready_20261101"
OUT = r"app/savedMap/sandbox_nav_plate_20261101"
FOOT = (0.25, 0.15, 0.115)

map_pgm = Image.open(SRC + "/map.pgm").convert("L")
w, h = map_pgm.size
occ = np.array(map_pgm) <= 60

import json as _json
import math as _math

_reg = _json.load(open(r"app/savedMap/sandbox_lidar_20261031/sandbox.json", encoding="utf-8"))["calibration"]["transform"]
_c, _s = _math.cos(_reg["yaw"]), _math.sin(_reg["yaw"])
poly = np.array([(_c * x - _s * y + _reg["x"], _s * x + _c * y + _reg["y"])
                 for x, y in ((0, 0), (2.97, 0), (2.97, 5.40), (0, 5.40))])

gc, gr = np.meshgrid(np.arange(w), np.arange(h))
xs = gc * RES + ORIGIN[0]
ys = gr * RES + ORIGIN[1]
inside = np.ones((h, w), dtype=bool)
for k in range(len(poly)):
    x1, y1 = poly[k]
    x2, y2 = poly[(k + 1) % len(poly)]
    inside &= (x2 - x1) * (ys - y1) - (y2 - y1) * (xs - x1) >= -1e-9

obstacles = np.array(Image.fromarray((occ * 255).astype("uint8")).filter(ImageFilter.MaxFilter(3))) > 0
keep = inside & ~obstacles
print("keepout %.2f m2" % (keep.sum() * 0.0025))

nodes = json.load(open(r".work\net_nodes.json", encoding="utf-8"))
pos = {int(i): tuple(p) for i, p, _ in nodes}
links = {int(i): [int(t) for t in to] for i, _, to in nodes}
edges = sorted({(min(x, y), max(x, y)) for x, ts in links.items() for y in ts})

d_out = ndi.distance_transform_edt(keep) * RES           # distance to the mask border
d_occ = ndi.distance_transform_edt(~occ) * RES           # distance to real obstacles (map)
# Runtime: cells within inscribed radius (+ half a cell of raster padding) of a
# lethal cell are 253. Here lethal = mask border or real obstacle.
blocked_runtime = (d_out < 0.115 + 0.025) | (d_occ < 0.115 + 0.025)


def samples(p, q, step=0.02):
    n = max(1, int(math.ceil(math.dist(p, q) / step)))
    return [(p[0] + (q[0] - p[0]) * i / n, p[1] + (q[1] - p[1]) * i / n) for i in range(n + 1)]


def in_keep(p):
    col = int(round((p[0] - ORIGIN[0]) / RES))
    row = int(round((p[1] - ORIGIN[1]) / RES))
    return 0 <= row < h and 0 <= col < w and keep[row, col]


def clear_pose(x, y, yaw, grid):
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
            if grid[r, c]:
                dx = (c * RES + RES / 2 + ORIGIN[0]) - cx
                dy = (r * RES + RES / 2 + ORIGIN[1]) - cy
                cc, ss = math.cos(yaw), math.sin(yaw)
                if abs(cc * dx + ss * dy) <= a_ + RES / 2 and abs(-ss * dx + cc * dy) <= half + RES / 2:
                    return False
    return True


def edge_clear(a_, b, grid, halfw):
    pa, pb = pos[a_], pos[b]
    yaw = math.atan2(pb[1] - pa[1], pb[0] - pa[0])
    return all(clear_pose(x, y, yaw, grid) for x, y in samples(pa, pb))


inside_mask = []
for a_, b in edges:
    ok = all(in_keep(p) for p in samples(pos[a_], pos[b]))
    inside_mask.append(ok)
runtime_ok = sum(1 for a_, b in edges if edge_clear(a_, b, blocked_runtime, 0.115))
print("edges in mask %d / %d ; pass runtime simulation %d" % (sum(inside_mask), len(edges), runtime_ok))

# edge widths from mask geometry
edge_widths = {}
for a_, b in edges:
    pa, pb = pos[a_], pos[b]
    length = math.dist(pa, pb)
    n = max(1, int(math.ceil(length / 0.05)))
    clear = 1e9
    for i in range(n + 1):
        x = pa[0] + (pb[0] - pa[0]) * i / n
        y = pa[1] + (pb[1] - pa[1]) * i / n
        col = int(round((x - ORIGIN[0]) / RES))
        row = int(round((y - ORIGIN[1]) / RES))
        if not keep[row, col]:
            clear = 0.0
            break
        clear = min(clear, d_out[row, col])
    width = round(2 * clear, 3) if clear < 1e8 else 0.0
    edge_widths["%d:%d" % (a_, b)] = width
    edge_widths["%d:%d" % (b, a_)] = width

os.makedirs(OUT, exist_ok=True)
map_pgm.save(OUT + "/map.pgm")
mask_img = Image.fromarray(np.where(keep, 255, 0).astype("uint8"))
mask_img.save(OUT + "/roads.pgm")
mask_img.save(OUT + "/keepout.pgm")
yaml = """image: {name}.pgm
mode: trinary
resolution: 0.05
origin: [-5.04, -7.53, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
for name in ("map", "roads", "keepout"):
    open(OUT + "/" + name + ".yaml", "w", encoding="utf-8", newline="\n").write(yaml.format(name=name))

lines = ["waypoints:"]
for i in sorted(pos):
    lines.append("  - id: %d" % i)
    lines.append("    pose: [%.4f, %.4f]" % pos[i])
    lines.append("    to: %s" % json.dumps(sorted(links[i])))
    lines.append("    name: 路口%d" % i)
open(OUT + "/road_network.yaml", "w", encoding="utf-8", newline="\n").write("\n".join(lines) + "\n")

files = ["map.pgm", "map.yaml", "roads.pgm", "roads.yaml", "keepout.pgm", "keepout.yaml", "road_network.yaml"]
sha = {n: hashlib.sha256(open(OUT + "/" + n, "rb").read()).hexdigest() for n in files}
meta = {
    "version": 1,
    "id": "sandbox_nav_plate_20261101",
    "frame": "map",
    "calibrated": True,
    "max_rms": 0.075,
    "calibration": {
        "source": [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
        "target": [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
        "transform": {"x": 0.0, "y": 0.0, "yaw": 0.0, "rms": 0.0, "maximum": 0.0, "scale_ratio": 1.0},
    },
    "edge_widths": edge_widths,
    "narrow_enabled": True,
    "narrow_nodes": [],
    "sources": {"localization": "map.yaml", "roads": "roads.yaml"},
    "sha256": sha,
}
json.dump(meta, open(OUT + "/sandbox.json", "w", encoding="utf-8", newline="\n"), ensure_ascii=False, indent=2)
print("wrote", OUT, "min width %.2f" % min(edge_widths.values()))
