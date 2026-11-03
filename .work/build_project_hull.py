"""Third attempt: keepout = convex hull of the sandbox plate minus lidar obstacles.

The plate (the real drivable rectangle) is recovered as the convex hull of the
existing keepout mask, which already spans the plate. Allowing the hull minus
obstacles gives the costmap a free band wide enough for the 0.23 m body while
still keeping the car off the room floor and the model districts.
"""
import hashlib
import json
import math
import os

import numpy as np
from PIL import Image, ImageFilter
from scipy.spatial import ConvexHull

RES = 0.05
ORIGIN = [-5.04, -7.53]
SRC = r"app/savedMap/sandbox_nav_ready_20261101"
OUT = r"app/savedMap/sandbox_nav_hull20261101"

map_pgm = Image.open(SRC + "/map.pgm").convert("L")
w, h = map_pgm.size
real = np.array(map_pgm)
occ = real <= 60

old = np.array(Image.open(SRC + "/keepout.pgm").convert("L")) >= 250
rows, cols = np.nonzero(old)
pts = np.column_stack([cols * RES + ORIGIN[0], rows * RES + ORIGIN[1]])
hull = ConvexHull(pts)
poly = pts[hull.vertices]
print("hull vertices", len(poly))

# rasterise the hull with a half-pixel scan test
grid_cols, grid_rows = np.meshgrid(np.arange(w), np.arange(h))
xs = grid_cols * RES + ORIGIN[0]
ys = grid_rows * RES + ORIGIN[1]
inside = np.ones((h, w), dtype=bool)
for k in range(len(poly)):
    x1, y1 = poly[k]
    x2, y2 = poly[(k + 1) % len(poly)]
    cross = (x2 - x1) * (ys - y1) - (y2 - y1) * (xs - x1)
    inside &= cross >= -1e-9

obstacles = np.array(Image.fromarray((occ * 255).astype("uint8")).filter(ImageFilter.MaxFilter(3))) > 0
keep = inside & ~obstacles
print("keepout cells %d (%.2f m2)" % (keep.sum(), keep.sum() * 0.0025))

nodes = json.load(open(r".work\net_nodes.json", encoding="utf-8"))
pos = {int(i): tuple(p) for i, p, _ in nodes}
links = {int(i): [int(t) for t in to] for i, _, to in nodes}
edges = sorted({(min(x, y), max(x, y)) for x, ts in links.items() for y in ts})


def sampled(p, q, step=0.02):
    length = math.dist(p, q)
    n = max(1, int(math.ceil(length / step)))
    for i in range(n + 1):
        yield p[0] + (q[0] - p[0]) * i / n, p[1] + (q[1] - p[1]) * i / n


def inside_keep(x, y):
    col = int(round((x - ORIGIN[0]) / RES))
    row = int(round((y - ORIGIN[1]) / RES))
    return 0 <= row < h and 0 <= col < w and keep[row, col]


edge_widths = {}
bad = 0
for a_, b in edges:
    if not all(inside_keep(x, y) for x, y in sampled(pos[a_], pos[b])):
        bad += 1
        width = 0.0
    else:
        width = 0.50
    edge_widths["%d:%d" % (a_, b)] = width
    edge_widths["%d:%d" % (b, a_)] = width
print("edges outside keepout %d" % bad)

os.makedirs(OUT, exist_ok=True)
map_pgm.save(OUT + "/map.pgm")
mask = np.where(keep, 255, 0).astype("uint8")
Image.fromarray(mask).save(OUT + "/roads.pgm")
Image.fromarray(mask).save(OUT + "/keepout.pgm")
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
    "id": "sandbox_nav_hull20261101",
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
print("wrote", OUT)
