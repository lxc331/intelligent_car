"""Rebuild a self-consistent sandbox project around the drawn road network.

Problem being fixed: the shipped sandbox_nav_ready_20261101 project has a
keepout mask that no longer contains its own road network (only 46 of 110
waypoints sit on traversable cells), so plan_route fails on every pair.

New rule: keepout white = thick band (WIDTH) around every network edge, minus
lidar obstacles. The band never leaves the drawn corridors, so the car keeps
the same permitted area while every edge is traversable by definition.
"""
import hashlib
import json
import math
import sys

import numpy as np
from PIL import Image, ImageDraw

RES = 0.05
ORIGIN = [-5.04, -7.53]
SRC = r"app/savedMap/sandbox_nav_ready_20261101"
OUT = sys.argv[1] if len(sys.argv) > 1 else r"app/savedMap/sandbox_nav_w20261101"
WIDTH = float(sys.argv[2]) if len(sys.argv) > 2 else 0.50     # corridor width in m

map_pgm = Image.open(SRC + "/map.pgm").convert("L")
w, h = map_pgm.size
real = np.array(map_pgm)
occ = real <= 60

nodes = json.load(open(r".work\net_nodes.json", encoding="utf-8"))
pos = {int(i): tuple(p) for i, p, _ in nodes}
links = {int(i): [int(t) for t in to] for i, _, to in nodes}
edges = sorted({(min(x, y), max(x, y)) for x, ts in links.items() for y in ts})

canvas = Image.new("L", (w, h), 0)
draw = ImageDraw.Draw(canvas)


def pixel(p):
    return ((p[0] - ORIGIN[0]) / RES, (p[1] - ORIGIN[1]) / RES)


half = WIDTH / 2 / RES
for a_, b in edges:
    draw.line([pixel(pos[a_]), pixel(pos[b])], fill=255, width=int(round(half * 2)))
for p in pos.values():
    x, y = pixel(p)
    r = half + 2
    draw.ellipse([x - r, y - r, x + r, y + r], fill=255)

band = np.array(canvas) > 0
from PIL import ImageFilter
obstacles = np.array(Image.fromarray((occ * 255).astype("uint8")).filter(ImageFilter.MaxFilter(3))) > 0
band = band & ~obstacles

Image.fromarray(np.where(band, 255, 0).astype("uint8")).save(r".work\maps\new_keepout_preview.pgm")
print("keepout cells %d (%.2f m2), width %.2f m" % (band.sum(), band.sum() * .0025, WIDTH))

# edge widths from the new mask
dist = np.array(Image.fromarray(np.where(band, 0, 255).astype("uint8"))).astype(float)
edge_widths = {}
for a_, b in edges:
    pa, pb = pos[a_], pos[b]
    length = math.dist(pa, pb)
    n = max(1, int(math.ceil(length / 0.02)))
    clear = 1e9
    for i in range(n + 1):
        x = pa[0] + (pb[0] - pa[0]) * i / n
        y = pa[1] + (pb[1] - pa[1]) * i / n
        col = int(round((x - ORIGIN[0]) / RES))
        row = int(round((y - ORIGIN[1]) / RES))
        if not band[row, col]:
            clear = 0.0
            break
        # local half width: walk perpendicular until the band ends
        dx, dy = pb[0] - pa[0], pb[1] - pa[1]
        norm = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / norm, dx / norm
        reach = 0.0
        for k in np.arange(0.025, 0.45, 0.025):
            c2 = int(round((x + nx * k - ORIGIN[0]) / RES))
            r2 = int(round((y + ny * k - ORIGIN[1]) / RES))
            c3 = int(round((x - nx * k - ORIGIN[0]) / RES))
            r3 = int(round((y - ny * k - ORIGIN[1]) / RES))
            if not band[r2, c2] or not band[r3, c3]:
                break
            reach = k
        clear = min(clear, reach)
    width = 2 * clear
    edge_widths["%d:%d" % (a_, b)] = round(width, 3)
    edge_widths["%d:%d" % (b, a_)] = round(width, 3)

import os
os.makedirs(OUT, exist_ok=True)
map_pgm.save(OUT + "/map.pgm")
Image.fromarray(np.where(band, 255, 0).astype("uint8")).save(OUT + "/roads.pgm")
Image.fromarray(np.where(band, 255, 0).astype("uint8")).save(OUT + "/keepout.pgm")

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
    lines.append("    name: 璺彛%d" % i)
open(OUT + "/road_network.yaml", "w", encoding="utf-8", newline="\n").write("\n".join(lines) + "\n")

files = ["map.pgm", "map.yaml", "roads.pgm", "roads.yaml", "keepout.pgm", "keepout.yaml", "road_network.yaml"]
sha = {n: hashlib.sha256(open(OUT + "/" + n, "rb").read()).hexdigest() for n in files}
meta = {
    "version": 1,
    "id": "sandbox_nav_w20261101",
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

