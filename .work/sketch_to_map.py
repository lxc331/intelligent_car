"""Convert the hand-drawn sandbox sketch into a navigation map draft + road network.

Input : .work/map_sketch/user_sketch.png  (grey = drivable, black/green/blue = blocked)
Output: .work/map_sketch/map_sketch.pgm/.yaml, road_network_sketch.yaml, preview.png

Scale is provisional: SCALE_M_PER_PX.  Correct it once the real size is known.
"""
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(r"D:\项目\intelligent_car-main")
OUT = ROOT / ".work" / "map_sketch"
SRC = Path(r"C:\Users\AAA\.dsh\attachments\v1\objects\ff\ff56b3d09d773014cf84fd260a2dd32c9c55cff59cbd8337bff0d62265fd3daf")
SCALE_M_PER_PX = 0.004          # provisional: whole drawing = 824*0.004 x 560*0.004 = 3.30 x 2.24 m
CLEARANCE_M = 0.07               # keep nodes this far from walls
NODE_STEP_M = 0.35               # spacing along straight corridors
RES = 0.02                       # map resolution (m/cell) for the pgm

img = Image.open(SRC).convert("RGB")
arr = np.asarray(img).astype(np.int16)
r, g, b = arr[..., 0], arr[..., 1], arr[..., 2]

grey = (np.abs(r - g) < 22) & (np.abs(g - b) < 22) & (r > 95) & (r < 190)
black = (r < 70) & (g < 70) & (b < 70)
green = (g > 110) & (r < 110) & (b < 110)
blue = (b > 150) & (r < 120) & (g > 90)
red = (r > 170) & (g < 90) & (b < 90)

free = (grey & ~red).copy()
blocked = black | green | blue
free &= ~blocked

# --- occupancy grid for map_server (resolution RES) -------------------------
step = max(1, int(round(RES / SCALE_M_PER_PX)))
h, w = free.shape
gh, gw = h // step, w // step
grid = np.full((gh, gw), 100, dtype=np.uint8)
for row in range(gh):
    for col in range(gw):
        block = free[row * step:(row + 1) * step, col * step:(col + 1) * step]
        if block.mean() > 0.65:
            grid[row, col] = 0
        elif blocked[row * step:(row + 1) * step, col * step:(col + 1) * step].mean() > 0.3:
            grid[row, col] = 100
        else:
            grid[row, col] = 205

flipped = np.flipud(grid)  # PGM row 0 = top; ROS origin bottom-left
pgm_path = OUT / "map_sketch.pgm"
with pgm_path.open("wb") as handle:
    handle.write(b"P5\n%d %d\n255\n" % (gw, gh))
    handle.write(flipped.tobytes())
(OUT / "map_sketch.yaml").write_text(
    "image: map_sketch.pgm\n"
    "resolution: %.4f\n"
    "origin: [0.0, 0.0, 0.0]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.25\n" % RES,
    encoding="utf-8",
)

# --- waypoint extraction on a coarse lattice -------------------------------
from scipy import ndimage

dist_px = ndimage.distance_transform_edt(free)
spacing_px = int(round(NODE_STEP_M / SCALE_M_PER_PX))
clear_px = CLEARANCE_M / SCALE_M_PER_PX

cells = {}
rows = range(spacing_px // 2, h, spacing_px)
cols = range(spacing_px // 2, w, spacing_px)
for py in rows:
    for px in cols:
        if dist_px[py, px] >= clear_px:
            cells[(px, py)] = True

def world(px, py):
    return (round(px * SCALE_M_PER_PX, 3), round((h - py) * SCALE_M_PER_PX, 3))

def line_ok(a, b):
    steps = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) / 2) + 1
    for i in range(steps + 1):
        t = i / steps
        px = int(round(a[0] + (b[0] - a[0]) * t))
        py = int(round(a[1] + (b[1] - a[1]) * t))
        if not (0 <= py < h and 0 <= px < w) or dist_px[py, px] < clear_px:
            return False
    return True

points = list(cells)
index = {p: i for i, p in enumerate(points)}
edges = []
for i, a in enumerate(points):
    for dx in (-spacing_px, 0, spacing_px):
        for dy in (-spacing_px, 0, spacing_px):
            if dx == 0 and dy == 0:
                continue
            nb = (a[0] + dx, a[1] + dy)
            j = index.get(nb)
            if j is None or j <= i:
                continue
            if line_ok(a, nb):
                edges.append((i, j))

# thin: keep junctions and every third node of a chain
deg = np.zeros(len(points), dtype=int)
for i, j in edges:
    deg[i] += 1
    deg[j] += 1
keep = {i for i in range(len(points)) if deg[i] != 2}
for i in range(len(points)):
    if i not in keep and i % 3 == 0:
        keep.add(i)

neigh = {i: set() for i in keep}
for i, j in edges:
    if i in keep and j in keep:
        neigh[i].add(j)
    elif i in keep or j in keep:
        pass

ids = {i: n for n, i in enumerate(sorted(keep))}
lines = ["# sandbox road network drawn from the hand sketch (scale provisional)", "waypoints:"]
for i in sorted(keep):
    px, py = points[i]
    x, y = world(px, py)
    to = sorted(ids[j] for j in neigh[i] if j in ids)
    lines.append("  - id: %d" % ids[i])
    lines.append("    pose: [%.3f, %.3f]" % (x, y))
    lines.append("    to: [%s]" % ", ".join(str(t) for t in to))
(OUT / "road_network_sketch.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")

# --- preview ---------------------------------------------------------------
prev = img.copy()
draw = ImageDraw.Draw(prev)
font = ImageFont.truetype(r"C:\Windows\Fonts\msyh.ttc", 13)
for i, j in edges:
    if i in keep and j in keep:
        draw.line([points[i], points[j]], fill=(255, 210, 90), width=2)
for i in sorted(keep):
    px, py = points[i]
    draw.ellipse([px - 4, py - 4, px + 4, py + 4], fill=(255, 90, 90))
    draw.text((px + 6, py - 6), str(ids[i]), font=font, fill=(255, 240, 200))
draw.text((10, 8), "路网节点 %d 个 / 边 %d 条（比例暂定 %.3f m/px）" % (len(keep), sum(len(v) for v in neigh.values()) // 2, SCALE_M_PER_PX),
          font=font, fill=(255, 220, 120))
prev.save(OUT / "road_network_preview.png")
print("nodes", len(keep), "edges", sum(len(v) for v in neigh.values()) // 2)
print("map", gw, "x", gh, "cells at", RES, "m ->", round(gw * RES, 2), "x", round(gh * RES, 2), "m")
print("wrote", OUT)
