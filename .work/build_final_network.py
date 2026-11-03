"""Final attempt: build the sandbox road network straight from the lidar map.

keepout = plate quad minus lidar obstacles (12.16 m^2, i.e. the real road area).
Network = centreline skeleton of that area, sampled every 0.15 m, edges pruned
by the same rectangle-swept test the car runs against its inflated costmap
(253 = within 0.14 m of lethal).
"""
import hashlib
import json
import math
import os

import numpy as np
import yaml
from PIL import Image, ImageFilter
from scipy import ndimage as ndi

RES = 0.05
ORIGIN = [-5.04, -7.53]
FOOT = (0.25, 0.15, 0.115)
PAD = 0.025
SRC = r"app/savedMap/sandbox_lidar_20261031"
OUT = r"app/savedMap/sandbox_nav_final_20261101"

meta = json.load(open(SRC + "/sandbox.json", encoding="utf-8"))
tr = meta["calibration"]["transform"]
c, s = math.cos(tr["yaw"]), math.sin(tr["yaw"])
corners = [(c * x - s * y + tr["x"], s * x + c * y + tr["y"])
           for x, y in ((0, 0), (2.97, 0), (2.97, 5.40), (0, 5.40))]

mp = Image.open(SRC + "/map.pgm").convert("L")
w, h = mp.size
occ = np.array(mp) <= 60
obst = np.array(Image.fromarray((occ * 255).astype("uint8")).filter(ImageFilter.MaxFilter(3))) > 0
gc, gr = np.meshgrid(np.arange(w), np.arange(h))
xs = gc * RES + ORIGIN[0]
ys = gr * RES + ORIGIN[1]
inside = np.ones((h, w), dtype=bool)
for k in range(4):
    x1, y1 = corners[k]
    x2, y2 = corners[(k + 1) % 4]
    inside &= (x2 - x1) * (ys - y1) - (y2 - y1) * (xs - x1) >= -1e-9
keep = inside & ~obst
d_out = ndi.distance_transform_edt(keep) * RES
d_occ = ndi.distance_transform_edt(~occ) * RES
bad = (d_out < 0.115 + PAD) | (d_occ < 0.115 + PAD)


def clear_pose(x, y, yaw, mask=None):
    mask = bad if mask is None else mask
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
    if not (0 <= r0 and 0 <= c0 and r1 < h and c1 < w):
        return False
    for r in range(r0, r1 + 1):
        for col in range(c0, c1 + 1):
            if mask[r, col]:
                dx = (col * RES + RES / 2 + ORIGIN[0]) - cx
                dy = (r * RES + RES / 2 + ORIGIN[1]) - cy
                cc, ss = math.cos(yaw), math.sin(yaw)
                if abs(cc * dx + ss * dy) <= a_ + RES / 2 and abs(-ss * dx + cc * dy) <= half + RES / 2:
                    return False
    return True


# Zhang-Suen thinning on the road area.
img = keep.astype(np.uint8).copy()


def neighbours(i):
    return (img[i - 1, j - 1], img[i - 1, j], img[i - 1, j + 1],
            img[i, j + 1], img[i + 1, j + 1], img[i + 1, j], img[i + 1, j - 1], img[i, j - 1])


changed = True
while changed:
    changed = False
    for step in (0, 1):
        remove = []
        for i in range(1, h - 1):
            for j in range(1, w - 1):
                if not img[i, j]:
                    continue
                p2, p3, p4, p5, p6, p7, p8, p9 = img[i - 1, j], img[i - 1, j + 1], img[i, j + 1], img[i + 1, j + 1], img[i + 1, j], img[i + 1, j - 1], img[i, j - 1], img[i - 1, j - 1]
                b = int(p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9)
                if b < 2 or b > 6:
                    continue
                a = int((not p2 and p3) + (not p3 and p4) + (not p4 and p5) + (not p5 and p6) +
                        (not p6 and p7) + (not p7 and p8) + (not p8 and p9) + (not p9 and p2))
                if a != 1:
                    continue
                if step == 0:
                    if p2 and p4 and p6:
                        continue
                    if p4 and p6 and p8:
                        continue
                else:
                    if p2 and p4 and p8:
                        continue
                    if p2 and p6 and p8:
                        continue
                remove.append((i, j))
        for i, j in remove:
            img[i, j] = 0
        changed = changed or bool(remove)

skel = img > 0
nb = ndi.convolve(skel.astype(np.uint8), np.ones((3, 3)), mode="constant") - skel
print("skel px", int(skel.sum()), "len m", round(skel.sum() * RES, 2))

# Sample nodes along the skeleton every >= 0.15 m, keeping track of pixel order.
pts = [(row, col) for row, col in zip(*np.nonzero(skel))]
pts_set = set(pts)
pos = []
for row, col in pts:
    x = col * RES + ORIGIN[0]
    y = row * RES + ORIGIN[1]
    if pos and math.hypot(x - pos[-1][0], y - pos[-1][1]) < 0.15:
        continue
    pos.append((x, y))

keep_nodes = []
for x, y in pos:
    ang = [k * math.pi / 8 for k in range(16)]
    if any(clear_pose(x, y, a) and clear_pose(x, y, a, occ) for a in ang):
        keep_nodes.append((x, y))
print("nodes %d / %d" % (len(keep_nodes), len(pos)))

# Connect nodes within 0.32 m if the straight segment passes both rasters.
nodes = {}
for idx, (x, y) in enumerate(keep_nodes):
    nodes[idx] = {"pose": [x, y], "to": set()}
for i in range(len(keep_nodes)):
    for j in range(i + 1, len(keep_nodes)):
        pa, pb = keep_nodes[i], keep_nodes[j]
        d = math.dist(pa, pb)
        if d > 0.70:
            continue
        yaw = math.atan2(pb[1] - pa[1], pb[0] - pa[0])
        n = max(1, int(math.ceil(d / 0.02)))
        ok = True
        for t in range(n + 1):
            x = pa[0] + (pb[0] - pa[0]) * t / n
            y = pa[1] + (pb[1] - pa[1]) * t / n
            if not clear_pose(x, y, yaw) or not clear_pose(x, y, yaw, occ):
                ok = False
                break
        if ok:
            nodes[i]["to"].add(j)
            nodes[j]["to"].add(i)
print("links", sum(len(v["to"]) for v in nodes.values()) // 2)

# Largest connected component.
seen = set()
best = set()
for i in nodes:
    if i in seen:
        continue
    stack = [i]
    comp = set()
    while stack:
        n = stack.pop()
        if n in comp:
            continue
        comp.add(n)
        stack.extend(t for t in nodes[n]["to"] if t not in comp)
    seen |= comp
    if len(comp) > len(best):
        best = comp
print("largest component", len(best))

alive = sorted(best)
remap = {old: new for new, old in enumerate(alive)}
lines = ["waypoints:"]
edge_widths = {}
for old in alive:
    new = remap[old]
    x, y = nodes[old]["pose"]
    tos = sorted(remap[t] for t in nodes[old]["to"] if t in remap)
    lines.append("  - id: %d" % new)
    lines.append("    pose: [%.4f, %.4f]" % (x, y))
    lines.append("    to: %s" % json.dumps(tos))
    lines.append("    name: 路口%d" % new)
    for t in tos:
        edge_widths["%d:%d" % (new, t)] = 0.5

os.makedirs(OUT, exist_ok=True)
mp.save(OUT + "/map.pgm")
# The keep array is indexed bottom-row-first (row 0 = smallest y); a canonical
# PGM stores the top row (largest y) first, so flip before saving.
mask_img = Image.fromarray(np.flipud(np.where(keep, 255, 0).astype("uint8")))
mask_img.save(OUT + "/keepout.pgm")
mask_img.save(OUT + "/roads.pgm")
yaml_text = """image: {name}.pgm
mode: trinary
resolution: 0.05
origin: [-5.04, -7.53, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
for name in ("map", "roads", "keepout"):
    open(OUT + "/" + name + ".yaml", "w", encoding="utf-8", newline="\n").write(yaml_text.format(name=name))
open(OUT + "/road_network.yaml", "w", encoding="utf-8", newline="\n").write("\n".join(lines) + "\n")

meta["id"] = "sandbox_nav_final_20261101"
meta["edge_widths"] = edge_widths
files = ["map.pgm", "map.yaml", "roads.pgm", "roads.yaml", "keepout.pgm", "keepout.yaml", "road_network.yaml"]
meta["sha256"] = {n: hashlib.sha256(open(OUT + "/" + n, "rb").read()).hexdigest() for n in files}
json.dump(meta, open(OUT + "/sandbox.json", "w", encoding="utf-8", newline="\n"), ensure_ascii=False, indent=2)
print("wrote", OUT, "nodes", len(alive), "edges", len(edge_widths))
