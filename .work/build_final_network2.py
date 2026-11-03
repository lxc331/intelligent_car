"""Sandbox navigation project whose keepout *is* the clearance band.

The earlier attempts kept the raw road area as keepout, so `plan_route` and the
car's costmap checks kept rejecting paths that clipped noise dots, 25 cm margins
or obstacles. Here the drivable raster is the band that is at least 0.14 m from
obstacles and 0.10 m inside the plate, which is exactly what a 0.40x0.23 m car
needs against the inflated costmap. Nodes sit on the band's centreline.

    python build_final_network2.py
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
SRC = r"app/savedMap/sandbox_lidar_20261031"
OUT = r"app/savedMap/sandbox_nav_band_20261101"
CLEAR_OCC = 0.14
CLEAR_OUT = 0.10
STEP = 0.15

meta = json.load(open(SRC + "/sandbox.json", encoding="utf-8"))
tr = meta["calibration"]["transform"]
c0, s0 = math.cos(tr["yaw"]), math.sin(tr["yaw"])
corners = [(c0 * x - s0 * y + tr["x"], s0 * x + c0 * y + tr["y"])
           for x, y in ((0, 0), (2.97, 0), (2.97, 5.40), (0, 5.40))]

mp = Image.open(SRC + "/map.pgm").convert("L")
w, h = mp.size
raw = np.array(mp)
occ = raw <= 60

# Drop speckle: hundreds of 1-3 px specks from lidar gaps otherwise punch the
# clearance band into dozens of islands.
lab, count = ndi.label(occ)
sizes = ndi.sum(occ, lab, range(1, count + 1))
keep_ids = {k + 1 for k, size in enumerate(sizes) if size >= 20}
occ_clean = np.isin(lab, list(keep_ids))
print("occupancy kept %d of %d cells, components %d -> %d"
      % (occ_clean.sum(), occ.sum(), count, len(keep_ids)))

gc, gr = np.meshgrid(np.arange(w), np.arange(h))
xs = gc * RES + ORIGIN[0]
ys = gr * RES + ORIGIN[1]
inside = np.ones((h, w), dtype=bool)
for k in range(4):
    x1, y1 = corners[k]
    x2, y2 = corners[(k + 1) % 4]
    inside &= (x2 - x1) * (ys - y1) - (y2 - y1) * (xs - x1) >= -1e-9

d_occ = ndi.distance_transform_edt(~occ_clean) * RES
d_out = ndi.distance_transform_edt(inside) * RES
band = inside & (d_occ >= CLEAR_OCC) & (d_out >= CLEAR_OUT)
print("band %.2f m2" % (band.sum() * RES * RES))


def clear_pose(x, y, yaw, mask):
    front, rear, half = 0.25, 0.15, 0.115
    a_ = (front + rear) / 2
    cx = x + (front - rear) / 2 * math.cos(yaw)
    cy = y + (front - rear) / 2 * math.sin(yaw)
    ex = abs(math.cos(yaw)) * a_ + abs(math.sin(yaw)) * half
    ey = abs(math.sin(yaw)) * a_ + abs(math.cos(yaw)) * half
    c_lo = int(math.floor((cx - ex - ORIGIN[0]) / RES))
    c_hi = int(math.floor((cx + ex - ORIGIN[0]) / RES))
    r_lo = int(math.floor((cy - ey - ORIGIN[1]) / RES))
    r_hi = int(math.floor((cy + ey - ORIGIN[1]) / RES))
    if c_lo < 0 or r_lo < 0 or c_hi >= w or r_hi >= h:
        return False
    cc, ss = math.cos(yaw), math.sin(yaw)
    for r in range(r_lo, r_hi + 1):
        for col in range(c_lo, c_hi + 1):
            if not mask[r, col]:
                continue
            dx = (col * RES + RES / 2 + ORIGIN[0]) - cx
            dy = (r * RES + RES / 2 + ORIGIN[1]) - cy
            if abs(cc * dx + ss * dy) <= a_ + RES / 2 and abs(-ss * dx + cc * dy) <= half + RES / 2:
                return False
    return True


blocked = (~band) | ndi.binary_dilation(occ_clean, np.ones((3, 3), bool))


def node_ok(x, y):
    return any(clear_pose(x, y, k * math.pi / 8, blocked) for k in range(16))


# Thin the band into centrelines.
img = band.astype(np.uint8).copy()
changed = True
while changed:
    changed = False
    for step in (0, 1):
        remove = []
        for i in range(1, h - 1):
            for j in range(1, w - 1):
                if not img[i, j]:
                    continue
                p2, p3, p4, p5 = img[i - 1, j], img[i - 1, j + 1], img[i, j + 1], img[i + 1, j + 1]
                p6, p7, p8, p9 = img[i + 1, j], img[i + 1, j - 1], img[i, j - 1], img[i - 1, j - 1]
                b = int(p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9)
                if b < 2 or b > 6:
                    continue
                a = int((not p2 and p3) + (not p3 and p4) + (not p4 and p5) + (not p5 and p6) +
                        (not p6 and p7) + (not p7 and p8) + (not p8 and p9) + (not p9 and p2))
                if a != 1:
                    continue
                if step == 0 and (p2 and p4 and p6 or p4 and p6 and p8):
                    continue
                if step == 1 and (p2 and p4 and p8 or p2 and p6 and p8):
                    continue
                remove.append((i, j))
        for i, j in remove:
            img[i, j] = 0
        changed = changed or bool(remove)

skel = img > 0
print("skeleton px %d (%.2f m)" % (skel.sum(), skel.sum() * RES))

samples = []
for row, col in zip(*np.nonzero(skel)):
    x, y = col * RES + ORIGIN[0], row * RES + ORIGIN[1]
    if samples and math.hypot(x - samples[-1][0], y - samples[-1][1]) < STEP:
        continue
    if node_ok(x, y):
        samples.append((x, y))
print("nodes %d" % len(samples))

nodes = {i: {"pose": p, "to": set()} for i, p in enumerate(samples)}
for i in range(len(samples)):
    for j in range(i + 1, len(samples)):
        d = math.dist(samples[i], samples[j])
        if d > 0.40:
            continue
        yaw = math.atan2(samples[j][1] - samples[i][1], samples[j][0] - samples[i][0])
        n = max(1, int(math.ceil(d / 0.02)))
        if all(clear_pose(samples[i][0] + (samples[j][0] - samples[i][0]) * t / n,
                          samples[i][1] + (samples[j][1] - samples[i][1]) * t / n, yaw, blocked)
               for t in range(n + 1)):
            nodes[i]["to"].add(j)
            nodes[j]["to"].add(i)
print("links %d" % (sum(len(v["to"]) for v in nodes.values()) // 2))

# Keep the graph light: every node keeps only its three nearest neighbours (plus
# whoever kept it), otherwise planning has to sweep hundreds of candidate edges.
KEEP_NEAREST = 3
for i, node in nodes.items():
    ranked = sorted(node["to"], key=lambda j: math.dist(node["pose"], nodes[j]["pose"]))
    for j in ranked[KEEP_NEAREST:]:
        node["to"].discard(j)
print("links after pruning %d" % (sum(len(v["to"]) for v in nodes.values()) // 2))

seen, best = set(), set()
for i in nodes:
    if i in seen:
        continue
    stack, comp = [i], set()
    while stack:
        n = stack.pop()
        if n in comp:
            continue
        comp.add(n)
        stack.extend(t for t in nodes[n]["to"] if t not in comp)
    seen |= comp
    if len(comp) > len(best):
        best = comp
alive = sorted(best)
remap = {old: new for new, old in enumerate(alive)}
print("largest component %d nodes, %d edges"
      % (len(alive), sum(len(nodes[o]["to"]) for o in alive) // 2))

lines = ["waypoints:"]
edge_widths = {}
for old in alive:
    new = remap[old]
    x, y = nodes[old]["pose"]
    tos = sorted(remap[t] for t in nodes[old]["to"] if t in remap)
    lines += ["  - id: %d" % new, "    pose: [%.4f, %.4f]" % (x, y),
              "    to: %s" % json.dumps(tos), "    name: 路口%d" % new]
    for t in tos:
        edge_widths["%d:%d" % (new, t)] = 0.5

os.makedirs(OUT, exist_ok=True)
# PGM row 0 is the top row (largest y); our arrays are bottom-row-first.
thirds = np.where(occ_clean, 0, 255).astype("uint8")
Image.fromarray(np.flipud(thirds)).save(OUT + "/map.pgm")
Image.fromarray(np.flipud(np.where(occ_clean, 0, 255) if False else np.where(band, 255, 0)).astype("uint8")).save(OUT + "/keepout.pgm")
Image.fromarray(np.flipud(np.where(band, 255, 0)).astype("uint8")).save(OUT + "/roads.pgm")
yaml_text = ("image: {name}.pgm\nmode: trinary\nresolution: 0.05\n"
             "origin: [-5.04, -7.53, 0]\nnegate: 0\n"
             "occupied_thresh: 0.65\nfree_thresh: 0.25\n")
for name in ("map", "roads", "keepout"):
    open(OUT + "/" + name + ".yaml", "w", encoding="utf-8", newline="\n").write(yaml_text.format(name=name))
open(OUT + "/road_network.yaml", "w", encoding="utf-8", newline="\n").write("\n".join(lines) + "\n")

meta["id"] = "sandbox_nav_band_20261101"
meta["edge_widths"] = edge_widths
files = ["map.pgm", "map.yaml", "roads.pgm", "roads.yaml", "keepout.pgm", "keepout.yaml", "road_network.yaml"]
meta["sha256"] = {n: hashlib.sha256(open(OUT + "/" + n, "rb").read()).hexdigest() for n in files}
json.dump(meta, open(OUT + "/sandbox.json", "w", encoding="utf-8", newline="\n"), ensure_ascii=False, indent=2)
print("wrote", OUT)
