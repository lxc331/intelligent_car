"""Template (nine-junction) network on a physically wide keepout, pruned to
edges that would pass the car's runtime costmap swept check.

keepout = plate rectangle minus lidar obstacles.
runtime simulation: 253 = within 0.115 m (+half raster cell) of lethal
(mask border or obstacle), tested with the real 0.25/0.15/0.115 footprint.
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
SRC = r"app/savedMap/sandbox_lidar_20261031"
OUT = r"app/savedMap/sandbox_nav_tpl_20261101"

meta = json.load(open(SRC + "/sandbox.json", encoding="utf-8"))
tr = meta["calibration"]["transform"]
c, s = math.cos(tr["yaw"]), math.sin(tr["yaw"])
corners = [(c * x - s * y + tr["x"], s * x + c * y + tr["y"])
           for x, y in ((0, 0), (2.97, 0), (2.97, 5.40), (0, 5.40))]

map_pgm = Image.open(SRC + "/map.pgm").convert("L")
w, h = map_pgm.size
occ = np.array(map_pgm) <= 60
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
blocked = (d_out < 0.115 + 0.025) | (d_occ < 0.115 + 0.025)

net = yaml.safe_load(open(SRC + "/road_network.yaml", encoding="utf-8"))["waypoints"]
pos = {x["id"]: tuple(x["pose"][:2]) for x in net}
links = {x["id"]: x["to"] for x in net}


def clear_pose(x, y, yaw):
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
        for col in range(c0, c1 + 1):
            if blocked[r, col]:
                dx = (col * RES + RES / 2 + ORIGIN[0]) - cx
                dy = (r * RES + RES / 2 + ORIGIN[1]) - cy
                cc, ss = math.cos(yaw), math.sin(yaw)
                if abs(cc * dx + ss * dy) <= a_ + RES / 2 and abs(-ss * dx + cc * dy) <= half + RES / 2:
                    return False
    return True


def edge_pass(a_, b):
    pa, pb = pos[a_], pos[b]
    yaw = math.atan2(pb[1] - pa[1], pb[0] - pa[0])
    n = max(1, int(math.ceil(math.dist(pa, pb) / 0.02)))
    return all(clear_pose(pa[0] + (pb[0] - pa[0]) * i / n, pa[1] + (pb[1] - pa[1]) * i / n, yaw) for i in range(n + 1))


keep_edges = {}
for a_, ts in links.items():
    for b in ts:
        keep_edges[(a_, b)] = edge_pass(a_, b)

print("directed edges pass %d / %d" % (sum(keep_edges.values()), len(keep_edges)))
for (a_, b), ok in sorted(keep_edges.items()):
    print("  %d->%d %s" % (a_, b, "ok" if ok else "BLOCKED"))

alive = {i for (a_, b), ok in keep_edges.items() if ok for i in (a_, b)}
new_links = {i: [b for b in links[i] if keep_edges.get((i, b))] for i in alive}

os.makedirs(OUT, exist_ok=True)
map_pgm.save(OUT + "/map.pgm")
mask_img = Image.fromarray(np.where(keep, 255, 0).astype("uint8"))
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

lines = ["waypoints:"]
for i in sorted(alive):
    lines.append("  - id: %d" % i)
    lines.append("    pose: [%.4f, %.4f]" % pos[i])
    lines.append("    to: %s" % json.dumps(sorted(new_links[i])))
    lines.append("    name: 路口%d" % i)
open(OUT + "/road_network.yaml", "w", encoding="utf-8", newline="\n").write("\n".join(lines) + "\n")

widths = {k: 0.5 for k, ok in keep_edges.items() if ok}
meta["id"] = "sandbox_nav_tpl_20261101"
meta["edge_widths"] = widths
files = ["map.pgm", "map.yaml", "roads.pgm", "roads.yaml", "keepout.pgm", "keepout.yaml", "road_network.yaml"]
meta["sha256"] = {n: hashlib.sha256(open(OUT + "/" + n, "rb").read()).hexdigest() for n in files}
json.dump(meta, open(OUT + "/sandbox.json", "w", encoding="utf-8", newline="\n"), ensure_ascii=False, indent=2)
print("wrote", OUT, "nodes", len(alive))
