"""Template-network project with a physically wide keepout.

Uses the existing sandbox_lidar_20261031 registration (plan 2.97x5.40 -> map),
keeps its nine-junction road network, but rebuilds keepout as "plate rectangle
minus lidar obstacles" instead of the plan white pixels. That restores the real
0.50 m corridor width so costmap inflation no longer eats the whole road.
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
OUT = r"app/savedMap/sandbox_lidar_wide_20261101"

meta = json.load(open(SRC + "/sandbox.json", encoding="utf-8"))
tr = meta["calibration"]["transform"]
c, s = math.cos(tr["yaw"]), math.sin(tr["yaw"])


def fwd(x, y):
    return (c * x - s * y + tr["x"], s * x + c * y + tr["y"])


corners = [fwd(0, 0), fwd(2.97, 0), fwd(2.97, 5.40), fwd(0, 5.40)]

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
print("keepout %.2f m2" % (keep.sum() * 0.0025))

d_out = ndi.distance_transform_edt(keep) * RES

import yaml

net = yaml.safe_load(open(SRC + "/road_network.yaml", encoding="utf-8"))["waypoints"]
pos = {x["id"]: tuple(x["pose"][:2]) for x in net}
links = {x["id"]: x["to"] for x in net}

for a_, b in sorted({(a_, b) for a_, ts in links.items() for b in ts}):
    pa, pb = pos[a_], pos[b]
    n = max(1, int(math.ceil(math.dist(pa, pb) / 0.03)))
    hit = all(
        keep[int(round((pa[1] + (pb[1] - pa[1]) * i / n - ORIGIN[1]) / RES)),
             int(round((pa[0] + (pb[0] - pa[0]) * i / n - ORIGIN[0]) / RES))]
        for i in range(n + 1))
    if not hit:
        print("  edge %d-%d leaves keepout" % (a_, b))

edge_widths = {}
for a_, b in sorted({(a_, b) for a_, ts in links.items() for b in ts} | {(b, a_) for a_, ts in links.items() for b in ts}):
    pa, pb = pos[a_], pos[b]
    n = max(1, int(math.ceil(math.dist(pa, pb) / 0.05)))
    clear = 1e9
    for i in range(n + 1):
        x = pa[0] + (pb[0] - pa[0]) * i / n
        y = pa[1] + (pb[1] - pa[1]) * i / n
        row = int(round((y - ORIGIN[1]) / RES))
        col = int(round((x - ORIGIN[0]) / RES))
        if not keep[row, col]:
            clear = 0.0
            break
        clear = min(clear, d_out[row, col])
    width = round(2 * clear, 3) if clear < 1e8 else 0.0
    edge_widths["%d:%d" % (a_, b)] = width
print("edge widths", {k: v for k, v in sorted(edge_widths.items()) if k.split(":")[0] < k.split(":")[1]})

os.makedirs(OUT, exist_ok=True)
map_pgm.save(OUT + "/map.pgm")
mask = Image.fromarray(np.where(keep, 255, 0).astype("uint8"))
mask.save(OUT + "/keepout.pgm")
Image.open(SRC + "/roads.pgm").save(OUT + "/roads.pgm")
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
open(OUT + "/road_network.yaml", "w", encoding="utf-8", newline="\n").write(
    open(SRC + "/road_network.yaml", encoding="utf-8").read())

meta["id"] = "sandbox_lidar_wide_20261101"
meta["edge_widths"] = edge_widths
files = ["map.pgm", "map.yaml", "roads.pgm", "roads.yaml", "keepout.pgm", "keepout.yaml", "road_network.yaml"]
meta["sha256"] = {n: hashlib.sha256(open(OUT + "/" + n, "rb").read()).hexdigest() for n in files}
json.dump(meta, open(OUT + "/sandbox.json", "w", encoding="utf-8", newline="\n"), ensure_ascii=False, indent=2)
print("wrote", OUT)
