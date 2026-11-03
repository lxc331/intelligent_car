"""Re-centre the hand-drawn road network on the live costmap corridors.

Reads .work/costmap_dump.pgm + .json + .work/net_nodes.json, moves every
waypoint to the place that maximises clearance along its edges, verifies the
result with the same rectangle/footprint test the car runtime uses, and writes
.work/net_nodes_recentred.json plus a diagnostic PNG.
"""
import json
import math

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage as ndi

RES = 0.05
ORIGIN = (-5.04, -7.53)
FOOT = (0.25, 0.15, 0.115)   # front, rear, half (car + claw)
MARGIN = 0.012               # extra clearance so centimetre errors do not bite

a = np.array(Image.open(r".work\costmap_dump.pgm").convert("L"))
blocked = a < 200            # 253/254 lethal + inscribed
unknown = a == 128
free = ~blocked & ~unknown
height, width = a.shape
d = ndi.distance_transform_edt(free) * RES

nodes = json.load(open(r".work\net_nodes.json", encoding="utf-8"))
pos = {int(i): tuple(p) for i, p, _ in nodes}
links = {int(i): [int(t) for t in to] for i, _, to in nodes}
edges = sorted({(min(a_, b), max(a_, b)) for a_, ts in links.items() for b in ts})


def cell(x, y):
    col = int(round((x - ORIGIN[0]) / RES))
    row = int(round((y - ORIGIN[1]) / RES))
    return row, col


def clearance(x, y):
    row, col = cell(x, y)
    if not (0 <= row < height and 0 <= col < width):
        return -1.0
    return float(d[row, col])


def line_clearance(p, q, step=0.02):
    distance = math.dist(p, q)
    count = max(1, int(math.ceil(distance / step)))
    best = 1e9
    for i in range(count + 1):
        t = i / count
        best = min(best, clearance(p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t))
    return best


def pose_clear(x, y, yaw):
    """Rectangle-vs-raster test identical in spirit to path_rules.pose_clear."""
    front, rear, half = FOOT
    a_ = (front + rear) / 2
    cx, cy = x + (front - rear) / 2 * math.cos(yaw), y + (front - rear) / 2 * math.sin(yaw)
    ex = abs(math.cos(yaw)) * a_ + abs(math.sin(yaw)) * half
    ey = abs(math.sin(yaw)) * a_ + abs(math.cos(yaw)) * half
    row0, col0 = cell(cx - ex, cy - ey)
    row1, col1 = cell(cx + ex, cy + ey)
    for row in range(min(row0, row1), max(row0, row1) + 1):
        for col in range(min(col0, col1), max(col0, col1) + 1):
            if not (0 <= row < height and 0 <= col < width):
                return False
            if blocked[row, col]:
                dx = (col * RES + RES / 2 + ORIGIN[0]) - cx
                dy = (row * RES + RES / 2 + ORIGIN[1]) - cy
                c, s = math.cos(yaw), math.sin(yaw)
                if abs(c * dx + s * dy) <= a_ + RES * 0.5 and abs(-s * dx + c * dy) <= half + RES * 0.5:
                    return False
    return True


def edge_ok(p, q):
    yaw = math.atan2(q[1] - p[1], q[0] - p[0])
    distance = math.dist(p, q)
    count = max(1, int(math.ceil(distance / 0.02)))
    for i in range(count + 1):
        t = i / count
        if not pose_clear(p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t, yaw):
            return False
    return True


for _ in range(8):
    for node in pos:
        neighbours = [b if a_ == node else a_ for a_, b in edges if node in (a_, b)]
        best, choice = -9, pos[node]
        for dx in np.arange(-0.12, 0.121, 0.02):
            for dy in np.arange(-0.12, 0.121, 0.02):
                cand = (pos[node][0] + dx, pos[node][1] + dy)
                score = min(line_clearance(cand, pos[n]) for n in neighbours)
                if score > best:
                    best, choice = score, cand
        pos[node] = choice

bad = [(a_, b) for a_, b in edges if not edge_ok(pos[a_], pos[b])]
print("recentred nodes, min clearance %.3f" % min(clearance(*p) for p in pos.values()))
print("edges %d blocked %d" % (len(edges), len(bad)))
print("blocked", ["%d-%d" % e for e in bad])

result = [{"id": i, "pose": [round(pos[i][0], 4), round(pos[i][1], 4)],
           "to": sorted(links[i]),
           "width": round(2 * min(line_clearance(pos[i], pos[n]) for n in links[i]), 3)} for i in sorted(pos)]
json.dump(result, open(r".work\net_nodes_recentred.json", "w", encoding="utf-8"), ensure_ascii=False, indent=1)

image = Image.fromarray(np.where(blocked, 0, 255).astype("uint8")).convert("RGB").resize((width * 4, height * 4), Image.NEAREST)
draw = ImageDraw.Draw(image)


def pixel(p):
    return ((p[0] - ORIGIN[0]) / RES * 4, (height - (p[1] - ORIGIN[1]) / RES) * 4)


for a_, b in edges:
    draw.line([pixel(pos[a_]), pixel(pos[b])], fill=(230, 0, 0) if (a_, b) in bad else (0, 120, 255), width=2)
for p in pos.values():
    x, y = pixel(p)
    draw.ellipse([x - 3, y - 3, x + 3, y + 3], fill=(255, 150, 0))
image.save(r".work\maps\network_recentred.png")
print("wrote .work/maps/network_recentred.png")
