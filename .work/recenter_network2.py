"""Nearest-corridor re-centring on the live costmap.

Same inputs as recentre_network.py, but scores a candidate waypoint by the
footprint test itself (pose_clear along each incident edge, padded by half a
cell exactly like the car runtime) instead of raw clearance. Waypoints that
cannot be placed on a passing spot keep their best candidate so the caller can
see exactly which edges still block.
"""
import json
import math

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage as ndi

RES = 0.05
ORIGIN = (-5.04, -7.53)
FOOT = (0.25, 0.15, 0.115)
HALF_CELL = RES / 2

a = np.array(Image.open(r".work\costmap_dump.pgm").convert("L"))
blocked = a < 200
unknown = a == 128
free = (~blocked) & (~unknown)
height, width = a.shape
d = ndi.distance_transform_edt(free) * RES

nodes = json.load(open(r".work\net_nodes.json", encoding="utf-8"))
pos = {int(i): tuple(p) for i, p, _ in nodes}
links = {int(i): [int(t) for t in to] for i, _, to in nodes}
edges = sorted({(min(x, y), max(x, y)) for x, ts in links.items() for y in ts})


def inside(row, col):
    return 0 <= row < height and 0 <= col < width


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
    if not (inside(r0, c0) and inside(r1, c1)):
        return False
    for r in range(r0, r1 + 1):
        for c in range(c0, c1 + 1):
            if not blocked[r, c]:
                continue
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


def score(node, cand):
    total = 0.0
    for other, _ in [e for e in edges if node in e]:
        mate = other if other != node else _
        p, q = (cand, pos[mate]) if cand == pos[node] or other == node else (pos[mate], cand)
        yaw = math.atan2(q[1] - p[1], q[0] - p[0])
        distance = math.dist(p, q)
        n = max(1, int(math.ceil(distance / 0.04)))
        for i in range(n + 1):
            if not pose_clear(p[0] + (q[0] - p[0]) * i / n, p[1] + (q[1] - p[1]) * i / n, yaw):
                return -1.0
        total += 1.0
    return total


for round_ in range(6):
    for node in pos:
        neighbours = [b if a_ == node else a_ for a_, b in edges if node in (a_, b)]
        best, choice = -1.0, pos[node]
        for dx in np.arange(-0.10, 0.101, 0.02):
            for dy in np.arange(-0.10, 0.101, 0.02):
                cand = (pos[node][0] + dx, pos[node][1] + dy)
                row = int(round((cand[1] - ORIGIN[1]) / RES))
                col = int(round((cand[0] - ORIGIN[0]) / RES))
                if not inside(row, col) or not free[row, col]:
                    continue
                ok_all = True
                for mate in neighbours:
                    p, q = (cand, pos[mate])
                    if not edge_ok(p, q):
                        ok_all = False
                        break
                if ok_all:
                    best, choice = 9.0, cand
                    break
            if best > 0:
                break
        pos[node] = choice

bad = [e for e in edges if not edge_ok(pos[e[0]], pos[e[1]])]
print("edges %d blocked %d" % (len(edges), len(bad)))
print("blocked", ["%d-%d" % e for e in bad])

result = [{"id": i, "pose": [round(pos[i][0], 4), round(pos[i][1], 4)], "to": sorted(links[i])} for i in sorted(pos)]
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
