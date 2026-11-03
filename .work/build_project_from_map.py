# -*- coding: utf-8 -*-
"""Build a sandbox project straight from the lidar map, without the 3x3 template.

    python build_project_from_map.py

Roads and keepout live in the localization (lidar map) frame, so registration is
the identity; the junction graph is the skeleton of the drivable surface, sampled
about every 0.25 m, with each waypoint's corridor width measured from the
distance transform.
"""
import hashlib
import json
import math
import shutil
import sys
import uuid
from pathlib import Path

import numpy as np
import scipy.ndimage as ndi
import yaml
from PIL import Image

MAP_DIR = Path("app/savedMap/sandbox_20261031_2025")
OUT = Path("app/savedMap/sandbox_nav_20261031")
CLEARANCE = 0.07          # keep waypoints this far from any hit
DISTRICT_CLOSE = 0.13     # model zones: bridge small gaps ...
DISTRICT_GROW = 0.10      # ... then grow, so a path cannot sneak between pieces
SPACING = 0.25            # waypoints along the skeleton
MIN_WIDTH = 0.24          # a corridor narrower than this is not a road

MAP_YAML = yaml.safe_load((MAP_DIR / "map.yaml").read_text(encoding="utf-8"))
IMAGE = np.array(Image.open(MAP_DIR / MAP_YAML["image"]).convert("L"))
RES = float(MAP_YAML["resolution"])
OX, OY = float(MAP_YAML["origin"][0]), float(MAP_YAML["origin"][1])
H, W = IMAGE.shape
free = IMAGE >= 250
occ = IMAGE <= 60
d_occ = ndi.distance_transform_edt(~occ) * RES


def to_xy(col, row):
    return col * RES + OX, (H - 1 - row) * RES + OY


def to_grid(x, y):
    return (x - OX) / RES, H - 1 - (y - OY) / RES


def drivable():
    closed = ndi.binary_closing(occ, structure=np.ones((3, 3), bool),
                                iterations=max(1, int(round(DISTRICT_CLOSE / RES / 2))))
    grown = ndi.binary_dilation(closed, iterations=max(1, int(round(DISTRICT_GROW / RES))))
    plate = ndi.binary_fill_holes(ndi.binary_dilation(free, iterations=2))
    mask = free & plate & ~grown & (d_occ >= CLEARANCE)
    mask = ndi.binary_opening(mask, structure=np.ones((2, 2), bool))
    return mask


def thin(mask):
    """Zhang-Suen thinning."""
    img = mask.copy().astype(np.uint8)
    changed = True
    while changed:
        changed = False
        for step in (0, 1):
            p = np.pad(img, 1)
            p2, p3 = p[:-2, 1:-1], p[:-2, 2:]
            p4, p5 = p[1:-1, 2:], p[2:, 2:]
            p6, p7 = p[2:, 1:-1], p[2:, :-2]
            p8, p9 = p[1:-1, :-2], p[:-2, :-2]
            b = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9
            a = ((p2 == 0) & (p3 == 1)).astype(np.uint8) + ((p3 == 0) & (p4 == 1)) \
                + ((p4 == 0) & (p5 == 1)) + ((p5 == 0) & (p6 == 1)) \
                + ((p6 == 0) & (p7 == 1)) + ((p7 == 0) & (p8 == 1)) \
                + ((p8 == 0) & (p9 == 1)) + ((p9 == 0) & (p2 == 1))
            if step == 0:
                cond = (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)
            else:
                cond = (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
            remove = (img == 1) & (b >= 2) & (b <= 6) & (a == 1) & cond
            if remove.any():
                img[remove] = 0
                changed = True
    return img.astype(bool)


def neighbors(skel):
    kern = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]])
    return ndi.convolve(skel.astype(np.uint8), kern, mode="constant")


def skeleton_graph(mask):
    skel = thin(mask)
    deg = neighbors(skel)
    nodes = skel & ((deg >= 3) | (deg == 1))
    lab, n = ndi.label(nodes, structure=np.ones((3, 3), bool))
    centres = ndi.center_of_mass(nodes, lab, range(1, n + 1))
    pts = []
    for row, col in centres:
        x, y = to_xy(col, row)
        pts.append((x, y))
    # trace skeleton paths between junction pixels
    ys, xs = np.nonzero(skel)
    cells = {(r, c): i for i, (r, c) in enumerate(zip(ys, xs))}
    adj = {k: [] for k in cells}
    for (r, c) in cells:
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == dc == 0:
                    continue
                other = (r + dr, c + dc)
                if other in cells:
                    adj[(r, c)].append(other)
    node_cell = {}
    for (r, c) in cells:
        if lab[r, c]:
            node_cell.setdefault(int(lab[r, c]) - 1, (r, c))
    edges = {}
    for start_node, start_cell in node_cell.items():
        for first in adj[start_cell]:
            path = [start_cell, first]
            prev, cur = start_cell, first
            while not lab[cur]:
                nxt = [q for q in adj[cur] if q != prev]
                if len(nxt) != 1:
                    break
                prev, cur = cur, nxt[0]
                path.append(cur)
            if not lab[cur]:
                continue
            end_node = int(lab[cur]) - 1
            if end_node == start_node and len(path) < 4:
                continue
            key = tuple(sorted((start_node, end_node)))
            length = sum(math.dist(to_xy(*a), to_xy(*b)) for a, b in zip(path, path[1:]))
            if key not in edges or edges[key][0] > length:
                edges[key] = (length, path)
    return pts, edges


def sample_waypoints(points, edges, drivable_mask):
    """Split each skeleton edge into waypoints ~SPACING apart (kdtree-free greedy)."""
    waypoints = []      # (x, y)
    links = []
    index_of = {}

    def waypoint_at(x, y):
        best = None
        for i, (wx, wy) in enumerate(waypoints):
            d = math.dist((x, y), (wx, wy))
            if d < SPACING * 0.6 and (best is None or d < best[1]):
                best = (i, d)
        if best is not None:
            return best[0]
        waypoints.append((x, y))
        return len(waypoints) - 1

    for (a, b), (length, path) in edges.items():
        px = [to_xy(*cell) for cell in path]
        acc = 0.0
        seq = [waypoint_at(*points[a])]
        for i in range(1, len(px)):
            acc += math.dist(px[i - 1], px[i])
            if acc >= SPACING or i == len(px) - 1:
                seq.append(waypoint_at(*px[i]))
                acc = 0.0
        for u, v in zip(seq, seq[1:]):
            if u != v:
                links.append((u, v))
    return waypoints, links


def corridor_width(x, y, drivable_mask):
    gx, gy = to_grid(x, y)
    gx, gy = int(round(gx)), int(round(gy))
    if not (0 <= gx < W and 0 <= gy < H):
        return 0.0
    if not drivable_mask[gy, gx]:
        return 0.0
    d = ndi.distance_transform_edt(drivable_mask)[gy, gx] * RES
    return 2 * d


def build():
    if OUT.exists():
        print("removing old", OUT)
        shutil.rmtree(OUT)
    drv = drivable()
    print("drivable cells %.1f%%" % (100.0 * drv.mean()))
    pts, edges = skeleton_graph(drv)
    print("skeleton nodes %d, edges %d" % (len(pts), len(edges)))
    waypoints, links = sample_waypoints(pts, edges, drv)
    widths = []
    for x, y in waypoints:
        widths.append(corridor_width(x, y, drv))
    print("waypoints %d, widths %.2f..%.2f m" % (len(waypoints), min(widths), max(widths)))

    OUT.mkdir(parents=True)
    # map.* identical to the localization map
    shutil.copy(MAP_DIR / MAP_YAML["image"], OUT / "map.pgm")
    (OUT / "map.yaml").write_text((MAP_DIR / "map.yaml").read_text(encoding="utf-8"),
                                  encoding="utf-8")
    # roads / keepout: drivable white, everything else black, same frame as map
    mask_img = np.where(drv, 255, 0).astype(np.uint8)
    pgm = b"P5\n%d %d\n255\n" % (W, H) + mask_img.tobytes()
    for name in ("roads", "keepout"):
        (OUT / (name + ".pgm")).write_bytes(pgm)
        text = (MAP_DIR / "map.yaml").read_text(encoding="utf-8")
        text = text.replace("image: " + MAP_YAML["image"], "image: " + name + ".pgm")
        (OUT / (name + ".yaml")).write_text(text, encoding="utf-8")

    # network
    to = {i: set() for i in range(len(waypoints))}
    for u, v in links:
        to[u].add(v)
        to[v].add(u)
    lines = ["waypoints:"]
    for i, (x, y) in enumerate(waypoints):
        lines.append("  - id: %d" % i)
        lines.append("    pose: [%.4f, %.4f]" % (x, y))
        lines.append("    to: [%s]" % ", ".join(str(t) for t in sorted(to[i])))
        lines.append("    name: 路口%d" % i)
    (OUT / "road_network.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")

    edge_widths = {}
    for u, v in links:
        width = min(widths[u], widths[v])
        edge_widths["%d:%d" % (u, v)] = round(width, 3)
        edge_widths["%d:%d" % (v, u)] = round(width, 3)

    pairs = [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]
    meta = dict(version=1, id=uuid.uuid4().hex, frame="map", calibrated=True, max_rms=0.075,
                calibration=dict(source=pairs, target=pairs,
                                 transform=dict(x=0.0, y=0.0, yaw=0.0, rms=0.0,
                                                maximum=0.0, scale_ratio=1.0)),
                edge_widths=edge_widths, narrow_enabled=True, narrow_nodes=[],
                sources=dict(localization=str(MAP_DIR), roads="skeleton of the lidar map"),
                note="graph derived from the 2026-10-31 lidar map; template replaced")
    sha = {}
    for path in OUT.iterdir():
        sha[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    meta["sha256"] = sha
    (OUT / "sandbox.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2),
                                      encoding="utf-8")
    print("wrote", OUT)

    sys.path.insert(0, str(Path("app").resolve()))
    from carapp.core.sandbox_project import load_project
    from carapp.core.sandbox_route import plan_route
    project = load_project(OUT / "sandbox.json")
    print("project loads: nodes %d" % len(list(project.network)))
    ok = fail = 0
    lengths = []
    ids = sorted(n for n in project.network.nodes)
    for a in ids[:1]:
        for b in ids[1:]:
            try:
                route = plan_route(project, (waypoints[a][0], waypoints[a][1], 0.0), b)
                ok += 1
                lengths.append(route.length)
            except ValueError:
                fail += 1
    print("routes from node %d: %d ok, %d failed" % (ids[0], ok, fail))
    if lengths:
        print("route lengths %.2f .. %.2f m" % (min(lengths), max(lengths)))


if __name__ == "__main__":
    build()
