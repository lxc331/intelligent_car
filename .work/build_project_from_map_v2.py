# -*- coding: utf-8 -*-
"""Second attempt: restrict the surface to the sandbox plate, prune the skeleton,
keep only edges wide enough for the 0.23 m body, then build the project."""
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
CLEARANCE = 0.10
DISTRICT_CLOSE = 0.13
DISTRICT_GROW = 0.10
SPACING = 0.25
MIN_EDGE = 0.23
SPUR_ITERS = 6
BODY_HALF = 0.115

# plate frame measured by redraw_plan_from_lidar_v2.py
PLATE_CENTRE = (1.297, -0.491)
PLATE_THETA = math.radians(46.97)
PLATE_LONG = 5.400
PLATE_SHORT = 2.960

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
    return int(round((x - OX) / RES)), int(round(H - 1 - (y - OY) / RES))


def plate_mask():
    yy, xx = np.mgrid[0:H, 0:W]
    x = xx * RES + OX
    y = (H - 1 - yy) * RES + OY
    c, s = math.cos(-PLATE_THETA), math.sin(-PLATE_THETA)
    u = c * (x - PLATE_CENTRE[0]) - s * (y - PLATE_CENTRE[1])
    v = s * (x - PLATE_CENTRE[0]) + c * (y - PLATE_CENTRE[1])
    return (np.abs(u) <= PLATE_LONG / 2) & (np.abs(v) <= PLATE_SHORT / 2)


def drivable():
    closed = ndi.binary_closing(occ, structure=np.ones((3, 3), bool),
                                iterations=max(1, int(round(DISTRICT_CLOSE / RES / 2))))
    grown = ndi.binary_dilation(closed, iterations=max(1, int(round(DISTRICT_GROW / RES))))
    mask = free & plate_mask() & ~grown & (d_occ >= CLEARANCE)
    return ndi.binary_opening(mask, structure=np.ones((2, 2), bool))


def thin(mask):
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
            a = (((p2 == 0) & (p3 == 1)) + ((p3 == 0) & (p4 == 1))
                 + ((p4 == 0) & (p5 == 1)) + ((p5 == 0) & (p6 == 1))
                 + ((p6 == 0) & (p7 == 1)) + ((p7 == 0) & (p8 == 1))
                 + ((p8 == 0) & (p9 == 1)) + ((p9 == 0) & (p2 == 1)))
            if step == 0:
                cond = (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)
            else:
                cond = (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
            remove = (img == 1) & (b >= 2) & (b <= 6) & (a == 1) & cond
            if remove.any():
                img[remove] = 0
                changed = True
    return img.astype(bool)


def deg_map(skel):
    kern = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]])
    return ndi.convolve(skel.astype(np.uint8), kern, mode="constant")


def prune(skel, iters):
    for _ in range(iters):
        d = deg_map(skel)
        ends = skel & (d == 1)
        if not ends.any():
            break
        skel = skel & ~ends
    return skel


def biggest_component(skel):
    lab, n = ndi.label(skel, structure=np.ones((3, 3), bool))
    if n == 0:
        return skel
    sizes = ndi.sum(skel, lab, range(1, n + 1))
    good = int(np.argmax(sizes)) + 1
    return lab == good


def skeleton_graph(skel):
    deg = deg_map(skel)
    node_px = skel & ((deg >= 3) | (deg == 1))
    lab, n = ndi.label(node_px, structure=np.ones((3, 3), bool))
    centres = ndi.center_of_mass(node_px, lab, range(1, n + 1))
    pts = [to_xy(col, row) for row, col in centres]
    cells = {(int(r), int(c)) for r, c in zip(*np.nonzero(skel))}
    adj = {}
    for (r, c) in cells:
        nbrs = []
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == dc == 0:
                    continue
                if (r + dr, c + dc) in cells:
                    nbrs.append((r + dr, c + dc))
        adj[(r, c)] = nbrs
    node_cell = {}
    for (r, c) in cells:
        if lab[r, c]:
            node_cell.setdefault(int(lab[r, c]) - 1, (r, c))
    edges = {}
    for a, start in node_cell.items():
        for first in adj[start]:
            path = [start, first]
            prev, cur = start, first
            while not lab[cur]:
                nxt = [q for q in adj[cur] if q != prev]
                if len(nxt) != 1:
                    break
                prev, cur = cur, nxt[0]
                path.append(cur)
            if not lab[cur]:
                continue
            b = int(lab[cur]) - 1
            if a == b and len(path) < 4:
                continue
            key = tuple(sorted((a, b)))
            length = sum(math.dist(to_xy(*p), to_xy(*q)) for p, q in zip(path, path[1:]))
            if key not in edges or edges[key][0] > length:
                edges[key] = (length, path)
    return pts, edges


def width_at(x, y, dt):
    gx, gy = to_grid(x, y)
    if not (0 <= gx < W and 0 <= gy < H):
        return 0.0
    return 2 * dt[gy, gx] * RES


def build():
    if OUT.exists():
        shutil.rmtree(OUT)
    drv = drivable()
    dt = ndi.distance_transform_edt(drv) * RES
    print("drivable %.1f%%" % (100.0 * drv.mean()))
    skel = biggest_component(prune(thin(drv), SPUR_ITERS))
    print("skeleton px %d" % int(skel.sum()))
    pts, edges = skeleton_graph(skel)
    print("junctions %d, raw edges %d" % (len(pts), len(edges)))

    waypoints = []
    links = []

    def wid(x, y):
        for i, (wx, wy) in enumerate(waypoints):
            if math.dist((x, y), (wx, wy)) < SPACING * 0.6:
                return i
        waypoints.append((x, y))
        return len(waypoints) - 1

    for (a, b), (length, path) in edges.items():
        px = [to_xy(*cell) for cell in path]
        widths = [width_at(x, y, dt) for x, y in px]
        seq, acc = [wid(*pts[a])], 0.0
        for i in range(1, len(px)):
            acc += math.dist(px[i - 1], px[i])
            if acc >= SPACING or i == len(px) - 1:
                seq.append(wid(*px[i]))
                acc = 0.0
        for u, v in zip(seq, seq[1:]):
            if u != v:
                links.append((u, v, min(widths)))
    # drop edges too narrow, then keep the largest connected part
    kept = [(u, v, w) for (u, v, w) in links if w >= MIN_EDGE]
    print("links %d, wide links %d" % (len(links), len(kept)))
    adj = {}
    for u, v, _ in kept:
        adj.setdefault(u, set()).add(v)
        adj.setdefault(v, set()).add(u)
    seen, comps = set(), []
    for start in adj:
        if start in seen:
            continue
        stack, comp = [start], set()
        while stack:
            cur = stack.pop()
            if cur in comp:
                continue
            comp.add(cur)
            seen.add(cur)
            stack.extend(adj[cur] - comp)
        comps.append(comp)
    comps.sort(key=len, reverse=True)
    main = comps[0]
    print("components %s" % [len(c) for c in comps[:6]])
    used = sorted(main)
    remap = {old: i for i, old in enumerate(used)}
    wps = [waypoints[i] for i in used]
    kept = [(remap[u], remap[v], w) for (u, v, w) in kept if u in main and v in main]

    OUT.mkdir(parents=True)
    shutil.copy(MAP_DIR / MAP_YAML["image"], OUT / "map.pgm")
    map_text = (MAP_DIR / "map.yaml").read_text(encoding="utf-8")
    (OUT / "map.yaml").write_text(map_text, encoding="utf-8")
    mask_img = np.where(drv, 255, 0).astype(np.uint8)
    pgm = b"P5\n%d %d\n255\n" % (W, H) + mask_img.tobytes()
    for name in ("roads", "keepout"):
        (OUT / (name + ".pgm")).write_bytes(pgm)
        (OUT / (name + ".yaml")).write_text(
            map_text.replace("image: " + MAP_YAML["image"], "image: " + name + ".pgm"),
            encoding="utf-8")

    to = {i: set() for i in range(len(wps))}
    for u, v, _ in kept:
        to[u].add(v)
        to[v].add(u)
    lines = ["waypoints:"]
    for i, (x, y) in enumerate(wps):
        lines += ["  - id: %d" % i, "    pose: [%.4f, %.4f]" % (x, y),
                  "    to: [%s]" % ", ".join(str(t) for t in sorted(to[i])),
                  "    name: 路口%d" % i]
    (OUT / "road_network.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")

    widths = {i: min([w for u, v, w in kept if u == i or v == i] or [0.0])
              for i in range(len(wps))}
    edge_widths = {}
    for u, v, w in kept:
        edge_widths["%d:%d" % (u, v)] = round(w, 3)
        edge_widths["%d:%d" % (v, u)] = round(w, 3)

    pairs = [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]
    meta = dict(version=1, id=uuid.uuid4().hex, frame="map", calibrated=True, max_rms=0.075,
                calibration=dict(source=pairs, target=pairs,
                                 transform=dict(x=0.0, y=0.0, yaw=0.0, rms=0.0,
                                                maximum=0.0, scale_ratio=1.0)),
                edge_widths=edge_widths, narrow_enabled=True, narrow_nodes=[],
                sources=dict(localization=str(MAP_DIR), roads="skeleton of the lidar map"),
                note="graph from the 2026-10-31 lidar map")
    sha = {}
    for path in OUT.iterdir():
        sha[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    meta["sha256"] = sha
    (OUT / "sandbox.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2),
                                      encoding="utf-8")
    print("wrote", OUT, "nodes", len(wps), "edges", len(kept))

    sys.path.insert(0, str(Path("app").resolve()))
    from carapp.core.sandbox_project import load_project
    from carapp.core.sandbox_route import plan_route
    project = load_project(OUT / "sandbox.json")
    ok = fail = 0
    lengths = []
    ids = sorted(n for n in project.network.nodes)
    for b in ids[1:]:
        try:
            route = plan_route(project, (wps[0][0], wps[0][1], 0.0), b)
            ok += 1
            lengths.append(route.length)
        except ValueError:
            fail += 1
    print("routes: %d ok, %d failed" % (ok, fail))
    if lengths:
        print("lengths %.2f .. %.2f m" % (min(lengths), max(lengths)))


if __name__ == "__main__":
    build()
