# -*- coding: utf-8 -*-
"""Rebuild the sandbox plan drawing (2.97 x 5.40 m) from the 2026-10-31 lidar map.

    python redraw_plan_from_lidar_v2.py            # drawing + quality metrics
    python redraw_plan_from_lidar_v2.py --build    # also build the sandbox project

Frame: the drawing covers the sandbox road area inside the raised rim.  The lidar
map shows the outer edge of that rim at 5.66 x 3.23 m, i.e. 0.13 m outside the
2.97 x 5.40 m road boundary on every side; the plan frame is that inner
rectangle, so the transform plan -> map is a pure rotation + translation.
"""
import sys
from pathlib import Path

import numpy as np
import scipy.ndimage as ndi
import yaml
from PIL import Image

LIDAR = Path("app/savedMap/sandbox_20261031_2025")
OUT_DIR = Path("app/savedMap/manual_map_lidar")
PROJECT = Path("app/savedMap/sandbox_lidar_20261031")

PW, PH = 594, 1080            # 2.97 x 5.40 m at 0.005 m/px
PRES = 0.005
CLEARANCE = 0.035
DISTRICT_CLOSE = 0.13
RIM = 0.13                    # outer rim of the sandbox bed beyond the road area

PLAN = np.array([[0.0, 0.0], [2.97, 0.0], [2.97, 5.40], [0.0, 5.40]])


def load_map(folder):
    meta = yaml.safe_load((folder / "map.yaml").read_text(encoding="utf-8"))
    img = np.array(Image.open(folder / meta["image"]).convert("L"))
    return img, float(meta["resolution"]), tuple(float(v) for v in meta["origin"])


real, RRES, RORI = load_map(LIDAR)
RH, RW = real.shape
occ = real <= 60
free = real >= 250
d_occ = ndi.distance_transform_edt(~occ) * RRES


def rail_rectangle():
    """Fit the raised rim: principal axis + the two outer histogram peaks per axis."""
    closed = ndi.binary_closing(occ, structure=np.ones((3, 3), bool), iterations=2)
    lab, n = ndi.label(closed)
    sizes = ndi.sum(closed, lab, range(1, n + 1))
    ring = lab == (int(np.argmax(sizes)) + 1)
    ys, xs = np.nonzero(ring)
    px = xs * RRES + RORI[0]
    py = (RH - 1 - ys) * RRES + RORI[1]
    pts = np.column_stack([px, py])
    centre = pts.mean(0)
    w, v = np.linalg.eigh(np.cov((pts - centre).T))
    axis = v[:, int(np.argmax(w))]
    theta = np.arctan2(axis[1], axis[0])
    u_axis = np.array([np.cos(theta), np.sin(theta)])
    v_axis = np.array([-np.sin(theta), np.cos(theta)])
    u = (pts - centre) @ u_axis
    vv = (pts - centre) @ v_axis

    def peaks(values, want, top=14):
        hist, edges = np.histogram(values, bins=np.arange(-4, 4, 0.02))
        centres = (edges[:-1] + edges[1:]) / 2
        idx = np.argsort(hist)[::-1][:top]
        best = None
        for i in range(len(idx)):
            for j in range(i + 1, len(idx)):
                pair = sorted((centres[idx[i]], centres[idx[j]]))
                sep = pair[1] - pair[0]
                if best is None or abs(sep - want) < abs(best[0] - want):
                    best = (sep, pair[0], pair[1])
        return best[1], best[2]

    ulo, uhi = peaks(u, 5.40 + 2 * RIM)
    vlo, vhi = peaks(vv, 2.97 + 2 * RIM)
    return centre, theta, (ulo, uhi), (vlo, vhi)


CENTRE, THETA, (ULO, UHI), (VLO, VHI) = rail_rectangle()
U_AXIS = np.array([np.cos(THETA), np.sin(THETA)])
V_AXIS = np.array([-np.sin(THETA), np.cos(THETA)])
U_ROAD = (ULO + RIM, UHI - RIM)
V_ROAD = (VLO + RIM, VHI - RIM)
UC = (U_ROAD[0] + U_ROAD[1]) / 2
VC = (V_ROAD[0] + V_ROAD[1]) / 2
MAP_CENTRE = CENTRE + UC * U_AXIS + VC * V_AXIS
PLAN_CENTRE = np.array([1.485, 2.70])
# plan x -> +-perpendicular of the long axis, plan y -> +-long axis; the two signs
# differ by a 180 deg turn, so pick the one that keeps the nine template junctions
# in open corridors (the layout is not symmetric).
PERP = np.array([U_AXIS[1], -U_AXIS[0]])
TEMPLATE_XS, TEMPLATE_YS = (.25, 1.65, 2.845), (.27, 3.125, 5.13)


def _rotation(sign):
    return np.column_stack([sign * PERP, sign * U_AXIS])


def _clearance(rot, plan_pt):
    p = (np.asarray(plan_pt, float) - PLAN_CENTRE) @ rot.T + MAP_CENTRE
    xs = (p[0] - RORI[0]) / RRES
    ys = RH - 1 - (p[1] - RORI[1]) / RRES
    return float(ndi.map_coordinates(d_occ, [[ys], [xs]], order=1, mode="nearest")[0])


def choose_rotation():
    best = None
    for sign in (1.0, -1.0):
        rot = _rotation(sign)
        values = [_clearance(rot, (x, y)) for y in TEMPLATE_YS for x in TEMPLATE_XS]
        score = min(values), sum(values) / len(values)
        print("rotation sign %+d: min clearance %.3f m, mean %.3f m" % (sign, score[0], score[1]))
        if best is None or score > best[0]:
            best = (score, rot, sign)
    print("chosen rotation sign %+d" % best[2])
    return best[1]


ROT = choose_rotation()


def to_map(pts):
    return (np.asarray(pts, float) - PLAN_CENTRE) @ ROT.T + MAP_CENTRE


def to_plan(pts):
    return (np.asarray(pts, float) - MAP_CENTRE) @ ROT + PLAN_CENTRE


def sample(grid, plan_pts):
    p = to_map(plan_pts)
    xs = (p[:, 0] - RORI[0]) / RRES
    ys = RH - 1 - (p[:, 1] - RORI[1]) / RRES
    return ndi.map_coordinates(grid, [ys, xs], order=0, mode="nearest")


def plan_points(mask, limit=8000, seed=11):
    ys, xs = np.nonzero(mask)
    if len(xs) > limit:
        idx = np.random.default_rng(seed).choice(len(xs), limit, replace=False)
        ys, xs = ys[idx], xs[idx]
    return np.column_stack([(xs + .5) * PRES, (PH - ys - .5) * PRES])


def make_drawing():
    closed = ndi.binary_closing(occ, structure=np.ones((3, 3), bool),
                                iterations=max(1, int(round(DISTRICT_CLOSE / RRES / 2))))
    gy, gx = np.mgrid[0:PH, 0:PW]
    plan_pts = np.column_stack([((gx.ravel() + .5) * PRES), ((PH - gy.ravel() - .5) * PRES)])
    lid = to_map(plan_pts)
    xi = np.clip(np.round((lid[:, 0] - RORI[0]) / RRES).astype(int), 0, RW - 1)
    yi = np.clip(np.round(RH - 1 - (lid[:, 1] - RORI[1]) / RRES).astype(int), 0, RH - 1)
    blocked = closed[yi, xi] | (d_occ[yi, xi] <= CLEARANCE)
    img = np.where(blocked, 0, 255).astype(np.uint8).reshape(PH, PW)
    img[0, :] = img[-1, :] = 0
    img[:, 0] = img[:, -1] = 0
    return img


def metrics(img):
    plan_road = plan_points(img >= 250)
    plan_block = plan_points(img <= 10)
    road_free = (sample(real, plan_road) >= 250).mean()
    road_clear = sample(d_occ, plan_road).mean()
    block_hit = (sample(real, plan_block) <= 60).mean()
    block_dist = sample(d_occ, plan_block).mean()
    print("road samples: free %.1f%%, mean clearance %.3f m" % (road_free * 100, road_clear))
    print("district samples: on obstacle %.1f%%, mean distance %.3f m" % (block_hit * 100, block_dist))
    return road_free, road_clear, block_hit, block_dist


def write_overlay(img):
    rgb = np.dstack([real] * 3).astype(np.uint8)
    gy, gx = np.mgrid[0:PH, 0:PW]
    lid = to_map(np.column_stack([((gx.ravel() + .5) * PRES), ((PH - gy.ravel() - .5) * PRES)]))
    xi = np.round((lid[:, 0] - RORI[0]) / RRES).astype(int)
    yi = np.round(RH - 1 - (lid[:, 1] - RORI[1]) / RRES).astype(int)
    ok = (xi >= 0) & (xi < RW) & (yi >= 0) & (yi < RH)
    block = img.ravel()[ok] == 0
    rgb[yi[ok][block], xi[ok][block]] = (255, 70, 70)
    rgb[yi[ok][~block], xi[ok][~block]] = (70, 130, 255)
    Image.fromarray(rgb).resize((RW * 3, RH * 3), Image.NEAREST).save(
        ".work/maps/plan_lidar_overlay_v2.png")


def build_project_now():
    sys.path.insert(0, str(Path("app").resolve()))
    from carapp.core.sandbox_project import build_project, load_project
    source = PLAN.tolist()
    target = [list(map(float, p)) for p in to_map(PLAN)]
    print("registration pairs:")
    for s, t in zip(source, target):
        print("  plan (%.3f, %.3f) -> map (%.3f, %.3f)" % (s[0], s[1], t[0], t[1]))
    if not PROJECT.exists():
        build_project(LIDAR / "map.yaml", OUT_DIR / "rawMap.yaml",
                      source, target, PROJECT)
    else:
        print("reusing project", PROJECT)
    project = load_project(PROJECT / "sandbox.json")
    print("project at", PROJECT)
    keepout = project.keepout
    grid = keepout.grid
    data = grid["data"]
    free_ratio = sum(1 for v in data if v == 0) / float(len(data))
    print("keepout free ratio: %.3f" % free_ratio)
    nodes = project.network.nodes
    items = nodes.items() if hasattr(nodes, "items") else [(n.id, n) for n in nodes]
    for node_id, node in sorted(items):
        gx = int((node.x - grid["origin"][0]) / grid["resolution"])
        gy = int((node.y - grid["origin"][1]) / grid["resolution"])
        row = grid["height"] - 1 - gy
        value = data[row * grid["width"] + gx] if 0 <= row < grid["height"] else None
        print("node %s map (%.2f, %.2f) keepout=%s" % (node_id, node.x, node.y,
                                                      "free" if value == 0 else value))
    return project


if __name__ == "__main__":
    print("rim rails: u %.3f..%.3f  v %.3f..%.3f  theta %.2f deg"
          % (ULO, UHI, VLO, VHI, np.degrees(THETA)))
    print("road area: %.3f x %.3f m, map centre (%.3f, %.3f)"
          % (U_ROAD[1] - U_ROAD[0], V_ROAD[1] - V_ROAD[0], MAP_CENTRE[0], MAP_CENTRE[1]))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    img = make_drawing()
    (OUT_DIR / "rawMap.pgm").write_bytes(b"P5\n%d %d\n255\n" % (PW, PH) + img.tobytes())
    (OUT_DIR / "rawMap.yaml").write_text(
        "# Sandbox 2.97 m x 5.40 m road area, rebuilt from the 2026-10-31 lidar map.\n"
        "image: rawMap.pgm\nmode: trinary\nresolution: 0.005\norigin: [0.0, 0.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n", encoding="utf-8")
    import json
    (OUT_DIR / "transform.json").write_text(json.dumps({
        "frame": "plan(2.97x5.40) -> lidar map",
        "centre_map": MAP_CENTRE.tolist(), "yaw_deg": float(np.degrees(THETA - np.pi / 2)),
        "rotation": ROT.tolist(), "rim": RIM, "rails": {"u": [ULO, UHI], "v": [VLO, VHI]},
    }, indent=2), encoding="utf-8")
    print("wrote", OUT_DIR)
    metrics(img)
    write_overlay(img)
    print("overlay -> .work/maps/plan_lidar_overlay_v2.png")
    if "--build" in sys.argv:
        build_project_now()
