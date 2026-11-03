# -*- coding: utf-8 -*-
"""Rebuild the 2.97 x 5.40 m sandbox plan drawing from the 2026-10-31 lidar map.

    python redraw_plan_from_lidar.py          # write app/savedMap/manual_map_lidar
    python redraw_plan_from_lidar.py --build  # also build app/savedMap/sandbox_lidar_20261031

The plan frame is the 2.97 x 5.40 m board the app's nine-junction template
expects; the four measured outer-rail corners of the lidar map give the
similarity that maps plan -> map.  The new drawing keeps that same scale, so
the previous hand drawing (idealised rectangles) is replaced by obstacles
warped straight out of the lidar map.
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
CLEARANCE = 0.035             # plan keeps this far from a lidar hit
DISTRICT_CLOSE = 0.13         # bridge model gaps into a blocked district

PLAN = np.array([[0.0, 0.0], [2.97, 0.0], [2.97, 5.40], [0.0, 5.40]])
BOARD = np.array([[2.708, 6.453], [0.604, 4.386], [4.564, 0.356], [6.668, 2.424]])
# BOARD sides: 2.95, 5.65, 2.95, 5.65 m -- measured outer rails, same winding as PLAN


def load_map(folder):
    meta = yaml.safe_load((folder / "map.yaml").read_text(encoding="utf-8"))
    img = np.array(Image.open(folder / meta["image"]).convert("L"))
    return img, float(meta["resolution"]), tuple(float(v) for v in meta["origin"])


def fit_similarity(src, dst):
    """dst = s * R(yaw) * src + t, least squares over the point pairs."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s0, d0 = src - mu_s, dst - mu_d
    en = (s0 ** 2).sum()
    a = (s0 * d0).sum() / en
    b = (s0[:, 0] * d0[:, 1] - s0[:, 1] * d0[:, 0]).sum() / en
    scale = np.hypot(a, b)
    yaw = np.arctan2(b, a)
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    trans = mu_d - scale * (mu_s @ rot.T)
    resid = scale * (src @ rot.T) + trans - dst
    rms = float(np.sqrt((resid ** 2).sum(1).mean()))
    return scale, yaw, trans, rms


real, RRES, RORI = load_map(LIDAR)
RH, RW = real.shape
occ = real <= 60
d_occ = ndi.distance_transform_edt(~occ) * RRES

SCALE, YAW, TRANS, RMS = fit_similarity(PLAN, BOARD)


def to_map(pts):
    c, s = np.cos(YAW), np.sin(YAW)
    rot = np.array([[c, -s], [s, c]])
    return SCALE * (np.asarray(pts, float) @ rot.T) + TRANS


def sample(grid, pts):
    p = to_map(pts)
    xs = (p[:, 0] - RORI[0]) / RRES
    ys = RH - 1 - (p[:, 1] - RORI[1]) / RRES
    return ndi.map_coordinates(grid, [ys, xs], order=0, mode="nearest")


def plan_points(mask, limit=8000, seed=11):
    ys, xs = np.nonzero(mask)
    if len(xs) > limit:
        idx = np.random.default_rng(seed).choice(len(xs), limit, replace=False)
        ys, xs = ys[idx], xs[idx]
    return np.column_stack([(xs + .5) * PRES, (PH - ys - .5) * PRES])


def inside_board(pts):
    """convex quad test for points already in map metres."""
    inside = np.ones(len(pts), dtype=bool)
    for i in range(4):
        a, b = BOARD[i], BOARD[(i + 1) % 4]
        edge = b - a
        rel = pts - a
        inside &= (edge[0] * rel[:, 1] - edge[1] * rel[:, 0]) >= -1e-9
    return inside


def make_drawing():
    closed = ndi.binary_closing(occ, structure=np.ones((3, 3), bool),
                                iterations=max(1, int(round(DISTRICT_CLOSE / RRES / 2))))
    gy, gx = np.mgrid[0:PH, 0:PW]
    pts = np.column_stack([((gx.ravel() + .5) * PRES), ((PH - gy.ravel() - .5) * PRES)])
    lid = to_map(pts)
    xi = np.clip(np.round((lid[:, 0] - RORI[0]) / RRES).astype(int), 0, RW - 1)
    yi = np.clip(np.round(RH - 1 - (lid[:, 1] - RORI[1]) / RRES).astype(int), 0, RH - 1)
    blocked = closed[yi, xi] | (d_occ[yi, xi] <= CLEARANCE) | ~inside_board(lid)
    img = np.where(blocked, 0, 255).astype(np.uint8).reshape(PH, PW)
    img[0, :] = img[-1, :] = 0
    img[:, 0] = img[:, -1] = 0
    return img


def report_quality():
    raw = np.array(Image.open("app/savedMap/manual_map/rawMap.pgm").convert("L"))
    old_road = sample(d_occ, plan_points(raw >= 250))
    old_block = sample(d_occ, plan_points(raw <= 10))
    print("old hand-drawn plan: road clearance %.3f m, district distance-to-hit %.3f m"
          % (old_road.mean(), old_block.mean()))
    img = np.array(Image.open(OUT_DIR / "rawMap.pgm").convert("L"))
    new_road = sample(d_occ, plan_points(img >= 250))
    new_block = sample(d_occ, plan_points(img <= 10))
    print("new lidar plan:      road clearance %.3f m, district distance-to-hit %.3f m"
          % (new_road.mean(), new_block.mean()))


def write_overlay(img):
    rgb = np.dstack([real] * 3).astype(np.uint8)
    gy, gx = np.mgrid[0:PH, 0:PW]
    pts = np.column_stack([((gx.ravel() + .5) * PRES), ((PH - gy.ravel() - .5) * PRES)])
    lid = to_map(pts)
    xi = np.round((lid[:, 0] - RORI[0]) / RRES).astype(int)
    yi = np.round(RH - 1 - (lid[:, 1] - RORI[1]) / RRES).astype(int)
    ok = (xi >= 0) & (xi < RW) & (yi >= 0) & (yi < RH)
    block = img.ravel()[ok] == 0
    rgb[yi[ok][block], xi[ok][block]] = (255, 70, 70)
    rgb[yi[ok][~block], xi[ok][~block]] = (70, 130, 255)
    Image.fromarray(rgb).resize((RW * 3, RH * 3), Image.NEAREST).save(
        ".work/maps/plan_lidar_overlay.png")


def build_project_now():
    sys.path.insert(0, str(Path("app").resolve()))
    from carapp.core.sandbox_project import build_project, load_project
    if PROJECT.exists():
        print("project already exists:", PROJECT)
        return
    result = build_project(LIDAR / "map.yaml", OUT_DIR / "rawMap.yaml",
                           PLAN.tolist(), BOARD.tolist(), PROJECT)
    project = load_project(PROJECT)
    print("project built:", PROJECT)
    print("nodes:", [(round(n.x, 3), round(n.y, 3)) for n in project.network.nodes][:12])
    print("edge widths:", project.metadata.get("edge_widths"))
    keepout = project.keepout
    data = keepout.grid["data"]
    free = sum(1 for v in data if v == 0) / float(len(data))
    print("keepout free ratio: %.3f" % free)
    for node in project.network.nodes:
        gx, gy = int((node.x - keepout.grid["origin"][0]) / keepout.grid["resolution"]), \
                 int((node.y - keepout.grid["origin"][1]) / keepout.grid["resolution"])
        row = keepout.grid["height"] - 1 - gy
        value = data[row * keepout.grid["width"] + gx] if 0 <= row < keepout.grid["height"] else None
        print("node %s at (%.2f, %.2f) keepout=%s" % (node.id, node.x, node.y,
                                                      "free" if value == 0 else value))
    return result


if __name__ == "__main__":
    print("transform plan->map: scale %.4f yaw %.2f deg t (%.3f, %.3f) corner RMS %.1f cm"
          % (SCALE, np.degrees(YAW), TRANS[0], TRANS[1], RMS * 100))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    img = make_drawing()
    (OUT_DIR / "rawMap.pgm").write_bytes(b"P5\n%d %d\n255\n" % (PW, PH) + img.tobytes())
    (OUT_DIR / "rawMap.yaml").write_text(
        "# Sandbox 2.97 m x 5.40 m (plan frame), rebuilt from the 2026-10-31 lidar map.\n"
        "image: rawMap.pgm\nmode: trinary\nresolution: 0.005\norigin: [0.0, 0.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n", encoding="utf-8")
    import json
    (OUT_DIR / "transform.json").write_text(json.dumps({
        "scale": SCALE, "yaw": YAW, "translation": [TRANS[0], TRANS[1]],
        "corner_rms_m": RMS, "frame": "plan(2.97x5.40) -> lidar map"}, indent=2), encoding="utf-8")
    print("wrote", OUT_DIR)
    report_quality()
    write_overlay(img)
    print("overlay -> .work/maps/plan_lidar_overlay.png")
    if "--build" in sys.argv:
        build_project_now()
