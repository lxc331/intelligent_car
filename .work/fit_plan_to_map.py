# -*- coding: utf-8 -*-
"""Best-fit similarity between the hand-drawn sandbox plan and the new lidar map.

    python fit_plan_to_map.py

Starts from the four outer-rail corners measured in the new map, then refines
rotation / scale / translation against the drawing's free-space mask.  Prints
how well the drawing agrees with reality and writes an overlay PNG.
"""
import numpy as np
import yaml
from PIL import Image

REAL_DIR = "app/savedMap/sandbox_20261031_2025"
PLAN_DIR = "app/savedMap/manual_map"
OUT = ".work/maps/plan_vs_lidar_overlay.png"


def load(folder, name="map.yaml"):
    meta = yaml.safe_load(open(folder + "/" + name, encoding="utf-8").read())
    img = np.array(Image.open(folder + "/" + meta["image"]).convert("L"))
    return img, float(meta["resolution"]), tuple(float(v) for v in meta["origin"])


real, rres, rorigin = load(REAL_DIR)
plan, pres, porigin = load(PLAN_DIR, "rawMap.yaml")
RH, RW = real.shape
PH, PW = plan.shape

free_real = real >= 250
occ_real = real <= 60
road_plan = plan >= 250
block_plan = plan <= 10


def sample(mask, limit=4000, seed=7):
    ys, xs = np.nonzero(mask)
    if len(xs) > limit:
        idx = np.random.default_rng(seed).choice(len(xs), limit, replace=False)
        ys, xs = ys[idx], xs[idx]
    return xs, ys


ROAD_X, ROAD_Y = sample(road_plan)
BLOCK_X, BLOCK_Y = sample(block_plan)


def plan_metres(xs, ys):
    return np.column_stack([(xs + 0.5) * pres + porigin[0], (PH - ys - 0.5) * pres + porigin[1]])


ROAD_M = plan_metres(ROAD_X, ROAD_Y)
BLOCK_M = plan_metres(BLOCK_X, BLOCK_Y)

C = np.array([[2.708, 6.453], [0.604, 4.386], [4.564, 0.356], [6.668, 2.424]])
real_rect = np.array([C[0], C[1], C[3], C[2]])
plan_corners = np.array([[0.0, 0.0], [2.97, 0.0], [2.97, 5.40], [0.0, 5.40]])


def similarity(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s0, d0 = src - mu_s, dst - mu_d
    dot = (s0 * d0).sum()
    cross = (s0[:, 0] * d0[:, 1] - s0[:, 1] * d0[:, 0]).sum()
    en = (s0 ** 2).sum()
    a, b = dot / en, cross / en
    scale = np.hypot(a, b)
    yaw = np.arctan2(b, a)
    rot = np.array([[a, -b], [b, a]]) / scale
    t = mu_d - scale * (mu_s @ rot.T)
    return scale, yaw, t


def to_px(pts_m):
    px = (pts_m[:, 0] - rorigin[0]) / rres
    py = RH - 1 - (pts_m[:, 1] - rorigin[1]) / rres
    return np.column_stack([px, py])


def transform_points(pts_m, scale, yaw, t):
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    return scale * (pts_m @ rot.T) + t


def evaluate(scale, yaw, t):
    q = to_px(transform_points(ROAD_M, scale, yaw, t))
    inside = (q[:, 0] >= 0) & (q[:, 0] < RW) & (q[:, 1] >= 0) & (q[:, 1] < RH)
    xi = np.clip(np.round(q[:, 0]).astype(int), 0, RW - 1)
    yi = np.clip(np.round(q[:, 1]).astype(int), 0, RH - 1)
    rf = (free_real[yi, xi] & inside).sum() / max(1, inside.sum())
    ro = (occ_real[yi, xi] & inside).sum() / max(1, inside.sum())
    q2 = to_px(transform_points(BLOCK_M, scale, yaw, t))
    in2 = (q2[:, 0] >= 0) & (q2[:, 0] < RW) & (q2[:, 1] >= 0) & (q2[:, 1] < RH)
    xi2 = np.clip(np.round(q2[:, 0]).astype(int), 0, RW - 1)
    yi2 = np.clip(np.round(q2[:, 1]).astype(int), 0, RH - 1)
    rb = (occ_real[yi2, xi2] & in2).sum() / max(1, in2.sum())
    return rf - 2.0 * ro + 0.3 * rb, rf, ro, rb


def coarse():
    best = None
    for k in range(4):
        rc = np.roll(real_rect, k, axis=0)
        for flip in (False, True):
            r = rc[::-1] if flip else rc
            scale, yaw, t = similarity(plan_corners, r)
            sc, rf, ro, rb = evaluate(scale, yaw, t)
            if best is None or sc > best[0]:
                best = (sc, scale, yaw, t, rf, ro, rb)
    return best


def refine(scale, yaw, t):
    best = (-9, scale, yaw, t, 0, 0, 0)
    for ds in np.arange(-.06, .061, .01):
        for dy in np.radians(np.arange(-3, 3.01, 1.0)):
            for dx in np.arange(-.15, .151, .05):
                for dyy in np.arange(-.15, .151, .05):
                    s2, y2, t2 = scale + ds, yaw + dy, t + np.array([dx, dyy])
                    sc, rf, ro, rb = evaluate(s2, y2, t2)
                    if sc > best[0]:
                        best = (sc, s2, y2, t2, rf, ro, rb)
    return best


if __name__ == "__main__":
    sc, scale, yaw, t, rf, ro, rb = coarse()
    print("coarse : score %.3f scale %.4f yaw %.2f t (%.3f,%.3f) road->free %.3f road->occ %.3f block->occ %.3f"
          % (sc, scale, np.degrees(yaw), t[0], t[1], rf, ro, rb))
    sc, scale, yaw, t, rf, ro, rb = refine(scale, yaw, t)
    print("refined: score %.3f scale %.4f yaw %.2f t (%.3f,%.3f) road->free %.3f road->occ %.3f block->occ %.3f"
          % (sc, scale, np.degrees(yaw), t[0], t[1], rf, ro, rb))

    rgb = np.dstack([real, real, real]).astype(np.uint8)
    for pts, colour in ((ROAD_M, (40, 120, 255)), (BLOCK_M, (255, 60, 60))):
        q = np.round(to_px(transform_points(pts, scale, yaw, t))).astype(int)
        ok = (q[:, 0] >= 0) & (q[:, 0] < RW) & (q[:, 1] >= 0) & (q[:, 1] < RH)
        rgb[q[ok, 1], q[ok, 0]] = colour
    Image.fromarray(rgb).resize((RW * 3, RH * 3), Image.NEAREST).save(OUT)
    print("overlay ->", OUT)
    for p in plan_corners:
        q = transform_points(np.array([p]), scale, yaw, t)[0]
        print("  (%.2f, %.2f) -> (%.3f, %.3f)" % (p[0], p[1], q[0], q[1]))

