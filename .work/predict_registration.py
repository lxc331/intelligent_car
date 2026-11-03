# -*- coding: utf-8 -*-
"""Predict the registration accuracy from the four sandbox board edges.

    python predict_registration.py

Finds the two long and two short board edges from the Hough peak list of the
new map (computed earlier), turns them into four corner points in metres, and
fits a similarity transform from the hand-drawn 2.97 x 5.40 m plan to them.
"""
import numpy as np

RES = 0.05
ORIGIN = (-5.04, -7.53)
H = 270

# Hough peaks found in .work/maps/map_20261031_2025 (theta deg, rho px)
LONG = [(45.5, 101.0), (45.5, 214.0)]
SHORT = [(135.5, -118.0), (135.5, -59.0)]


def intersect(t1, r1, t2, r2):
    a = np.array([[np.cos(t1), np.sin(t1)], [np.cos(t2), np.sin(t2)]])
    return np.linalg.solve(a, np.array([r1, r2]))


def px_to_m(p):
    return np.array([ORIGIN[0] + p[0] * RES, ORIGIN[1] + (H - 1 - p[1]) * RES])


corners = []
for t_l, r_l in LONG:
    for t_s, r_s in SHORT:
        corners.append(px_to_m(intersect(np.deg2rad(t_l), r_l, np.deg2rad(t_s), r_s)))
corners = np.array(corners)

# Hand-drawn plan corners, same winding as produced above (CCW seen from top).
plan = np.array([[0.0, 0.0], [2.97, 0.0], [0.0, 5.40], [2.97, 5.40]])

# order both sets counter-clockwise around their centroids
def ccw(pts):
    c = pts.mean(axis=0)
    ang = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
    return pts[np.argsort(ang)]

corners_ccw = ccw(corners)
plan_ccw = ccw(plan)
if np.linalg.det(np.c_[corners_ccw[:2] - corners_ccw[0]]) < 0:
    pass

# Umeyama similarity fit: plan -> map
src, dst = plan_ccw, corners_ccw
mu_s, mu_d = src.mean(0), dst.mean(0)
s0, d0 = src - mu_s, dst - mu_d
cov = d0.T @ s0 / len(src)
u, sv, vt = np.linalg.svd(cov)
s = np.eye(2)
if np.linalg.det(u) * np.linalg.det(vt) < 0:
    s[-1, -1] = -1
rot = u @ s @ vt
scale = np.trace(np.diag(sv) @ s) / (s0 ** 2).sum(axis=1).mean()
pred = scale * s0 @ rot.T + mu_d
res = np.linalg.norm(pred - dst, axis=1)

print("map corners (m):")
for p in corners_ccw:
    print("  (%.3f, %.3f)" % tuple(p))
print("\nplan corners (m):")
for p in plan_ccw:
    print("  (%.3f, %.3f)" % tuple(p))
print("\nscale  %.4f   (1.0 = no scale correction)" % scale)
print("rotation %.2f deg" % np.degrees(np.arctan2(rot[1, 0], rot[0, 0])))
print("translation  dx %.3f  dy %.3f m" % tuple(mu_d - scale * (mu_s @ rot.T)))
print("RMS residual %.3f m   max %.3f m" % (np.sqrt((res ** 2).mean()), res.max()))
print("residuals:", " ".join("%.3f" % r for r in res))
