# -*- coding: utf-8 -*-
"""Measure the big occupied loops (sandbox board, room walls) in a ROS map.

    python measure_loops.py MAP_DIR

Prints each sizable connected component with its oriented extents in metres,
so two maps of the same panorama can be compared before registration.
"""
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


def load(folder):
    folder = Path(folder)
    meta = yaml.safe_load((folder / "map.yaml").read_text(encoding="utf-8"))
    res = float(meta["resolution"])
    img = np.array(Image.open(folder / meta["image"]).convert("L"))
    return img, res


def components(points, w, h):
    """4-connected components over occupied pixels, iterative flood fill."""
    mask = np.zeros((h, w), dtype=bool)
    mask[points[:, 1], points[:, 0]] = True
    seen = np.zeros_like(mask)
    out = []
    for y in range(h):
        for x in range(w):
            if not mask[y, x] or seen[y, x]:
                continue
            stack = [(x, y)]
            seen[y, x] = True
            pix = []
            while stack:
                cx, cy = stack.pop()
                pix.append((cx, cy))
                for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                    if 0 <= nx < w and 0 <= ny < h and mask[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        stack.append((nx, ny))
            if len(pix) >= 300:
                out.append(np.array(pix, dtype=float))
    return out


def oriented_extents(px, res):
    centre = px.mean(axis=0)
    q = px - centre
    cov = np.cov(q.T)
    vals, vecs = np.linalg.eigh(cov)
    order = np.argsort(vals)[::-1]
    axes = vecs[:, order]
    proj = q @ axes
    ext = proj.max(axis=0) - proj.min(axis=0)
    angle = np.degrees(np.arctan2(axes[1, 0], axes[0, 0]))
    return centre, ext * res, angle


def report(name, folder):
    img, res = load(folder)
    h, w = img.shape
    ys, xs = np.nonzero(img <= 60)
    print("\n== %s  %dx%d @%.3f m/px" % (name, w, h, res))
    print("occupied px:", len(xs))
    comps = components(np.column_stack([xs, ys]), w, h)
    rows = []
    for px in comps:
        centre, ext, angle = oriented_extents(px, res)
        rows.append((float(ext[0] * ext[1]), len(px), ext, angle, centre))
    rows.sort(reverse=True, key=lambda r: r[0])
    for area, count, ext, angle, centre in rows[:8]:
        print("  comp px=%6d  extents %.2f x %.2f m  angle %+6.1f deg  centre (%.2f, %.2f) m"
              % (count, ext[0], ext[1], angle, centre[0] * res, centre[1] * res))


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        report(arg, arg)
