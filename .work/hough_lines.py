# -*- coding: utf-8 -*-
"""Hough line peaks of a ROS map, to measure the sandbox board edge spacing.

    python hough_lines.py MAP_DIR [MAP_DIR ...]
"""
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


def load(folder):
    folder = Path(folder)
    meta = yaml.safe_load((folder / "map.yaml").read_text(encoding="utf-8"))
    img = np.array(Image.open(folder / meta["image"]).convert("L"))
    return img, float(meta["resolution"])


def peaks(folder, occ_max=80, n=10):
    img, res = load(folder)
    ys, xs = np.nonzero(img <= occ_max)
    thetas = np.deg2rad(np.arange(0, 180, 0.5))
    diag = int(np.hypot(img.shape[0], img.shape[1])) + 2
    acc = np.zeros((len(thetas), 2 * diag + 1), dtype=np.int32)
    for t_i, t in enumerate(thetas):
        rho = xs * np.cos(t) + ys * np.sin(t)
        idx = np.round(rho + diag).astype(int)
        np.add.at(acc[t_i], idx, 1)
    flat = []
    for t_i in range(acc.shape[0]):
        for r_i in range(1, acc.shape[1] - 1):
            v = acc[t_i, r_i]
            if v >= 12 and v >= acc[t_i, r_i - 1] and v >= acc[t_i, r_i + 1]:
                flat.append((int(v), float(thetas[t_i]), r_i - diag))
    flat.sort(reverse=True)
    kept = []
    for v, t, r in flat:
        if all(abs(r - r2) > 2 or abs(t - t2) > 0.05 for _, t2, r2 in kept):
            kept.append((v, t, r))
        if len(kept) >= n:
            break
    print("\n== %s   %dx%d @%.3f" % (folder, img.shape[1], img.shape[0], res))
    for v, t, r in kept:
        print("  votes %4d  theta %6.1f deg  rho %7.2f px = %6.2f m" % (v, np.rad2deg(t), r, r * res))
    return [(v, t, r * res) for v, t, r in kept]


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        peaks(arg)
