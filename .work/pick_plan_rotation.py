# -*- coding: utf-8 -*-
"""Pick which of the two plan->map rotations fits the drawing's model districts."""
import numpy as np
import scipy.ndimage as ndi
import yaml
from PIL import Image

P = "app/savedMap/sandbox_20261031_2025"
m = yaml.safe_load(open(P + "/map.yaml", encoding="utf-8"))
real = np.array(Image.open(P + "/" + m["image"]).convert("L"))
RES = float(m["resolution"])
OX, OY = float(m["origin"][0]), float(m["origin"][1])
H, W = real.shape
occ = real <= 60
d_occ = ndi.distance_transform_edt(~occ) * RES

raw = np.array(Image.open("app/savedMap/manual_map/rawMap.pgm").convert("L"))
PH, PW = raw.shape


def pts(mask, limit=6000, seed=5):
    ys, xs = np.nonzero(mask)
    if len(xs) > limit:
        idx = np.random.default_rng(seed).choice(len(xs), limit, replace=False)
        ys, xs = ys[idx], xs[idx]
    return np.column_stack([(xs + .5) * .005, (PH - ys - .5) * .005])


block = pts(raw <= 10)
road = pts(raw >= 250)

U = np.array([np.cos(np.radians(46.97)), np.sin(np.radians(46.97))])
PERP = np.array([U[1], -U[0]])
CENTRE = np.array([1.297, -0.491])          # plan centre lands here (plate centre)
PLAN_CENTRE = np.array([1.485, 2.70])


def sample(grid, p):
    xs = (p[:, 0] - OX) / RES
    ys = H - 1 - (p[:, 1] - OY) / RES
    return ndi.map_coordinates(grid, [ys, xs], order=0, mode="nearest")


for s in (1.0, -1.0):
    rot = np.column_stack([s * PERP, s * U])
    f = lambda q: (np.asarray(q, float) - PLAN_CENTRE) @ rot.T + CENTRE
    b = f(block)
    r = f(road)
    print("s=%+d  district on obstacle %.1f%%, mean dist %.3f m; road free %.1f%%, clearance %.3f m"
          % (s, (sample(real, b) <= 60).mean() * 100, sample(d_occ, b).mean(),
             (sample(real, r) >= 250).mean() * 100, sample(d_occ, r).mean()))
