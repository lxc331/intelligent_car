# -*- coding: utf-8 -*-
"""Which plan->map rotation puts the nine-junction template on real corridors?"""
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

U = np.array([np.cos(np.radians(46.97)), np.sin(np.radians(46.97))])
PERP = np.array([U[1], -U[0]])
CENTRE = np.array([1.297, -0.491])
PLAN_CENTRE = np.array([1.485, 2.70])
XS, YS = (.25, 1.65, 2.845), (.27, 3.125, 5.13)


def sample(p):
    xs = (p[0] - OX) / RES
    ys = H - 1 - (p[1] - OY) / RES
    return ndi.map_coordinates(d_occ, [[ys], [xs]], order=1, mode="nearest")[0]


for s in (1.0, -1.0):
    rot = np.column_stack([s * PERP, s * U])
    clear = []
    print("rot s=%+d  yaw %.1f deg" % (s, np.degrees(np.arctan2(rot[1, 1], rot[0, 1]))))
    for row, y in enumerate(YS):
        line = []
        for col, x in enumerate(XS):
            p = (np.array([x, y]) - PLAN_CENTRE) @ rot.T + CENTRE
            c = sample(p)
            clear.append(c)
            line.append("(%.2f,%.2f)%.2fm" % (p[0], p[1], c))
        print("  row%d " % row + "  ".join(line))
    print("  min clearance %.3f m, mean %.3f m" % (min(clear), sum(clear) / len(clear)))
