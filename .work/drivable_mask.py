# -*- coding: utf-8 -*-
"""Where can the real footprint actually stand? Render that mask over the map."""
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path("app").resolve()))
from carapp.core.navigation_path import pose_clear  # noqa: E402

P = Path("app/savedMap/sandbox_drawn_20261031")
grid_yaml = (P / "map.yaml").read_text(encoding="utf-8")
import yaml  # noqa: E402

meta = yaml.safe_load(grid_yaml)
pgm = np.array(Image.open(P / meta["image"]).convert("L"))
res = float(meta["resolution"])
ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
h, w = pgm.shape
FOOT = (0.25, 0.15, 0.115)
grid = dict(width=w, height=h, resolution=res, origin=(ox, oy, 0.0),
            data=np.flipud(np.where(pgm >= 250, 0, 100)).astype(np.uint8).ravel().tolist())

mask = np.zeros((h, w), bool)
for r in range(h):
    y = oy + (r + 0.5) * res
    for c in range(w):
        x = ox + (c + 0.5) * res
        for k in range(8):
            if pose_clear(grid, (x, y, k * math.pi / 4), FOOT):
                mask[r, c] = True
                break
print("drivable cells %.1f%%" % (100.0 * mask.mean()))
np.save(".work/maps/drivable_mask.npy", mask)

rgb = np.dstack([pgm, pgm, pgm]).astype(np.uint8)
rgb[mask, 0] = 255
rgb[mask, 1] = 120
rgb[mask, 2] = 60
Image.fromarray(rgb).resize((w * 3, h * 3), Image.NEAREST).save(".work/maps/drivable_overlay.png")
print("wrote .work/maps/drivable_overlay.png")
