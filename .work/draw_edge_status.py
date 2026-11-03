# -*- coding: utf-8 -*-
"""Show which drawn segments fail the footprint test (red = blocked)."""
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path("app").resolve()))
from carapp.core.sandbox_project import load_project  # noqa: E402

P = Path("app/savedMap/sandbox_drawn_20261031")
project = load_project(P / "sandbox.json")
grid = project.localization.grid
w, h, res = grid["width"], grid["height"], grid["resolution"]
ox, oy = grid["origin"][0], grid["origin"][1]
nodes = project.network.nodes
widths = project.metadata["edge_widths"]
scale = 4
img = Image.open(P / "map.pgm").convert("RGB").resize((w * scale, h * scale), Image.NEAREST)
d = ImageDraw.Draw(img)


def px(n):
    return ((n.x - ox) / res * scale, (h - (n.y - oy) / res) * scale)


for i in nodes:
    for j in nodes[i].to:
        if j <= i:
            continue
        wd = widths.get("%d:%d" % (i, j), 0.0)
        color = (0, 120, 255) if wd >= 0.23 else (255, 0, 0)
        d.line([px(nodes[i]), px(nodes[j])], fill=color, width=2)
for i in nodes:
    x, y = px(nodes[i])
    d.ellipse([x - 1.5, y - 1.5, x + 1.5, y + 1.5], fill=(0, 200, 0))
d.text((6, 6), "blue=ok red=blocked", fill=(0, 0, 0))
out = Path(".work/maps/drawn_edges_status.png")
img.save(out)
print("wrote", out)
