# -*- coding: utf-8 -*-
"""Draw the nine-junction network over the lidar map."""
import numpy as np
import yaml
from PIL import Image, ImageDraw

P = "app/savedMap/sandbox_lidar_20261031"
m = yaml.safe_load(open(P + "/map.yaml", encoding="utf-8"))
img = Image.open(P + "/" + m["image"]).convert("L")
W, H = img.size
RES = float(m["resolution"])
OX, OY = float(m["origin"][0]), float(m["origin"][1])

net = yaml.safe_load(open(P + "/road_network.yaml", encoding="utf-8"))
wp = net["waypoints"]
print("waypoints:", len(wp))

rgb = img.convert("RGB")
rgb = rgb.resize((W * 4, H * 4), Image.NEAREST)
d = ImageDraw.Draw(rgb)


def px(x, y):
    return (((x - OX) / RES) * 4, (H - 1 - (y - OY) / RES) * 4)


for n in wp:
    x, y = px(*n["pose"])
    d.ellipse([x - 6, y - 6, x + 6, y + 6], fill=(255, 0, 0))
    d.text((x + 8, y - 6), str(n["id"]), fill=(255, 0, 0))
for n in wp:
    for t in n["to"]:
        other = next(o for o in wp if o["id"] == t)
        d.line([px(*n["pose"]), px(*other["pose"])], fill=(0, 140, 255), width=2)
rgb.save(".work/maps/template_over_map.png")
print("wrote .work/maps/template_over_map.png")
