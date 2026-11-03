# -*- coding: utf-8 -*-
"""Validate the drawn project: load it, plan sample routes, draw an overlay."""
import random
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path("app").resolve()))
from carapp.core.sandbox_project import load_project  # noqa: E402
from carapp.core.sandbox_route import plan_route  # noqa: E402

PROJECT = Path(sys.argv[1] if len(sys.argv) > 1 else "app/savedMap/sandbox_drawn_20261031/sandbox.json")
OUT = Path(".work/maps/drawn_network_overlay.png")


def main():
    project = load_project(PROJECT)
    net = project.network
    nodes = net.nodes
    grid = project.localization.grid
    width, height = grid["width"], grid["height"]
    res = grid["resolution"]
    ox, oy = grid["origin"][0], grid["origin"][1]
    pgm = np.array(Image.open(PROJECT.parent / "map.pgm").convert("L"))
    scale = 4
    canvas = Image.fromarray(pgm).convert("RGB").resize((width * scale, height * scale), Image.NEAREST)
    draw = ImageDraw.Draw(canvas)

    def px(x, y):
        return ((x - ox) / res * scale, (height - (y - oy) / res) * scale)

    for node in nodes.values():
        for other in node.to:
            if other > node.id:
                draw.line([px(*node.pose[:2]), px(*nodes[other].pose[:2])], fill=(0, 120, 255), width=2)
    for node in nodes.values():
        cx, cy = px(*node.pose[:2])
        draw.ellipse([cx - 3, cy - 3, cx + 3, cy + 3], fill=(255, 40, 40))

    rng = random.Random(7)
    ids = sorted(nodes)
    ok = fail = 0
    reasons = {}
    pairs = [(rng.choice(ids), rng.choice(ids)) for _ in range(300)]
    for a, b in pairs:
        if a == b:
            continue
        try:
            plan_route(project, (nodes[a].x, nodes[a].y, 0.0), b)
            ok += 1
        except ValueError as exc:
            fail += 1
            key = str(exc).split("；")[0]
            reasons[key] = reasons.get(key, 0) + 1
    canvas.save(OUT)
    print("nodes %d links %d" % (len(nodes), sum(len(n.to) for n in nodes.values()) // 2))
    print("routes ok %d fail %d (of %d)" % (ok, fail, len(pairs)))
    for k, v in sorted(reasons.items(), key=lambda kv: -kv[1]):
        print("  fail", v, k)
    print("overlay", OUT)


if __name__ == "__main__":
    main()
