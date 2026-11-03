# -*- coding: utf-8 -*-
"""Compare two saved maps (PGM + YAML): metrics table + side-by-side trinary PNG.

    python .work/compare_maps.py DIR_A DIR_B [OUT_PNG]
"""
import pathlib
import sys

import numpy as np
import yaml
from PIL import Image


def load(directory):
    directory = pathlib.Path(directory)
    meta = yaml.safe_load((directory / "map.yaml").read_text(encoding="utf-8"))
    image = Image.open(directory / "map.pgm").convert("L")
    data = np.array(image)
    # pgm semantics from map_server: 0 occupied, 255 free, 128 unknown
    occupied = data < 100
    unknown = (data >= 100) & (data < 200)
    free = data >= 200
    return {
        "dir": directory.name,
        "size": image.size,
        "res": float(meta["resolution"]),
        "meters": (image.size[0] * float(meta["resolution"]), image.size[1] * float(meta["resolution"])),
        "occupied": occupied.mean() * 100,
        "free": free.mean() * 100,
        "unknown": unknown.mean() * 100,
        "data": data,
    }


def paint(m):
    rgb = np.zeros((m["data"].shape[0], m["data"].shape[1], 3), dtype=np.uint8)
    rgb[m["data"] < 100] = (0, 0, 0)
    rgb[(m["data"] >= 100) & (m["data"] < 200)] = (150, 150, 150)
    rgb[m["data"] >= 200] = (255, 255, 255)
    return rgb


def main():
    a = load(sys.argv[1])
    b = load(sys.argv[2])
    out = pathlib.Path(sys.argv[3]) if len(sys.argv) > 3 else pathlib.Path(sys.argv[1]) / "compare_two.png"
    print("%-14s %-22s %-8s %-6s %-6s %-6s" % ("map", "size(px) @m/px", "meters", "occ%", "free%", "unk%"))
    for m in (a, b):
        print("%-14s %-22s %-8s %-6.2f %-6.1f %-6.1f" % (
            m["dir"], "%dx%d @%.2f" % (m["size"][0], m["size"][1], m["res"]),
            "%.2fx%.2f" % m["meters"], m["occupied"], m["free"], m["unknown"]))
    h = max(a["data"].shape[0], b["data"].shape[0])
    w = a["data"].shape[1] + b["data"].shape[1] + 12
    canvas = np.full((h, w, 3), 70, dtype=np.uint8)
    for i, m in enumerate((a, b)):
        rgb = paint(m)
        x = 0 if i == 0 else a["data"].shape[1] + 12
        canvas[:rgb.shape[0], x:x + rgb.shape[1]] = rgb
    Image.fromarray(canvas).save(out)
    print("wrote", out)


if __name__ == "__main__":
    main()
