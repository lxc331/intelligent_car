# -*- coding: utf-8 -*-
"""Trinary map comparison: 0=occupied, 254=free, 205=unknown (Cartographer / map_saver)."""
import re
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(r"D:\项目\intelligent_car-main")
PAIRS = {
    "new_cartographer": (ROOT / ".work/maps/lowbat_20261031/map.pgm",
                         ROOT / ".work/maps/lowbat_20261031/map.yaml"),
    "old_vendor_slam": (ROOT / "app/savedMap/sandbox_half/map.pgm",
                        ROOT / "app/savedMap/sandbox_half/map.yaml"),
}


def load(pgm, yaml_path):
    img = np.array(Image.open(pgm).convert("L"), dtype=np.uint8)
    text = Path(yaml_path).read_text(encoding="utf-8", errors="replace")
    res = float(re.search(r"resolution:\s*([0-9.]+)", text).group(1))
    return img, res


def report(name, img, res):
    counts = Counter(img.ravel().tolist())
    occ = img < 50
    unknown = (img >= 50) & (img < 250)
    free = img >= 250
    ys, xs = np.nonzero(occ)
    print(f"{name}: {img.shape[1]}x{img.shape[0]} @{res:.2f} => "
          f"{img.shape[1]*res:.2f} x {img.shape[0]*res:.2f} m")
    print("   values:", dict(sorted(counts.items(), key=lambda kv: -kv[1])[:6]))
    print(f"   occupied {occ.mean():.3f}  free {free.mean():.3f}  unknown {unknown.mean():.3f}")
    if xs.size:
        print(f"   occupied bbox cells x[{xs.min()},{xs.max()}] y[{ys.min()},{ys.max()}]")
    return img


painted = {}
for name, (pgm, yml) in PAIRS.items():
    img, res = load(pgm, yml)
    report(name, img, res)
    p = np.full(img.shape, 128, dtype=np.uint8)
    p[img >= 250] = 255
    p[(img >= 50) & (img < 250)] = 160
    p[img < 50] = 0
    painted[name] = p

h = max(p.shape[0] for p in painted.values())
w = sum(p.shape[1] for p in painted.values()) + 20
canvas = np.full((h, w), 60, dtype=np.uint8)
x = 0
for name in ("new_cartographer", "old_vendor_slam"):
    p = painted[name]
    canvas[:p.shape[0], x:x + p.shape[1]] = p
    x += p.shape[1] + 20
out = ROOT / ".work/maps/lowbat_20261031/compare_trinary.png"
Image.fromarray(canvas).resize((w * 2, h * 2), Image.NEAREST).save(out)
print("wrote", out)
