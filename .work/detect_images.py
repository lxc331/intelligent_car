# -*- coding: utf-8 -*-
"""Run the App traffic detector on saved images and show every raw row >= 0.05."""
import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "app"))

spec = importlib.util.spec_from_file_location("probe", ROOT / ".work" / "live_traffic_raw_probe.py")
probe = importlib.util.module_from_spec(spec)
_images = sys.argv[1:]
sys.argv = sys.argv[:1]
spec.loader.exec_module(probe)
sys.argv = sys.argv[:1] + _images

run = probe.load_runner()
for name in sys.argv[1:]:
    path = Path(name)
    items, size = run(path.read_bytes())
    print("%s %dx%d" % (path.name, size[0], size[1]))
    for item in items:
        print("   %-9s %.3f box=%s area=%d" % (item.label, item.score, item.box,
                                               item.box[2] * item.box[3]))
    if not items:
        print("   (nothing above 0.05)")
