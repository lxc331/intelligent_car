import io

src = io.open(r".work\build_project_wide2.py", encoding="utf-8").read()

old_block = '''old = np.array(Image.open(SRC + "/keepout.pgm").convert("L")) >= 250
rows, cols = np.nonzero(old)
pts = np.column_stack([cols * RES + ORIGIN[0], rows * RES + ORIGIN[1]])
poly = pts[ConvexHull(pts).vertices]
'''

new_block = '''import json as _json
import math as _math

_reg = _json.load(open(r"app/savedMap/sandbox_lidar_20261031/sandbox.json", encoding="utf-8"))["calibration"]["transform"]
_c, _s = _math.cos(_reg["yaw"]), _math.sin(_reg["yaw"])
poly = np.array([(_c * x - _s * y + _reg["x"], _s * x + _c * y + _reg["y"])
                 for x, y in ((0, 0), (2.97, 0), (2.97, 5.40), (0, 5.40))])
'''

assert old_block in src
src = src.replace(old_block, new_block)
src = src.replace("from scipy.spatial import ConvexHull", "")
src = src.replace('OUT = r"app/savedMap/sandbox_nav_w2_20261101"', 'OUT = r"app/savedMap/sandbox_nav_plate_20261101"')
src = src.replace('"id": "sandbox_nav_w2_20261101"', '"id": "sandbox_nav_plate_20261101"')
io.open(r".work\build_project_plate.py", "w", encoding="utf-8", newline="\n").write(src)
print("written")
