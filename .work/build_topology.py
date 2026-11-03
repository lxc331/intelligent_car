"""Build the final sandbox road network topology from the user's hand sketch + corrections.

Layout (3x3 ring grid, relative coordinates only - NOT metres):
  N0 N1 N2
  N3 N4 N5
  N6 N7 N8
Plus: west stub N9 (left of N3), east stub N10 (right of N5),
      M11 = midpoint of N1-N2, M12 = midpoint of N4-N5, connected by a small path.
Traffic lights: controlled movements listed in TRAFFIC_LIGHTS.
"""
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

OUT = Path(r"D:\项目\intelligent_car-main\DSH_Home\.work\map_sketch")
OUT.mkdir(parents=True, exist_ok=True)
SRC = Path(r"C:\Users\AAA\.dsh\attachments\v1\objects\ff\ff56b3d09d773014cf84fd260a2dd32c9c55cff59cbd8337bff0d62265fd3daf")

XS = [120, 443, 740]
YS = [75, 303, 465]
NODES = {}
for yi, y in enumerate(YS):
    for xi, x in enumerate(XS):
        NODES[yi * 3 + xi] = (x, y)
NODES[11] = (592, 75)     # N1-N2 中点
NODES[12] = (592, 303)    # N4-N5 中点

EDGES = [
    # 外环
    (0, 1), (1, 11), (11, 2), (2, 5), (5, 8), (8, 7), (7, 6), (6, 3), (3, 0),
    # 中间十字
    (1, 4), (4, 7), (3, 4), (4, 12), (12, 5),
    # 小路：N1N2 中点 ↔ N4N5 中点
    (11, 12),
]
TRAFFIC_LIGHTS = [
    (3, 6),
    (4, 1), (4, 3), (4, 7), (4, 5),
    (5, 2), (5, 8),
]

lines = [
    "# 沙盘路网拓扑（用户手绘草图 + 口述修正；坐标为相对值，单位任意，等 SLAM 建图后换算成米）",
    "# 外环闭环、中间十字贯通；N11/N12 之间有小路；左右两侧是绿化带，没有出口",
    "waypoints:",
]
names = {
    0: "西北角", 1: "北中", 2: "东北角",
    3: "西口", 4: "中心", 5: "东口",
    6: "西南角", 7: "南中", 8: "东南角",
    11: "N1N2中点", 12: "N4N5中点",
}
for n in sorted(NODES):
    x, y = NODES[n]
    to = sorted({b if a == n else a for a, b in EDGES if a == n or b == n})
    lines.append("  - id: %d" % n)
    lines.append("    pose: [%.2f, %.2f]" % (x * 0.01, (560 - y) * 0.01))
    lines.append("    name: \"%s\"" % names[n])
    lines.append("    to: [%s]" % ", ".join(str(t) for t in to))
lines.append("")
lines.append("# 红绿灯控制的通行方向（from -> to；左右两侧的灯只控制对向来车，沙盘上没有对应道路）")
lines.append("traffic_lights:")
for a, b in TRAFFIC_LIGHTS:
    lines.append("  - from: %d" % a)
    lines.append("    to: %d" % b)
(OUT / "road_topology.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")

img = Image.open(SRC).convert("RGB")
d = ImageDraw.Draw(img)
font = ImageFont.truetype(r"C:\Windows\Fonts\msyh.ttc", 15)
small = ImageFont.truetype(r"C:\Windows\Fonts\msyh.ttc", 13)
for a, b in EDGES:
    d.line([NODES[a], NODES[b]], fill=(120, 235, 130), width=3)
for a, b in TRAFFIC_LIGHTS:
    ax, ay = NODES[a]
    bx, by = NODES[b]
    t = 0.42
    x, y = ax + (bx - ax) * t, ay + (by - ay) * t
    d.ellipse([x - 6, y - 6, x + 6, y + 6], fill=(240, 60, 60), outline=(40, 0, 0))
for n, (x, y) in NODES.items():
    d.ellipse([x - 12, y - 12, x + 12, y + 12], fill=(255, 214, 90), outline=(20, 20, 20), width=2)
    d.text((x + 15, y - 9), "N%d" % n, font=font, fill=(255, 245, 205))
d.text((8, 6), "路网拓扑：外环闭环 + 中间十字 + N11-N12 小路（红点=本车需要遵守的红绿灯方向；左右两侧的灯只管对向车）", font=font, fill=(255, 220, 120))
d.text((8, 536), "坐标为相对值，不是米制；等那台车通电后用 slam_toolbox 建图，再把节点标到真图上", font=small, fill=(205, 215, 225))
img.save(OUT / "road_topology.png")
print("nodes", len(NODES), "edges", len(EDGES), "lights", len(TRAFFIC_LIGHTS))
print("wrote", OUT / "road_topology.png")
print("wrote", OUT / "road_topology.yaml")
