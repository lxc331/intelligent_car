# -*- coding: utf-8 -*-
"""Fix README mapping-speed wording; also repair a backtick-mangled link."""
import io

PATH = "README.md"
lines = io.open(PATH, encoding="utf-8").read().splitlines()

NEW_START = (
    "建图遥控速度在「建图」页控制栏里直接改（`建图速度（m/s）` 输入框，旁边 `0.03/0.04/0.05/0.06/0.08` 是快捷键），"
    "改完自动写回 `settings.json` 的 `mapping.speed`，重启后仍是这个值。"
    "`app/run.bat` 里的 `CARAPP_MAP_SPEED` 只在还没保存过速度时作为初始值"
    "（脚本预设 0.05，程序默认 0.04），范围始终 0.02–0.10。"
)
NEW_MAPPING = (
    "2. 点击“开始建图”。程序只启建图节点，避免重新启动底盘、雷达和摄像头；若已有 `/slam_toolbox`，"
    "则接入已有建图。地图、定位、雷达和画面就绪后可低速遥控。默认 0.04 m/s"
    "（首次启动可用 `app/run.bat` 的 `CARAPP_MAP_SPEED` 给初值，脚本预设 0.05）；"
    "页面里可直接改并自动保存，范围 0.02–0.10 m/s。"
)

changed = []
for index, line in enumerate(lines):
    if "CARAPP_MAP_SPEED" in line and line.startswith("建图遥控的默认速度由环境变量"):
        lines[index] = NEW_START
        changed.append(index + 1)
    elif "\x07" in line and "开始建图" in line:
        lines[index] = NEW_MAPPING
        changed.append(index + 1)

bad = [(i + 1, repr(line)) for i, line in enumerate(lines) if "\x07" in line]
assert not bad, bad
assert len(changed) == 2, changed
io.open(PATH, "w", encoding="utf-8", newline="\n").write("\n".join(lines) + "\n")
print("fixed lines", changed)
