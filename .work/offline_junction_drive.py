"""离线演练：用已录的实车画面跑「红灯 → 俯视确认 → 仰视看灯 → 停车」全链路。

不连车也能跑：只读 .work/traffic_light_live 下已保存的帧，按文件顺序当时间轴。
用法：
    py -3 .work/offline_junction_drive.py

输出每一帧的场景判定、云台动作、以及（假想测距）下的驾驶层速度上限，
用于明天接上真实测距/车轮前，先确认状态机顺序对。
"""
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "app"))

from carapp.config import load_config                                  # noqa: E402
from carapp.core.junction_drive import StopLineDrive                  # noqa: E402
from carapp.features.traffic_light import _build, _build_look          # noqa: E402

LIVE = ROOT / ".work" / "traffic_light_live"
# (文件, 假想离停止线距离 m)。距离是占位值，明天换成雷达/视觉测距。
# red_frames=3，所以每张帧重复 3 次，模拟连续画面。
FRAMES = [
    ("car2_1500.jpg", 5.0), ("car2_1500.jpg", 4.4), ("car2_1500.jpg", 3.8),
    ("pitch_1600.jpg", 2.6), ("pitch_1600.jpg", 2.0), ("pitch_1600.jpg", 1.4),
    ("stop_lookup_1720.jpg", .9), ("stop_lookup_1720.jpg", .6), ("stop_lookup_1720.jpg", .3),
]


def main():
    cfg = load_config()
    detector, scene = _build(cfg)
    views = _build_look(cfg)
    drive = StopLineDrive.from_config(cfg)
    now = 100.0
    print("view   red  active  blocked  phase   speed  note")
    for name, distance in FRAMES:
        path = LIVE / name
        if not path.exists():
            print("missing", path)
            continue
        now += 0.3
        result = detector.detect(path.read_bytes(), "jpeg")
        scene.observe(result.objects, result.image_size, now)
        snapshot = scene.snapshot()
        action, notice = views.step(bool(snapshot["red_pending"]),
                                    bool(snapshot["crosswalk_active"]), now)
        command = drive.step(snapshot, distance, now)
        pitch = {views.LIFT: views.pitch, views.DIP: views.dip,
                 views.RESTORE: views.restore}.get(action)
        print("%-22s %-4s %-6s %-8s %-7s %-6s %s" % (
            name, snapshot["red_pending"], snapshot["crosswalk_active"],
            scene.red_required, command.phase,
            "--" if command.speed is None else "%.2f" % command.speed,
            notice or command.note))
        if pitch is not None:
            print("    -> 云台 pitch %d" % pitch)
    print("\nscene:", scene.snapshot()["scene"], "| view:", views.view,
          "| drive phase:", drive.phase)


if __name__ == "__main__":
    main()
