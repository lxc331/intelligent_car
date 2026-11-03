"""Read-only check: is the tracking-pick calibration usable for the selected car?"""
import sys
sys.path.insert(0, r"D:\项目\intelligent_car-main\app")
from carapp import config as car_config
from carapp.core.visual_pick import calibration_problem

cfg = car_config.load_config()
car = cfg["car"]
tcfg = cfg["tracking_pick"]


def pose():
    actions = cfg["servo"]["actions"]
    p = {"pitch": actions["look_down"], "yaw": 1500, "arm": actions["arm_down"]}
    profile = car.get("profile_id")
    override = tcfg.get("preparation_poses", {}).get(profile, {}) if profile else {}
    p.update({k: override[k] for k in p if k in override})
    return car_config.validate_servo_targets(cfg, p)


key = (car.get("profile_id") or "manual") + ":" + car["host"] + ":" + tcfg["color"]
print("车辆: %s  host=%s  transport=%s" % (car.get("profile_id"), car["host"], car.get("transport")))
print("标定键: %s" % key)
print("准备姿态: %s" % pose())
cal = tcfg["calibrations"].get(key)
if cal is None and car.get("transport") != "wifi":
    cal = tcfg["calibrations"].get(car["host"] + ":" + tcfg["color"])
problem = calibration_problem(cal, car["host"], tcfg["color"], pose())
if problem:
    print("标定不可用: %s" % problem)
    print("已有标定键: %s" % ", ".join(tcfg["calibrations"]))
    raise SystemExit(1)
print("标定可用: x=%.3f y=%.3f width=%.3f 建立时间=%s" % (cal["x"], cal["y"], cal["width"], cal.get("created")))
notes = []
if not .30 <= cal["x"] <= .70:
    notes.append("x 偏离画面中心较多，目标可能没放在两爪正中")
if not .50 <= cal["y"] <= .98:
    notes.append("y 不在近景范围，夹取线可能太远")
if not .06 <= cal["width"] <= .60:
    notes.append("width 异常，目标可能太小/太大")
print("几何检查: %s" % ("；".join(notes) if notes else "OK"))
print("一键追踪夹取前置条件满足，可以测车了")
