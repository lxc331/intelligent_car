# -*- coding: utf-8 -*-
"""配置加载：读取 config/settings.json，缺失项用内置默认值补齐。

设计要点：
- 配置文件是"用户可编辑"的唯一入口，改 IP 只需要改这里
- 文件不存在时自动按默认值生成，保证"复制到别人电脑双击就能跑"
"""

import copy
import json
import os
import math

from carapp.core.connection_profiles import normalize_profiles

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(APP_DIR, "config")
CONFIG_PATH = os.path.join(CONFIG_DIR, "settings.json")

# 2026-10-23 用户提供的物理范围；相机两端各缩进 150。
HARDWARE_LIMITS = {"pitch": [1000, 2400], "yaw": [500, 2500],
                   "arm": [1200, 2000], "gripper": [1500, 2000]}
SAFE_LIMITS = {"pitch": [1150, 2250], "yaw": [650, 2350],
               "arm": [1200, 2000], "gripper": [1500, 2000]}

# 逐车标定：装配方向、限位、动作值、预设和红绿灯视角。
# 方向用 signs 表示「数值增大的方向」：pitch +1 = 更仰视、-1 = 更俯视；
# yaw +1 = 更左、-1 = 更右。标定值覆盖 settings.json 里的同名项（切车生效），
# 之后仍会过一次 normalize_servo_config，越界值被夹回限位。
CALIBRATIONS = {
    # 1 号车：装配方向与内置默认一致（2026-10-23 用户实机核对）。
    "HW-10CCA58E": {
        "servo": {
            "limits": {"pitch": [1150, 2250], "yaw": [650, 2350],
                       "arm": [1200, 2000], "gripper": [1500, 2000]},
            # 用户实机确认：1200 抬起，1700 降下；1300 俯视夹取区，1500 前视。
            "actions": {"open": 2000, "close": 1500, "half_open": 1750,
                        "arm_up": 1200, "arm_down": 1700,
                        "look_down": 1300, "look_forward": 1500},
            "presets": {
                "待机（张爪）": {"pitch": 1300, "yaw": 1500, "arm": 1700, "gripper": 2000},
                "俯视观察": {"pitch": 1300, "yaw": 1500, "arm": 1700, "gripper": 2000},
                "夹取准备": {"pitch": 1300, "yaw": 1500, "arm": 1700, "gripper": 2000},
                "抬起持物": {"pitch": 1300, "yaw": 1500, "arm": 1200, "gripper": 1500},
            },
            "signs": {"pitch": 1, "yaw": 1},
        },
        "traffic_light": {"dip_pitch": 1450, "confirm_pitch": 1720, "restore_pitch": 1500},
        # 2026-10-29 实测：指令 0.06 m/s 实际 0.112 m/s，比例 1.87（车头前路障 + 雷达当尺）
        "junction": {"speed_scale": 1.87},
    },
    # 2 号车：2026-10-28 实车逐项重测（摄像头取证 + 用户现场确认）。
    # 俯仰方向与 1 号车相反：1150 仰视…1800 俯视，物理上限只到 1800。
    # 云台水平限位不变，但数值越大越左。
    "HW-A0857935": {
        "servo": {
            "limits": {"pitch": [1150, 1800], "yaw": [650, 2350],
                       "arm": [1250, 2000], "gripper": [1500, 2000]},
            "actions": {"open": 2000, "close": 1500, "half_open": 1750,
                        "arm_up": 1300, "arm_down": 1950,
                        "look_down": 1600, "look_forward": 1450},
            "presets": {
                "待机（张爪）": {"pitch": 1600, "yaw": 1500, "arm": 1900, "gripper": 2000},
                "俯视观察": {"pitch": 1750, "yaw": 1500, "arm": 1900, "gripper": 2000},
                "夹取准备": {"pitch": 1780, "yaw": 1500, "arm": 1950, "gripper": 2000},
                "抬起持物": {"pitch": 1600, "yaw": 1500, "arm": 1300, "gripper": 1500},
            },
            "signs": {"pitch": -1, "yaw": 1},
        },
        # 实测：1450 平视（灯在画面上部）、1600 低头能看到停止线、
        # 1380 抬头时红灯落在画面中部（检出 0.59）。
        "traffic_light": {"dip_pitch": 1600, "confirm_pitch": 1380, "restore_pitch": 1450},
        # 2026-10-29 实测：指令 0.06 m/s 实际约 0.076–0.079 m/s，比例 1.30；
        # 行号-距离用俯视 1600 视角标定（该车相机比 1 号车低、倾角大）
        "junction": {"speed_scale": 1.30, "row_slope": -0.001733, "row_intercept": 1.123,
                     "row_trigger": 396., "finish_travel": .03, "track_pitch": 1450},
    },
}

DEFAULTS = {
    "car": {
        "host": "192.168.150.3",
        "http_port": 8080,
        "ws_port": 9090,
        "subnet_prefix": "192.168.150",
        # 两条接入路径：有线直连 192.168.150.x、车自身 WiFi 热点 192.168.149.x
        "subnet_prefixes": ["192.168.150", "192.168.149"],
    },
    # 多车档案：顶部下拉一键切换。host 是"当前使用"的地址，由切换动作更新。
    # 按两台车采集文档记录的 NetworkManager 持久地址配置。
    "cars": [
        {"id": "HW-10CCA58E", "name": "1 号车", "host": "192.168.150.3",
         "wifi_host": "192.168.149.1", "ssid": "HW-10CCA58E", "sta_host": "", "sta_ssid": ""},
        {"id": "HW-A0857935", "name": "2 号车", "host": "192.168.150.2",
         "wifi_host": "192.168.149.1", "ssid": "HW-A0857935", "sta_host": "", "sta_ssid": ""},
    ],
    "base": {
        "topic": "/controller/cmd_vel",
        # 三档线速度 (m/s)，实车麦轮慢速对位足够
        "speed_levels": {"慢": 0.06, "中": 0.10, "快": 0.15},
        "default_level": "慢",
        # 转向角速度 (rad/s)
        "turn_speed": 0.40,
        # 按住方向键时的发布频率
        "publish_hz": 10,
        # 一键夹取序列里默认前进/后退时长（秒）
        "pick_forward_sec": 0.6,
        "pick_backward_sec": 0.4,
    },
    "camera": {
        "default_topic": "/image_raw",
        "topics": [
            "/image_raw",
            "/color_track_pick/image_result",
            "/line_following/image_result",
            "/object_tracking/image_result",
            "/llm_control/image_result",
        ],
        "max_width": 860,
        "save_dir": "捕获",
    },
    "servo": {
        "ids": {"pitch": 1, "yaw": 2, "arm": 3, "gripper": 4},
        "names": {
            "pitch": "云台俯仰",
            "yaw": "云台水平",
            "arm": "升降臂",
            "gripper": "夹爪",
        },
        "limits": copy.deepcopy(SAFE_LIMITS),
        # 数值增大的方向：pitch +1 = 更仰视，yaw +1 = 更左。逐车标定会覆盖。
        "signs": {"pitch": 1, "yaw": 1},
        "actions": {"open": 2000, "close": 1500, "half_open": 1750,
                    # 用户实机确认：1200 抬起，1700 降下。
                    "arm_up": 1200, "arm_down": 1700,
                    "look_down": 1300, "look_forward": 1500},
        "presets": {
            # 2026-10-23 本次 2 号车稳定画面核对：1300 俯视夹取区，1500 前视。
            # 旧记录方向相反；另一台车需单独校准，不根据注释推断方向。
            "待机（张爪）": {"pitch": 1300, "yaw": 1500, "arm": 1700, "gripper": 2000},
            "俯视观察": {"pitch": 1300, "yaw": 1500, "arm": 1700, "gripper": 2000},
            "夹取准备": {"pitch": 1300, "yaw": 1500, "arm": 1700, "gripper": 2000},
            "抬起持物": {"pitch": 1300, "yaw": 1500, "arm": 1200, "gripper": 1500},
        },
    },
    "tracking_pick": {
        "color": "red", "max_speed": .06, "min_speed": .02,
        "max_turn": .30, "turn_gain": 1.2, "control_hz": 10,
        "x_tolerance": .035, "y_tolerance": .025, "overshoot_margin": .08,
        "stable_frames": 3, "frame_timeout": .6, "lost_timeout": 2.,
        "odom_timeout": .8, "max_seconds": 45., "max_distance": .8,
        "calibrations": {},
    },
    "cruise": {"detection_distance": .60, "speed": .06, "mode": "avoid",
               "scan_angle": 90., "lidar_type": "standard"},
    "terminal": {"port": 22, "username": "pi", "auto_connect": True},
    "mapping": {"remote_dir": "/home/ubuntu/ros2_ws/src/slam/maps",
                "scope": "container", "serialize": True},
    "navigation": {"speed": .06},
    "traffic_light": {
        # 全帧之外再对七块区域裁切放大后推理：640x480 里一盏灯只有十几到
        # 三十像素，整帧缩到 320 后模型看不到，而模型对取景非常敏感——同一
        # 盏灯换个位置分数就从 0.8 掉到 0.05。按「车离路口多远」分三条带：
        # 远处灯在画面上部（顶部三块 45% 宽、30% 高，y 0–30%）、1–2 m 时
        # 在中部（两块 45%×40%，y 28%–68%）、停车位时灯在头顶只露出画面
        # 下沿（两块 55%×28%，y 72%–100%）。实测十张实车帧该组合每帧都出红框，
        # 最差帧置信度 0.43（单条带各自最差 0.24–0.12 甚至 0）。
        "confidence": .5,
        "zoom": [[.15, 0., .45, .3], [.35, 0., .45, .3], [.55, 0., .45, .3],
                 [.25, .28, .45, .4], [.45, .28, .45, .4],
                 [0., .72, .55, .28], [.45, .72, .55, .28]],
        # 放大区域用更低阈值：实测同一盏灯分数在 0.40–0.64 抖动
        "zoom_confidence": .35,
        # 放大区域的候选门限：分数落在 0.06–0.35 的红框不直接算检出，而是
        # 以框中心裁一块带上下文的正方形再推理一次（换一种取景复检）。模型
        # 对取景极其敏感，同一盏灯换个裁切就从 0.5 掉到 0；换车后顶部灯带
        # 分数只有 0.08，走这条路复检能回到 0.41–0.47。
        "candidate_confidence": .06,
        # 红灯停车面积门限（折算到 640x480 的像素²）。厂家原值 700 是车停在
        # 近处、灯约 28×28 时定的；上述放大区在 1–2 m 处只剩 270–600 px²，
        # 所以默认降到 450，等实车标定后再改 settings.json。
        "red_area": 450.,
        # 红灯框的高宽比窗口（h/w）。实测十张实车帧的灯框都在 0.88–1.05
        # 之间（圆灯近方形），而红白锥桶、红屋顶明显更细长，用宽松窗口
        # 0.6–1.6 去掉这类误检；设成空列表即关闭该过滤。
        "red_aspect": [.6, 1.6],
        # 连续 3 帧看到红灯才判停（0.5 s）；短暂漏检保持 1.5 s，避免红灯
        # LED 闪烁或云台压下确认斑马线（最长 1.2 s 看不到灯）时误松刹车。
        # 红灯是独立证据：先看红灯，再让云台俯视确认斑马线，所以车已经停在
        # 停止线、画面里没有斑马线时，红灯照样能判停。
        "red_frames": 3,
        "red_hold": 1.5,
        # 看到红灯后先把云台压到 1450 俯视确认斑马线（实测 pitch 1450 时最近
        # 斑马线中心 y≈438、置信度 0.96），确认到就转入仰视监控红灯。
        "dip_pitch": 1450,
        "confirm_dip": 1.2,
        # 停在斑马线前时红绿灯几乎在头顶，前视画面里没有灯：确认斑马线后把
        # 云台仰起（实测 pitch 1720 时红灯在画面下方 45%×28% 块里，置信度
        # 0.73、面积 528 px²），离开路口再复位到前视。
        "confirm_pitch": 1720,
        "restore_pitch": 1500,
        "confirm_delay": 1.4,
        # 仰视最长保持时间：红灯可见时一直保持仰视（切回前视会让红灯消失、
        # 看起来像放行），只有红灯消失或始终没确认到红灯才按这个时长回前视
        "confirm_hold": 8.,
        # 回前视后多久再抬头复核（红灯消失后若仍在路口，用它循环看灯）
        "confirm_retry": .8,
        # 换视角后先丢掉这一段画面（云台还在动、图像不可信），只吃稳定后的帧
        "confirm_settle": .3,
        # 绿灯确认起步（默认关）：0 = 红灯消失即放行（现在的行为）；>0 = 必须再
        # 连续看到这么多帧绿灯才松刹车。默认关的原因：实车仰视 1380 / 1720 时
        # 右侧竖直绿灯目前召回是 0，开了会一直停着。等测到绿灯样本再打开。
        "green_area": 300.,
        "release_green_frames": 0,
        # 等绿灯的兜底超时：红灯消失后还没确认到绿灯，超过这个秒数自动放行，
        # 避免绿灯漏检把车永久钉在停止线前。
        "release_timeout": 12.,
    },
    # 路口停车/起步的驾驶层框架参数（纯逻辑，尚未接车轮）。
    # 明天实测「车到停止线的距离」和实际速度后，把这些值标定成真值即可用。
    "junction": {
        "slow_distance": 3.0,     # 离停止线多远开始限速接近
        "stop_distance": .9,      # 进入停止线前的低速区
        "brake_distance": .35,    # 离停止线这么近就开始刹停
        "approach_speed": .12,    # 接近段速度上限（m/s）
        "slow_speed": .08,        # 减速段速度上限
        "crawl_speed": .04,       # 爬行段速度上限
        "go_speed": .10,          # 绿灯起步速度上限
        "crawl_timeout": 8.0,     # 爬行最长时长，超时保持停车
        # 2026-10-29 实车标定（1 号车、前视 1500）：近处斑马线下沿像素行 ↔ 离停止线距离。
        # 三点拟合 distance = row_intercept + row_slope * row（0.53/0.42/0.31 m → 378/418/476）。
        "row_slope": -.002220,    # 每 1 像素对应多少米（负值：行号越大离车越近）
        "row_intercept": 1.36125,  # 行号为 0 时的外推距离（仅用于同一条直线）
        "row_trigger": 470.,      # 触发行号 ≈ 0.32 m，之后改由里程计收尾
        "finish_travel": .22,     # 触发后按真实米数再走这么多就停车
        # 厂家 /odom 是用 cmd_vel 积分出来的假里程计（odom_publisher_node 用 linear_x*dt），
        # 2026-10-29 用车前路障雷达实测：指令 0.06 m/s 实际约 0.112 m/s，比例 1.87。
        "speed_scale": 1.87,      # 真实速度 / 指令速度，用于把里程换算成真实米数
    },
    "status": {"poll_interval": 1.0, "nodes_interval": 6.0, "log_lines": 300},
}


def _merge(base, override):
    """把 override 深度合并到 base 的副本上（列表整体替换）。"""
    out = copy.deepcopy(base)
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def load_config(path=CONFIG_PATH):
    cfg = copy.deepcopy(DEFAULTS)
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                cfg = _merge(DEFAULTS, migrate_cruise_config(json.load(f)))
        except Exception:
            # 配置写坏了也不能让程序起不来，退回默认值
            pass
    else:
        save_config(cfg, path)
    return normalize_connection_config(normalize_servo_config(apply_calibration(cfg)))


def apply_calibration(cfg):
    """按当前车辆档案套用逐车标定（方向/限位/动作/预设/红绿灯视角）。

    标定表是硬编码的实测结果：两台车的云台装配方向相反，用同一套数值会让
    云台往反方向顶到机械限位，所以这里覆盖 settings.json 的同名项，而不是
    只做默认值填充。未知车辆保持配置文件里的值。
    """
    profile = (cfg.get("car") or {}).get("profile_id") or ""
    calibration = CALIBRATIONS.get(profile)
    if not calibration:
        return cfg
    servo = cfg.setdefault("servo", {})
    values = calibration.get("servo") or {}
    for key in ("limits", "actions", "signs"):
        if isinstance(values.get(key), dict):
            servo.setdefault(key, {})
            servo[key].update(copy.deepcopy(values[key]))
    if isinstance(values.get("presets"), dict):
        servo["presets"] = copy.deepcopy(values["presets"])
    light = cfg.setdefault("traffic_light", {})
    for key, value in (calibration.get("traffic_light") or {}).items():
        light[key] = value
    junction = cfg.setdefault("junction", {})
    for key, value in (calibration.get("junction") or {}).items():
        junction[key] = value
    return cfg


def migrate_cruise_config(cfg):
    """Move prior cruise settings to the vendor threshold without touching other features."""
    cruise = cfg.get("cruise")
    if isinstance(cruise, dict):
        value = cruise.get("detection_distance", cruise.get("front_back_distance", .60))
        try:
            distance = float(value)
            if not math.isfinite(distance):
                raise ValueError("nonfinite distance")
        except (ValueError, TypeError, OverflowError):
            distance = DEFAULTS["cruise"]["detection_distance"]
        cruise["detection_distance"] = max(.30, min(1.50, distance))
        cruise.pop("front_back_distance", None)
        cruise.pop("side_distance", None)
        try:
            speed = float(cruise.get("speed", .06))
            if not math.isfinite(speed) or not .02 <= speed <= .15:
                raise ValueError("invalid speed")
        except (ValueError, TypeError, OverflowError):
            speed = DEFAULTS["cruise"]["speed"]
        cruise["speed"] = speed
        if cruise.get("mode", "avoid") not in ("avoid", "follow", "guard"):
            cruise["mode"] = "avoid"
    return cfg


def normalize_connection_config(cfg):
    """补全旧的两车档案；共享热点地址不用于推断车辆身份。"""
    profiles = cfg.get("cars") or copy.deepcopy(DEFAULTS["cars"])
    for index, profile in enumerate(profiles):
        known = next((p for p in DEFAULTS["cars"]
                      if p["host"] == profile.get("host") or p["id"] == profile.get("id")), None)
        if known:
            for key, value in known.items():
                profile.setdefault(key, value)
        profile.setdefault("id", "car-%d" % (index + 1))
        profile.setdefault("name", profile["id"])
    cfg["cars"] = profiles
    return normalize_profiles(cfg)


def normalize_servo_config(cfg):
    """旧配置迁移到当前安全包络，旧预设也必须经过同一限位。"""
    servo = cfg["servo"]
    for key, (safe_lo, safe_hi) in SAFE_LIMITS.items():
        requested = servo["limits"].get(key, [safe_lo, safe_hi])
        lo, hi = max(safe_lo, int(requested[0])), min(safe_hi, int(requested[1]))
        servo["limits"][key] = [lo, hi] if lo <= hi else [safe_lo, safe_hi]
    action_keys = {"open": "gripper", "close": "gripper", "half_open": "gripper",
                   "arm_up": "arm", "arm_down": "arm",
                   "look_down": "pitch", "look_forward": "pitch"}
    for name, key in action_keys.items():
        lo, hi = servo["limits"][key]
        servo["actions"][name] = max(lo, min(hi, int(servo["actions"][name])))
    for pose in servo["presets"].values():
        for key, value in list(pose.items()):
            if key in SAFE_LIMITS:
                lo, hi = servo["limits"][key]
                pose[key] = max(lo, min(hi, int(value)))
    return cfg


def validate_servo_targets(cfg, targets):
    """发送前原子校验；拒绝整个越界请求，不偷偷移动到另一个位置。"""
    if not targets:
        raise ValueError("没有舵机目标")
    result = {}
    for key, value in targets.items():
        if key not in SAFE_LIMITS or key not in cfg["servo"]["ids"]:
            raise ValueError("未知舵机：%s" % key)
        number = float(value)
        lo, hi = cfg["servo"]["limits"][key]
        safe_lo, safe_hi = SAFE_LIMITS[key]
        lo, hi = max(lo, safe_lo), min(hi, safe_hi)
        if not math.isfinite(number) or not number.is_integer() or not lo <= number <= hi:
            raise ValueError("%s=%s 超出允许范围 %d–%d" % (key, value, lo, hi))
        result[key] = int(number)
    return result


def save_config(cfg, path=CONFIG_PATH):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, ensure_ascii=False, indent=2)
