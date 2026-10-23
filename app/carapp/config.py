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

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(APP_DIR, "config")
CONFIG_PATH = os.path.join(CONFIG_DIR, "settings.json")

# 2026-10-23 用户提供的物理范围；相机两端各缩进 150。
HARDWARE_LIMITS = {"pitch": [1000, 2400], "yaw": [500, 2500],
                   "arm": [1200, 2000], "gripper": [1500, 2000]}
SAFE_LIMITS = {"pitch": [1150, 2250], "yaw": [650, 2350],
               "arm": [1200, 2000], "gripper": [1500, 2000]}

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
         "wifi_host": "192.168.149.1", "ssid": "HW-10CCA58E"},
        {"id": "HW-A0857935", "name": "2 号车", "host": "192.168.150.2",
         "wifi_host": "192.168.149.1", "ssid": "HW-A0857935"},
    ],
    "base": {
        "topic": "/controller/cmd_vel",
        # 三档线速度 (m/s)，实车麦轮慢速对位足够
        "speed_levels": {"慢": 0.06, "中": 0.10, "快": 0.15},
        "default_level": "慢",
        # 转向角速度 (rad/s)
        "turn_speed": 0.40,
        # 按住方向键时的发布频率与单次最长时间（安全阀）
        "publish_hz": 10,
        "jog_max_sec": 5.0,
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
    "terminal": {"port": 22, "username": "pi", "auto_connect": True},
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
                cfg = _merge(DEFAULTS, json.load(f))
        except Exception:
            # 配置写坏了也不能让程序起不来，退回默认值
            pass
    else:
        save_config(cfg, path)
    return normalize_connection_config(normalize_servo_config(cfg))


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
    car = cfg["car"]
    selected = next((p for p in profiles if p["id"] == car.get("profile_id")), None)
    if not selected:
        selected = next((p for p in profiles if p.get("host") == car["host"]), None)
    car["profile_id"] = selected["id"] if selected else ""
    car.setdefault("transport", "wifi" if car["host"] == "192.168.149.1" else "wired")
    return cfg


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
