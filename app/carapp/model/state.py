# -*- coding: utf-8 -*-
"""全局状态模型：所有后台线程写这里，UI 定时读这里。

用一个锁保护字典，读写都是整体替换，避免 UI 读到一半的脏数据。
"""

import threading
import time


class CarState:
    def __init__(self):
        self._lock = threading.Lock()
        self._data = {
            "ws_connected": False,
            "ws_detail": "",
            "cam_connected": False,
            "cam_error": "",
            "battery_mv": None,
            "pose": None,          # (x, y, yaw_deg)
            "node_count": None,
            "cam_fps": 0.0,
            "cam_size": None,      # (w, h)
            "cam_res_name": "",
            "gripper_target": None,
            "servo_targets": {},   # {"pitch": 1300, ...}
            "base_cmd": (0.0, 0.0),  # 当前底盘速度指令 (vx m/s, wz rad/s)
            "last_action": "",
            "updated": 0.0,
        }

    def update(self, **kwargs):
        with self._lock:
            self._data.update(kwargs)
            self._data["updated"] = time.time()

    def snapshot(self):
        with self._lock:
            return dict(self._data)

    def get(self, key, default=None):
        with self._lock:
            return self._data.get(key, default)
