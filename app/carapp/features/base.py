# -*- coding: utf-8 -*-
"""功能插件的公共基类与上下文对象。

加新功能的正确姿势（三步）：
    1. 在 carapp/features/ 下新建一个 xxx.py，写一个继承 Feature 的类
    2. 实现 build(parent) 造界面，需要的话实现 on_tick(state) 刷新
    3. 在 carapp/features/__init__.py 的 FEATURES 列表里加上这个类

主程序会自动把它挂成一个新的选项卡，不需要改 main.py。
"""

import os
import math

from carapp.config import APP_DIR, validate_servo_targets


class AppContext:
    """各功能模块能用到的公共资源。"""

    def __init__(self, app):
        self.app = app

    # ---- 常用对象的快捷访问 ----
    @property
    def cfg(self):
        return self.app.cfg

    @property
    def state(self):
        return self.app.state

    @property
    def bridge(self):
        return self.app.bridge

    @property
    def root(self):
        return self.app.root

    # ---- 服务 ----
    def log(self, msg):
        self.app.log(msg)

    def add_frame_listener(self, cb):
        """注册摄像头帧回调：cb(timestamp, payload_bytes, mode)。"""
        self.app.frame_listeners.append(cb)

    def save_dir(self, sub=""):
        base = self.cfg["camera"].get("save_dir", "捕获")
        path = base if os.path.isabs(base) else os.path.join(APP_DIR, base)
        if sub:
            path = os.path.join(path, sub)
        os.makedirs(path, exist_ok=True)
        return path

    # ---- 舵机控制（所有功能共用这一条通路）----
    def publish_servo(self, targets, duration=0.8):
        """所有按钮、滑杆、预设和序列都必须经过此处的边界校验。"""
        try:
            targets = validate_servo_targets(self.cfg, targets)
            duration = float(duration)
            if not math.isfinite(duration) or not 0.1 <= duration <= 10.0:
                raise ValueError("舵机动作时间需在 0.1–10 秒")
        except (ValueError, TypeError, OverflowError) as exc:
            self.log("拒绝舵机指令：%s" % exc)
            return False
        ids = self.cfg["servo"]["ids"]
        names = self.cfg["servo"]["names"]
        state = []
        for key, pos in targets.items():
            if key not in ids:
                continue
            state.append({"id": [ids[key]], "position": [int(pos)], "offset": [0]})
        if not state:
            return False
        ok = self.bridge.publish("/ros_robot_controller/pwm_servo/set_state",
                                 {"duration": float(duration), "state": state})
        if ok:
            merged = dict(self.state.get("servo_targets") or {})
            merged.update({k: int(v) for k, v in targets.items()})
            self.state.update(servo_targets=merged,
                              gripper_target=merged.get("gripper"),
                              last_action="舵机 %s" % ", ".join(
                                  "%s=%s" % (names.get(k, k), v) for k, v in targets.items()))
            self.log("下发 " + ", ".join("%s=%s" % (names.get(k, k), v) for k, v in targets.items()))
        else:
            self.log("下发失败：ROS 未连接")
        return ok

    # ---- 底盘运动（车轮）----
    def publish_twist(self, vx=0.0, wz=0.0, quiet=False, *, vy=0.0):
        """vx 前进、vy 向左平移（m/s），wz 向左旋转（rad/s）。

        话题默认 /controller/cmd_vel（geometry_msgs/Twist），
        这是车上 controller 节点订阅的那条，和手柄/APP 走同一通路。
        """
        try:
            vx, vy, wz = float(vx), float(vy), float(wz)
            if not all(math.isfinite(v) for v in (vx, vy, wz)):
                raise ValueError("底盘速度必须为有限数值")
        except (ValueError, TypeError, OverflowError) as exc:
            self.log("拒绝底盘指令：%s" % exc)
            return False
        cfg = self.cfg.get("base", {})
        topic = cfg.get("topic", "/controller/cmd_vel")
        ok = self.bridge.publish(topic, {
            "linear": {"x": vx, "y": vy, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": float(wz)},
        })
        if ok:
            self.state.update(base_cmd=(vx, wz), base_vy=vy)
            if not quiet and (vx or vy or wz):
                self.state.update(last_action="底盘 vx=%.2f vy=%.2f wz=%.2f" % (vx, vy, wz))
        return ok

    def stop_base(self):
        """连续发三次零速；网络断开时无法保证指令送达车辆。"""
        for _ in range(3):
            self.publish_twist(0.0, 0.0, quiet=True)
        self.state.update(base_cmd=(0.0, 0.0), base_vy=0.0)


class Feature:
    """所有功能选项卡的基类。"""

    key = "base"
    title = "功能"

    def __init__(self, ctx: AppContext):
        self.ctx = ctx

    def build(self, parent):
        """在 parent 里创建控件。子类必须实现。"""
        raise NotImplementedError

    def on_tick(self, st):
        """主循环每 100ms 调一次，st 是 CarState.snapshot()。"""

    def on_close(self):
        """关闭程序时的清理。"""

    def on_activate(self):
        """进入本选项卡；不自动恢复动作。"""

    def on_deactivate(self):
        """离开本选项卡。"""
