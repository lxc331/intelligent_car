# -*- coding: utf-8 -*-
"""功能：夹取（含底盘配合 + 底部实时画面）。

页面结构：
    左上  车轮：按住方向键走、松开停（速度三档）
    左中  夹取动作：张爪 / 合爪 / 抬臂 / 降臂 / 云台俯视 / 云台平视
    左下  预设姿态 + 一键夹取序列（可带前进 / 后退）
    右上  四路舵机滑杆微调
    底部  实时画面（CameraPanel），边夹边看

所有指令：
    舵机 → /ros_robot_controller/pwm_servo/set_state
    车轮 → /controller/cmd_vel   (geometry_msgs/Twist)
与在车上直接 ros2 topic pub 等价，不往车上写任何文件。
"""

import threading
import time
import tkinter as tk

from carapp.features.base import Feature
from carapp.ui import theme
from carapp.ui.camerapanel import CameraPanel
from carapp.features.tracking_pick import TrackingPickControls


class GripperFeature(Feature):
    key = "gripper"
    title = "夹取 + 实时画面"

    def __init__(self, ctx):
        super().__init__(ctx)
        self.sliders = {}
        self.var_labels = {}
        self._busy = False          # 舵机动作 / 序列占用中
        self._abort = False         # 序列中断标志
        self._jog_evt = None        # 当前点动线程停止事件
        self._jog_dir = None
        self._motion_thread = None
        self.tracking = None

    # ---------- 界面 ----------
    def build(self, parent):
        cfg = self.ctx.cfg["servo"]
        self.ids = cfg["ids"]
        self.names = cfg["names"]
        self.limits = cfg["limits"]
        self.base_cfg = self.ctx.cfg.get("base", {})

        top = tk.Frame(parent, bg=theme.BG)
        top.pack(fill="x")

        left = tk.Frame(top, bg=theme.BG)
        left.pack(side="left", fill="both", expand=True)

        right = tk.Frame(top, bg=theme.BG, width=410)
        right.pack(side="right", fill="y", padx=(6, 0))
        right.pack_propagate(False)

        self._build_wheels(left)
        self._build_actions(left)
        self._build_presets(left)
        self.tracking = TrackingPickControls(left, self)
        self._build_sliders(right)

        self.cam = CameraPanel(parent, self.ctx)
        self.cam.pack(fill="both", expand=True, padx=0, pady=(6, 6))

    # ---- 车轮 ----
    def _build_wheels(self, parent):
        box = self._section(parent, "车轮（按住方向键走，松开即停；单次最长 5 秒）")

        tk.Label(box, text="速度", bg=theme.PANEL, fg=theme.TEXT_DIM,
                 font=theme.FONT_SMALL).pack(side="left", padx=(0, 4))

        levels = list(self.base_cfg.get("speed_levels", {"慢": 0.06, "中": 0.10, "快": 0.15}).keys())
        default = self.base_cfg.get("default_level", "慢")
        self.var_level = tk.StringVar(value=default if default in levels else levels[0])
        om = tk.OptionMenu(box, self.var_level, *levels)
        om.config(bg=theme.PANEL_2, fg=theme.TEXT, relief="flat", bd=0,
                  highlightthickness=1, highlightbackground=theme.BORDER,
                  font=theme.FONT_SMALL, width=4, anchor="center",
                  activebackground=theme.BORDER, activeforeground=theme.TEXT)
        om["menu"].config(bg=theme.PANEL_2, fg=theme.TEXT,
                          activebackground=theme.BORDER, font=theme.FONT_SMALL)
        om.pack(side="left", padx=(0, 10))

        self.hold_btns = []
        for text, vx, wz in (("↑ 前进", 1, 0), ("↓ 后退", -1, 0),
                             ("← 左转", 0, 1), ("→ 右转", 0, -1)):
            b = self._hold_btn(box, text, vx, wz)
            self.hold_btns.append(b)

        self._btn(box, "■ 停止", self.stop_all, fg=theme.ERR)

        tk.Label(box, text="点动", bg=theme.PANEL, fg=theme.TEXT_DIM,
                 font=theme.FONT_SMALL).pack(side="left", padx=(12, 4))
        self._btn(box, "前 0.3s", lambda: self.nudge(1, 0, 0.3))
        self._btn(box, "后 0.3s", lambda: self.nudge(-1, 0, 0.3))

    def _hold_btn(self, parent, text, vx, wz):
        b = tk.Button(parent, text=text, bg=theme.PANEL_2, fg=theme.TEXT,
                      relief="flat", font=theme.FONT_SMALL,
                      activebackground=theme.ACCENT, activeforeground="#101010",
                      padx=10, pady=3)
        b.pack(side="left", padx=3, pady=2)
        b.bind("<ButtonPress-1>", lambda e, a=vx, c=wz: self.jog(a, c))
        b.bind("<ButtonRelease-1>", lambda e: self.stop_wheels())
        b.bind("<Leave>", lambda e: self.stop_wheels())
        return b

    # ---- 夹取动作 ----
    def _build_actions(self, parent):
        box = self._section(parent, "夹取动作（只动对应关节）")
        self.actions = self.ctx.cfg["servo"]["actions"]
        for label, key, action, duration in (
            ("张开夹爪", "gripper", "open", 0.8),
            ("闭合夹爪", "gripper", "close", 0.8),
            ("半开夹爪", "gripper", "half_open", 0.8),
            ("升降臂抬起", "arm", "arm_up", 1.0),
            ("升降臂降下", "arm", "arm_down", 1.0),
            ("云台俯视", "pitch", "look_down", 1.0),
            ("云台前视", "pitch", "look_forward", 1.0),
        ):
            value = self.actions[action]
            self._btn(box, "%s %d" % (label, value),
                      lambda k=key, v=value, d=duration: self.send({k: v}, d))

    # ---- 预设 + 序列 ----
    def _build_presets(self, parent):
        cfg = self.ctx.cfg["servo"]
        box = self._section(parent, "预设姿态 / 一键序列（车轮 + 夹爪）")

        row1 = tk.Frame(box, bg=theme.PANEL)
        row1.pack(fill="x")
        for name, pose in cfg.get("presets", {}).items():
            self._btn(row1, name, lambda n=name, p=pose: self.apply_preset(n, p))

        row = tk.Frame(box, bg=theme.PANEL)
        row.pack(fill="x", pady=(4, 0))

        tk.Label(row, text="前进", bg=theme.PANEL, fg=theme.TEXT_DIM,
                 font=theme.FONT_SMALL).pack(side="left", padx=(8, 2))
        self.var_fwd = tk.StringVar(value=str(self.base_cfg.get("pick_forward_sec", 0.6)))
        tk.Entry(row, textvariable=self.var_fwd, width=5, bg=theme.PANEL_2,
                 fg=theme.TEXT, insertbackground=theme.TEXT, relief="flat",
                 justify="center", font=theme.FONT_MONO).pack(side="left", ipady=2)
        tk.Label(row, text="秒  后退", bg=theme.PANEL, fg=theme.TEXT_DIM,
                 font=theme.FONT_SMALL).pack(side="left", padx=(8, 2))
        self.var_back = tk.StringVar(value=str(self.base_cfg.get("pick_backward_sec", 0.4)))
        tk.Entry(row, textvariable=self.var_back, width=5, bg=theme.PANEL_2,
                 fg=theme.TEXT, insertbackground=theme.TEXT, relief="flat",
                 justify="center", font=theme.FONT_MONO).pack(side="left", ipady=2)
        tk.Label(row, text="秒", bg=theme.PANEL, fg=theme.TEXT_DIM,
                 font=theme.FONT_SMALL).pack(side="left", padx=(2, 10))

        self._btn(row, "定时夹取（原模式）", self.run_pick_sequence, fg=theme.ACCENT)
        self._btn(row, "中止", self.abort_sequence, fg=theme.ERR)

    # ---- 滑杆 ----
    def _build_sliders(self, parent):
        cfg = self.ctx.cfg["servo"]
        box = self._section(parent, "手动微调（拖动后点「应用」）")
        tk.Label(box, text="相机两端各留 150；数值为目标指令，非位置回读",
                 bg=theme.PANEL, fg=theme.TEXT_DIM, font=theme.FONT_SMALL).pack(anchor="w")
        standby = cfg.get("presets", {}).get("待机（张爪）", {})
        for key in ("pitch", "yaw", "arm", "gripper"):
            self._slider_row(box, key, standby.get(key))

        row = tk.Frame(box, bg=theme.PANEL)
        row.pack(anchor="w", pady=(6, 2))
        self._btn(row, "应用全部", self.apply_sliders, fg=theme.ACCENT)
        self._btn(row, "恢复待机", lambda: self.apply_preset(
            "待机", cfg["presets"].get("待机（张爪）", {})))

        self.lbl_now = tk.Label(parent, text="当前目标：--", bg=theme.BG,
                                fg=theme.TEXT_DIM, font=theme.FONT_SMALL,
                                wraplength=340, justify="left")
        self.lbl_now.pack(anchor="w", padx=6, pady=(4, 0))

    def _section(self, parent, title):
        box = tk.Frame(parent, bg=theme.PANEL)
        box.pack(fill="x", padx=6, pady=(6, 0))
        tk.Label(box, text=title, bg=theme.PANEL, fg=theme.TEXT_DIM,
                 font=theme.FONT_SMALL).pack(anchor="w", padx=8, pady=(5, 0))
        inner = tk.Frame(box, bg=theme.PANEL)
        inner.pack(fill="x", padx=8, pady=(2, 6))
        return inner

    def _btn(self, parent, text, cmd, fg=None):
        b = tk.Button(parent, text=text, command=cmd, bg=theme.PANEL_2,
                      fg=fg or theme.TEXT, relief="flat", font=theme.FONT_SMALL,
                      activebackground=theme.BORDER, padx=9, pady=3)
        b.pack(side="left", padx=3, pady=2)
        return b

    def _slider_row(self, parent, key, default=None):
        lo, hi = self.limits[key]
        line = tk.Frame(parent, bg=theme.PANEL)
        line.pack(fill="x", pady=1)

        tk.Label(line, text="CH%d %s [%d–%d]" % (self.ids[key], self.names[key], lo, hi),
                 width=29, anchor="w", bg=theme.PANEL,
                 fg=theme.TEXT, font=theme.FONT_SMALL).pack(side="left")

        init = int(default) if default is not None else (lo + hi) // 2
        init = max(lo, min(hi, init))
        var = tk.IntVar(value=init)
        val = tk.Label(line, text=str(var.get()), width=5, bg=theme.PANEL,
                       fg=theme.ACCENT, font=theme.FONT_MONO)
        val.pack(side="right")

        sc = tk.Scale(line, from_=lo, to=hi, orient="horizontal", variable=var,
                      showvalue=0, bg=theme.PANEL, fg=theme.TEXT, relief="flat",
                      highlightthickness=0, troughcolor=theme.PANEL_2,
                      activebackground=theme.ACCENT, sliderlength=14,
                      command=lambda v, k=key: self._on_slide(k, v))
        sc.pack(side="left", fill="x", expand=True, padx=4)
        self.sliders[key] = var
        self.var_labels[key] = val

    def _on_slide(self, key, value):
        self.var_labels[key].config(text=str(int(float(value))))

    # ---------- 底盘 ----------
    def _speed(self):
        levels = self.base_cfg.get("speed_levels", {"慢": 0.06, "中": 0.10, "快": 0.15})
        return float(levels.get(self.var_level.get(), 0.06))

    def _turn(self):
        return float(self.base_cfg.get("turn_speed", 0.40))

    def jog(self, sx, sz):
        """按住方向键：起一条 10Hz 的发布线程，直到松开/超时。"""
        if self._busy:
            self.ctx.log("序列执行中，先「中止」才能手动移动")
            return
        self.stop_wheels()
        if self._motion_thread and self._motion_thread.is_alive():
            self.ctx.log("上个运动线程尚未退出，请稍后再试")
            return
        ev = threading.Event()
        self._jog_evt = ev
        vx = sx * self._speed()
        wz = sz * self._turn()
        max_sec = float(self.base_cfg.get("jog_max_sec", 5.0))
        hz = int(self.base_cfg.get("publish_hz", 10))

        def worker():
            t0 = time.monotonic()
            try:
                while not ev.is_set():
                    if time.monotonic() - t0 >= max_sec:
                        self.ctx.log("单次移动已满 %.0f 秒，自动停车" % max_sec)
                        break
                    if not self.ctx.publish_twist(vx, wz, quiet=True):
                        break
                    ev.wait(1.0 / hz)
            finally:
                self.ctx.stop_base()

        self._motion_thread = threading.Thread(target=worker, daemon=True)
        self._motion_thread.start()
        self.ctx.log("车轮：vx=%.2f m/s" % vx if vx else "车轮：wz=%.2f rad/s" % wz)

    def stop_wheels(self):
        if self._jog_evt is not None:
            self._jog_evt.set()
            self._jog_evt = None
        thread = self._motion_thread
        if thread and thread is not threading.current_thread() and thread.is_alive():
            thread.join(timeout=0.3)
        self.ctx.stop_base()

    def nudge(self, sx, sz, seconds):
        """点动：走固定时长后自动停，便于精确对位。"""
        if self._busy:
            self.ctx.log("序列执行中，先「中止」才能手动移动")
            return
        self.stop_wheels()
        if self._motion_thread and self._motion_thread.is_alive():
            self.ctx.log("上个运动线程尚未退出，请稍后再试")
            return
        self._abort = False
        cancel = threading.Event()
        self._jog_evt = cancel
        # Tk 变量同样只能在主线程读
        vx = sx * self._speed()
        wz = sz * self._turn()

        def worker():
            try:
                self._move_for(vx, wz, seconds, cancel)
            except Exception as exc:
                self.ctx.log("点动中断：%s" % exc)
        self._motion_thread = threading.Thread(target=worker, daemon=True)
        self._motion_thread.start()
        self.ctx.log("点动 %.1f 秒" % seconds)

    def _move_for(self, vx, wz, seconds, cancel=None):
        cancel = cancel or threading.Event()
        deadline = time.monotonic() + seconds
        try:
            while time.monotonic() < deadline and not self._abort and not cancel.is_set():
                if not self.ctx.publish_twist(vx, wz, quiet=True):
                    raise RuntimeError("底盘指令发送失败")
                cancel.wait(min(0.05, max(0, deadline - time.monotonic())))
        finally:
            self.ctx.stop_base()

    # ---------- 舵机 ----------
    def send(self, targets, duration=0.8):
        if self._busy:
            self.ctx.log("动作执行中，请稍候…")
            return
        self.ctx.publish_servo(targets, duration)

    def apply_preset(self, name, pose):
        if self._busy:
            self.ctx.log("序列执行中，请先中止")
            return
        if not pose:
            return
        self.ctx.log("应用预设：%s" % name)
        self.ctx.publish_servo(pose, 1.0)

    def apply_sliders(self):
        if self._busy:
            self.ctx.log("序列执行中，请先中止")
            return
        targets = {k: v.get() for k, v in self.sliders.items()}
        self.ctx.publish_servo(targets, 1.0)

    # ---------- 序列 ----------
    def abort_sequence(self):
        self._abort = True
        if self.tracking:
            self.tracking.cancel()
        self.stop_wheels()
        self.ctx.log("已发出中止，车轮已停")

    def run_pick_sequence(self):
        """张爪 → 降臂 → 前进靠近 → 合爪 → 抬臂 → 后退。车轮参与对位。"""
        if self._busy:
            self.ctx.log("已有动作在执行")
            return
        self.stop_wheels()
        if self._motion_thread and self._motion_thread.is_alive():
            self.ctx.log("请等待当前运动停止")
            return
        try:
            fwd = max(0.0, min(5.0, float(self.var_fwd.get())))
        except Exception:
            fwd = 0.6
        try:
            back = max(0.0, min(5.0, float(self.var_back.get())))
        except Exception:
            back = 0.4

        self._busy = True
        self._abort = False
        # 注意：Tk 变量只能在主线程读，这里先取出，别丢进后台线程里 get()
        level_name = self.var_level.get()
        slow = self._speed()
        self.ctx.log("=== 一键夹取：前进 %.1fs / 后退 %.1fs（速度 %s）==="
                     % (fwd, back, level_name))

        def worker():
            steps = [
                ("张爪", lambda: self.ctx.publish_servo({"gripper": self.actions["open"]}, 0.8), 0.9),
                ("降臂", lambda: self.ctx.publish_servo({"arm": self.actions["arm_down"]}, 0.8), 1.1),
                ("前进靠近", lambda: self._move_for(slow, 0, fwd), 0.2),
                ("合爪", lambda: self.ctx.publish_servo({"gripper": self.actions["close"]}, 0.8), 1.2),
                ("抬臂", lambda: self.ctx.publish_servo({"arm": self.actions["arm_up"]}, 1.0), 1.4),
                ("后退", lambda: self._move_for(-slow, 0, back), 0.2),
            ]
            try:
                for name, act, wait in steps:
                    if self._abort:
                        self.ctx.log("序列在中途被中止（%s 之后）" % name)
                        break
                    self.ctx.log("  → %s" % name)
                    if act() is False:
                        raise RuntimeError("%s 指令发送失败，终止序列" % name)
                    deadline = time.monotonic() + wait
                    while not self._abort and time.monotonic() < deadline:
                        time.sleep(min(0.05, max(0, deadline - time.monotonic())))
                if not self._abort:
                    self.ctx.log("=== 序列完成 ===")
            except Exception as exc:
                self.ctx.log("序列中断：%s" % exc)
            finally:
                self.ctx.stop_base()
                self._busy = False

        threading.Thread(target=worker, daemon=True).start()

    # ---------- 刷新 ----------
    def on_tick(self, st):
        if self.tracking:
            self.tracking.tick(st)
        targets = st.get("servo_targets") or {}
        if targets:
            text = "  ".join("%s=%s" % (self.names.get(k, k), v)
                             for k, v in targets.items() if k in self.names)
            self.lbl_now.config(text="当前目标：" + (text or "--"))
        self.cam.on_tick(st)

    def stop_all(self):
        self._abort = True
        if self.tracking:
            self.tracking.cancel()
        self.stop_wheels()
        self.ctx.log("已停止：车轮归零")

    def on_close(self):
        self._abort = True
        if self.tracking:
            self.tracking.close()
        try:
            self.stop_wheels()
        except Exception:
            pass
        self.cam.on_close()
