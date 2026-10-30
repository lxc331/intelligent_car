"""Controls for colour tracking, per-car pickup calibration and supervised approach."""
import importlib.util
import json
import os
import threading
import time
import tkinter as tk

from carapp.config import save_config, validate_servo_targets
from carapp.core.color_tracker import COLORS, ColorTracker
from carapp.core.pick_approach import approach_config
from carapp.core.visual_pick import VisualPickRunner, validate_calibration, validate_pick_reference
from carapp.ui import theme


class TrackingPickControls:
    def __init__(self, parent, owner):
        self.owner, self.ctx = owner, owner.ctx
        self.cfg = self.ctx.cfg["tracking_pick"]
        self.runner = None
        self.events = []
        self.prepared_at = 0
        self.available = importlib.util.find_spec("PIL") is not None
        self.tracker = ColorTracker(self.cfg["color"])
        if self.available:
            self.tracker.start()
        self.ctx.add_frame_listener(self.on_frame)
        box = owner._section(parent, "视觉追踪夹取（目标需在视野内，首次先标定）")
        row = tk.Frame(box, bg=theme.PANEL)
        row.pack(fill="x")
        self.color_var = tk.StringVar(value=next((cn for cn,en in COLORS.items() if en==self.cfg["color"]), "红色"))
        menu = tk.OptionMenu(row, self.color_var, *COLORS, command=self.change_color)
        menu.config(bg=theme.PANEL_2, fg=theme.TEXT, highlightthickness=0)
        menu.pack(side="left")
        owner._btn(row, "① 视觉准备", self.prepare)
        owner._btn(row, "② 两爪间标定", self.calibrate)
        owner._btn(row, "一键追踪夹取", self.start, fg=theme.ACCENT)
        owner._btn(row, "中止追踪", self.cancel, fg=theme.ERR)
        limits = tk.Frame(box, bg=theme.PANEL)
        limits.pack(fill='x')
        tk.Label(limits, text='最大追踪行程（米）', bg=theme.PANEL, fg=theme.TEXT_DIM,
                 font=theme.FONT_SMALL).pack(side='left')
        self.distance_var = tk.StringVar(value=str(self.cfg.get('approach_distance', 1.5)))
        tk.Spinbox(limits, from_=.3, to=3., increment=.1, width=5, textvariable=self.distance_var,
                   bg=theme.PANEL_2, fg=theme.TEXT).pack(side='left', padx=4)
        tk.Label(limits, text='1 米目标建议 1.5 米行程；目标需持续可见', bg=theme.PANEL,
                 fg=theme.TEXT_DIM, font=theme.FONT_SMALL).pack(side='left')
        self.label = tk.Label(box, text="标定时须把目标放在两爪之间；不能用远处目标标定。仅换目标远近时无需重标定。",
                              bg=theme.PANEL, fg=theme.TEXT_DIM, anchor="w", font=theme.FONT_SMALL,
                              wraplength=720, justify="left")
        self.label.pack(fill="x", pady=(3,0))

    def key(self):
        car = self.ctx.cfg["car"]
        # 两台车的热点 IP 相同，不能共享夹取标定。
        return (car.get("profile_id") or "manual")+":"+car["host"]+":"+self.cfg["color"]

    def pose(self):
        actions = self.ctx.cfg["servo"]["actions"]
        pose = {"pitch": actions["look_down"], "yaw": 1500, "arm": actions["arm_down"]}
        # A measured pickup height belongs to this car's visual preparation,
        # not to every manual arm-down action or another car's calibration.
        profile = self.ctx.cfg['car'].get('profile_id')
        override = self.cfg.get('preparation_poses', {}).get(profile, {}) if profile else {}
        pose.update({key: override[key] for key in pose if key in override})
        return validate_servo_targets(self.ctx.cfg, pose)

    def calibration(self):
        cal = self.cfg["calibrations"].get(self.key())
        car = self.ctx.cfg["car"]
        if cal is None and car.get("transport") != "wifi":
            # 兼容原来的有线标定；共享热点的旧标定无法区分车辆，需重新做。
            cal = self.cfg["calibrations"].get(car["host"]+":"+self.cfg["color"])
        return cal

    def on_frame(self, timestamp, payload, mode):
        if self.available and self.ctx.app.cam_topic == "/image_raw":
            self.tracker.submit(payload)

    def reset(self):
        self.prepared_at = 0
        self.tracker.reset()
        self.ctx.state.update(tracking_target=None, tracking_phase="等待标定或启动")

    def change_color(self, name):
        if self.owner._busy:
            self.color_var.set(next(cn for cn,en in COLORS.items() if en==self.cfg["color"]))
            self.ctx.log("追踪执行中，请先中止再换颜色")
            return
        self.cfg["color"] = COLORS[name]
        self.tracker.reset(self.cfg["color"])
        self.prepared_at = 0
        save_config(self.ctx.cfg)

    def require_camera(self):
        if not self.available:
            raise ValueError("视觉追踪需要 Pillow，请先安装 Pillow 后重启 APP")
        if not self.ctx.bridge.connected:
            raise ValueError("小车尚未连接")
        if self.ctx.app.cam_topic != "/image_raw":
            raise ValueError("请先把实时画面切换到 /image_raw 原始图像")

    def prepare(self):
        if self.ctx.motion_in_use(exclude=self.owner):
            self.ctx.log("其他功能正在占用底盘，请先点击顶部「停止全部」")
            return
        if self.owner._busy:
            self.ctx.log("请先中止当前动作")
            return
        try:
            self.require_camera()
            self.owner.stop_wheels()
            targets = dict(self.pose(), gripper=self.ctx.cfg["servo"]["actions"]["open"])
            if not self.ctx.publish_servo(targets, 1.0):
                return
            self.prepared_at = time.monotonic()
            self.tracker.reset()
            self.status("视觉准备：保持姿态，把目标放在可夹位置，等识别框稳定后标定")
        except ValueError as exc:
            self.ctx.log(str(exc))

    def calibrate(self):
        if self.owner._busy:
            self.ctx.log("动作执行中不能重新标定")
            return
        try:
            self.require_camera()
            obs = self.tracker.snapshot()
            now = time.monotonic()
            targets = self.ctx.state.get("servo_targets") or {}
            if not self.prepared_at or now-self.prepared_at < 1.3 or any(targets.get(k)!=v for k,v in self.pose().items()):
                raise ValueError("先点①视觉准备，保持准备姿态，再标定")
            if now-obs["seen_at"] > self.cfg["frame_timeout"] or obs["stable"] < self.cfg["stable_frames"] or not obs["target"]:
                raise ValueError("目标未稳定识别，请检查颜色、光照和目标是否在画面内")
            target = obs["target"]
            validate_pick_reference(target)
            cal = dict((k,target[k]) for k in ("x","y","width"))
            cal.update(host=self.ctx.cfg["car"]["host"], color=self.cfg["color"], pose=self.pose(),
                       created=time.strftime("%Y-%m-%d %H:%M:%S"), version=2, contact_confirmed=True)
            self.cfg["calibrations"][self.key()] = cal
            save_config(self.ctx.cfg)
            path = self.ctx.save_dir("追踪夹取/标定")
            if self.tracker.last_payload:
                with open(os.path.join(path, "标定_%s.jpg" % time.strftime("%Y%m%d_%H%M%S")), "wb") as f:
                    f.write(self.tracker.last_payload)
            self.status("标定已保存；保持相机姿态，将同一目标放远后点一键追踪夹取")
        except ValueError as exc:
            self.ctx.log(str(exc))

    def status(self, text):
        self.ctx.state.update(tracking_phase=text)
        self.ctx.log("追踪夹取："+text)
        self.events.append({"time": time.time(), "phase": text, "target": self.tracker.snapshot().get("target"),
                            "pose": self.ctx.state.get("pose"), "base_command": self.ctx.state.get("base_cmd")})
        path = getattr(self, "record_dir", None)
        if path and self.tracker.last_payload:
            with open(os.path.join(path, "阶段_%02d.jpg" % len(self.events)), "wb") as f:
                f.write(self.tracker.last_payload)

    def start(self):
        if self.ctx.motion_in_use(exclude=self.owner):
            self.ctx.log("其他功能正在占用底盘，请先点击顶部「停止全部」")
            return
        if self.owner._busy:
            self.ctx.log("已有动作执行中")
            return
        try:
            self.require_camera()
            run_cfg = approach_config(self.cfg, self.distance_var.get())
            cal = validate_calibration(self.calibration(), self.ctx.cfg["car"]["host"], self.cfg["color"], self.pose())
            validate_servo_targets(self.ctx.cfg, dict(self.pose(), gripper=self.ctx.cfg["servo"]["actions"]["open"]))
            self.owner.stop_wheels()
            if self.owner._motion_thread and self.owner._motion_thread.is_alive():
                raise ValueError("请等待手动运动停止")
        except ValueError as exc:
            self.ctx.log(str(exc))
            return
        camera = self.ctx.app.camera
        if camera is None:
            self.ctx.log("摄像头未启动，不能开始追踪")
            return
        self.record_dir = self.ctx.save_dir("追踪夹取/"+time.strftime("%Y%m%d_%H%M%S"))
        self.owner._busy = True
        self.owner._abort = False
        self.events, self.trace_entries = [], []
        owned_recording = not camera.recording
        self.runner = VisualPickRunner(run_cfg, cal, self.ctx.cfg["servo"]["actions"],
            self.tracker.snapshot,
            lambda: {"pose": self.ctx.state.get("pose"), "seen_at": self.ctx.state.get("pose_seen_at",0)} if self.ctx.state.get("pose") else None,
            lambda vx,wz: self.ctx.publish_twist(vx,wz,quiet=True), self.ctx.publish_servo,
            self.status, lambda: self.ctx.bridge.connected, self.trace_entries.append)
        runner = self.runner

        def worker():
            try:
                if owned_recording:
                    camera.start_record(self.record_dir)
                self.status("设置视觉准备姿态")
                if not runner._servo(dict(self.pose(),gripper=self.ctx.cfg["servo"]["actions"]["open"]),1.):
                    return
                self.tracker.reset()
                runner.run()
            except Exception as exc:
                runner.result = "停止："+str(exc)
                self.status("停止："+str(exc))
            finally:
                try:
                    runner.cancel()
                    if owned_recording:
                        camera.stop_record()
                    with open(os.path.join(self.record_dir,"运行记录.json"),"w",encoding="utf-8") as f:
                        json.dump({"host":self.ctx.cfg["car"]["host"],"calibration":cal,
                                   "events":self.events,"trace":self.trace_entries,"result":runner.result,
                                   "approach_limits":{k:run_cfg[k] for k in ('max_distance','max_seconds','max_speed')},
                                   "vehicle_files_modified":[]},f,ensure_ascii=False,indent=2)
                finally:
                    self.owner._busy = False
                    self.record_dir = None
        threading.Thread(target=worker,daemon=True,name="visual-pick").start()

    def cancel(self):
        if self.runner:
            self.runner.cancel()

    def background_tick(self, state):
        """Refresh shared overlays even while the controls' page is hidden."""
        obs = self.tracker.snapshot()
        target = obs.get("target") if time.monotonic()-obs["seen_at"] <= self.cfg["frame_timeout"] else None
        cal = self.calibration()
        state.update(tracking_target=target, tracking_calibration=cal)
        self.ctx.state.update(tracking_target=target, tracking_calibration=cal)

    def tick(self, state, *, refresh=True):
        # The standalone entry retains its old state-refresh behavior. Gripper
        # already runs the background hook once per cycle and only draws here.
        if refresh:
            self.background_tick(state)
        target, cal = state.get("tracking_target"), state.get("tracking_calibration")
        suffix = ("目标 x=%.2f y=%.2f" % (target["x"],target["y"])) if target else "未发现稳定目标"
        try:
            validate_calibration(cal,self.ctx.cfg["car"]["host"],self.cfg["color"],self.pose())
            calibration_text = "已标定，夹取线 y=%.2f" % cal["y"]
        except ValueError:
            calibration_text = "标定无效：目标放两爪之间后重标定"
        self.label.config(text="%s；%s；%s" % (state.get("tracking_phase") or "等待操作",suffix,calibration_text))

    def close(self):
        self.cancel()
        self.tracker.stop()
