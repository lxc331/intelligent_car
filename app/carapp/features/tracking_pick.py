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
from carapp.core.visual_pick import (VisualPickRunner, calibration_problem,
                                     validate_calibration, validate_pick_reference)
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
        self.label = tk.Label(box, text="标定：目标要推到两个夹爪的根部（合上夹爪能夹住它），不能只放在爪尖之间或远处；"
                                        "标定后跟车会一直走到这条夹取线才合爪。仅换目标远近时无需重标定。",
                              bg=theme.PANEL, fg=theme.TEXT_DIM, anchor="w", font=theme.FONT_SMALL,
                              wraplength=720, justify="left")
        self.label.pack(fill="x", pady=(3,0))

    def car_identity(self):
        """Who the calibration belongs to: the vehicle, not the address we reach it on."""
        car = self.ctx.cfg["car"]
        return car.get("profile_id") or car["host"]

    def key(self):
        car = self.ctx.cfg["car"]
        profile, color = car.get("profile_id"), self.cfg["color"]
        if profile:
            # 夹取标定属于这台车本身：有线、自带热点、STA 连的是同一副机械。
            return profile+":"+color
        return "manual:"+car["host"]+":"+color

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
        calibrations = self.cfg["calibrations"]
        car = self.ctx.cfg["car"]
        identity, host, color = self.car_identity(), car["host"], self.cfg["color"]
        # 旧版本把地址也写进键；这台车换过接入地址或连接方式时，用档案里保存的
        # 各个地址都试着找一遍，不必重新标定。
        profile = next((p for p in self.ctx.cfg.get("cars", [])
                        if p.get("id") == car.get("profile_id")), {})
        keys = [self.key()]
        if car.get("profile_id"):
            keys.append(identity+":"+host+":"+color)
        keys.append(host+":"+color)
        for endpoint in (profile.get("host"), profile.get("sta_host"),
                         None if car.get("transport") == "wifi" else profile.get("wifi_host")):
            if endpoint:
                keys.extend([identity+":"+endpoint+":"+color, endpoint+":"+color])
        cal = None
        for key in dict.fromkeys(keys):
            cal = calibrations.get(key)
            if cal:
                break
        endpoints = {str(value) for value in (host, profile.get("host"), profile.get("sta_host"))
                     if value}
        if car.get("transport") != "wifi" and profile.get("wifi_host"):
            endpoints.add(str(profile["wifi_host"]))
        if cal and cal.get("host") != identity and cal.get("host") in endpoints:
            # 旧标定存的是地址；迁移成车辆身份后再校验。
            cal = dict(cal, host=identity)
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
            self.status("视觉准备：把目标推到两个张开夹爪的最里面（贴住夹爪根部，不是只放在爪子前端），等识别框稳定后标定")
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
            cal.update(host=self.car_identity(), color=self.cfg["color"], pose=self.pose(),
                       created=time.strftime("%Y-%m-%d %H:%M:%S"), version=2, contact_confirmed=True)
            self.cfg["calibrations"][self.key()] = cal
            save_config(self.ctx.cfg)
            path = self.ctx.save_dir("追踪夹取/标定")
            if self.tracker.last_payload:
                with open(os.path.join(path, "标定_%s.jpg" % time.strftime("%Y%m%d_%H%M%S")), "wb") as f:
                    f.write(self.tracker.last_payload)
            self.status("标定已保存；保持相机姿态，将同一目标放远后点一键追踪夹取")
            self.ctx.log("追踪夹取：夹取线 x=%.3f y=%.3f width=%.3f（合爪位置须与此一致）"
                         % (cal["x"], cal["y"], cal["width"]))
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
        self.ctx.log("追踪夹取：收到启动请求")
        try:
            self._start()
        except Exception as exc:
            # 界面回调里抛异常只会进控制台，现场看起来就是“点了没反应”。
            self.ctx.log("追踪夹取启动失败：%s: %s" % (type(exc).__name__, exc))

    def _start(self):
        if self.ctx.motion_in_use(exclude=self.owner):
            self.ctx.log("其他功能正在占用底盘，请先点击顶部「停止全部」")
            return
        if self.owner._busy:
            self.ctx.log("已有动作执行中")
            return
        try:
            self.require_camera()
            run_cfg = approach_config(self.cfg, self.distance_var.get())
            cal = validate_calibration(self.calibration(), self.car_identity(), self.cfg["color"], self.pose())
            validate_servo_targets(self.ctx.cfg, dict(self.pose(), gripper=self.ctx.cfg["servo"]["actions"]["open"]))
            self.owner.stop_wheels()
            if self.owner._motion_thread and self.owner._motion_thread.is_alive():
                raise ValueError("请等待手动运动停止")
        except ValueError as exc:
            text = str(exc)
            self.ctx.log(text)
            if "标定" in text:
                self.ctx.log("追踪夹取：这台车还没有可用的夹取标定，车轮不会动。"
                             "顺序：①视觉准备 → 把目标放到张开的两爪之间 → ②两爪间标定 → 一键追踪夹取")
            return
        camera = self.ctx.app.camera
        if camera is None:
            self.ctx.log("摄像头未启动，不能开始追踪")
            return
        self.ctx.log("追踪夹取：开始追踪，标定线 y=%.3f（目标 y 到线才合爪；最后 %.0f%% 画面高度只直行）"
                     % (cal["y"], run_cfg.get("straight_margin", .12) * 100))
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
        problem = calibration_problem(cal,self.car_identity(),self.cfg["color"],self.pose())
        if problem:
            calibration_text = "标定不可用（%s）" % problem
        else:
            calibration_text = "已标定，夹取线 y=%.2f" % cal["y"]
        self.label.config(text="%s；%s；%s" % (state.get("tracking_phase") or "等待操作",suffix,calibration_text),
                          fg=theme.ERR if problem else theme.TEXT_DIM)

    def close(self):
        self.cancel()
        self.tracker.stop()
