"""Cruise controls, live camera and PC-only operation evidence."""
import json
import math
import os
import threading
import time
import tkinter as tk
from datetime import datetime
from dataclasses import asdict

from carapp.config import save_config
from carapp.core.cruise import CruisePolicy, CruiseRunner, CruiseSettings
from carapp.core.laser_scan import LaserMonitor
from carapp.core.vendor_lidar import split_scan
from carapp.features.base import Feature
from carapp.ui import theme
from carapp.ui.camerapanel import CameraPanel


class CruiseFeature(Feature):
    key = "cruise"
    title = "自动巡航 + 实时画面"
    MODES = {"avoid": "雷达避障", "follow": "雷达跟随", "guard": "雷达警卫"}
    MODE_HELP = {
        "avoid": "雷达避障：前方扫描范围内的最近距离达到检测距离时，按厂家策略前进转弯。",
        "follow": "雷达跟随：追踪检测距离内的最近目标，会前进、后退和转向；距离控制目标为检测距离的一半。",
        "guard": "雷达警卫：朝向检测距离内的最近目标，只转向，不前进或后退。",
    }

    def __init__(self, ctx):
        super().__init__(ctx)
        self.active = False
        self.monitor = LaserMonitor()
        self.runner = CruiseRunner(ctx, self.monitor, self._notify)
        self.record_dir = None
        self.record_camera = None
        self.record_settings = None
        self.owns_recording = False
        self.events = []
        self.pending_frames = []
        self.record_lock = threading.RLock()

    @property
    def motion_active(self):
        return self.runner.running or bool(self.runner.thread and self.runner.thread.is_alive())

    @property
    def _busy(self):
        return self.runner.running

    @property
    def _motion_thread(self):
        return self.runner.thread

    def build(self, parent):
        cfg = self.ctx.cfg.get("cruise", {})
        panel = tk.Frame(parent, bg=theme.PANEL, padx=12, pady=6)
        panel.pack(fill="x", padx=6, pady=4)
        modes = tk.Frame(panel, bg=theme.PANEL)
        modes.pack(fill="x", pady=(0, 6))
        self.mode = tk.StringVar(value=cfg.get("mode", "avoid"))
        if self.mode.get() not in self.MODES:
            self.mode.set("avoid")
        for value, title in self.MODES.items():
            tk.Radiobutton(modes, text=title, variable=self.mode, value=value,
                           command=self._mode_changed, bg=theme.PANEL, fg=theme.TEXT,
                           selectcolor=theme.PANEL_2, activebackground=theme.PANEL,
                           activeforeground=theme.ACCENT, font=theme.FONT,
                           anchor="w").pack(side="left", padx=(0, 22))
        row = tk.Frame(panel, bg=theme.PANEL)
        row.pack(fill="x")
        self.detection_distance = tk.StringVar(value=str(cfg.get("detection_distance", .60)))
        self.speed = tk.StringVar(value=str(cfg.get("speed", .06)))
        for column, label, variable, lower, upper, step in (
            (0, "检测距离（米）", self.detection_distance, .30, 1.50, .05),
            (2, "移动速度（米/秒）", self.speed, .02, .15, .01),
        ):
            tk.Label(row, text=label, bg=theme.PANEL, fg=theme.TEXT,
                     font=theme.FONT).grid(row=0, column=column, sticky="w")
            tk.Spinbox(row, from_=lower, to=upper, increment=step, textvariable=variable,
                       width=7, format="%.2f", bg=theme.PANEL_2, fg=theme.ACCENT,
                       insertbackground=theme.TEXT, buttonbackground=theme.PANEL_2,
                       font=theme.FONT_MONO, relief="flat").grid(row=0, column=column+1, padx=(6, 16))
        actions = tk.Frame(panel, bg=theme.PANEL)
        actions.pack(fill="x", pady=(6, 0))
        for text, command, color in (
            ("应用设置", self.apply_settings, theme.ACCENT),
            ("▶ 启动巡航", self.start, theme.OK),
            ("■ 停止巡航", self.stop_all, theme.ERR),
        ):
            tk.Button(actions, text=text, command=command, bg=theme.PANEL_2, fg=color,
                      font=theme.FONT, relief="flat", padx=12, pady=6,
                      activebackground=theme.BORDER).pack(side="left", padx=(0, 8))
        limits_hint = tk.Label(panel, text="检测距离从雷达中心计量，范围 0.30–1.50 m；移动速度 0.02–0.15 m/s。切换模式后须重新启动。",
                 bg=theme.PANEL, fg=theme.TEXT_DIM, font=theme.FONT_SMALL,
                 anchor="w", justify="left", wraplength=1050)
        limits_hint.pack(fill="x", pady=(4, 3))
        self.mode_help = tk.Label(panel, text=self.MODE_HELP[self.mode.get()], bg=theme.PANEL,
                                  fg=theme.TEXT_DIM, font=theme.FONT_SMALL,
                                  anchor="w", justify="left", wraplength=1050)
        self.mode_help.pack(fill="x", pady=(0, 3))
        self.applied = tk.Label(panel, text="", bg=theme.PANEL, fg=theme.ACCENT,
                                font=theme.FONT_SMALL, anchor="w", justify="left", wraplength=1050)
        self.applied.pack(fill="x")
        self.phase = tk.Label(panel, text="待机：等待雷达和画面就绪后启动", bg=theme.PANEL,
                              fg=theme.TEXT, font=theme.FONT_BOLD, anchor="w", justify="left", wraplength=1050)
        self.phase.pack(fill="x", pady=(4, 3))
        self.sensor = tk.Label(panel, text="雷达：等待 /scan_raw", bg=theme.PANEL,
                               fg=theme.TEXT_DIM, font=theme.FONT_SMALL, anchor="w")
        self.sensor.pack(fill="x")
        hint = tk.Label(panel, text="采用原厂雷达避障、跟随和警卫策略，按配置的前方检测角处理雷达数据。"
                            "切页、失焦、连接中断或雷达数据失效会停止；录像和关键帧直接保存在电脑。",
                 bg=theme.PANEL, fg=theme.TEXT_DIM, font=theme.FONT_SMALL,
                 wraplength=1050, justify="left", anchor="w")
        hint.pack(fill="x", pady=(6, 0))
        def wrap(event):
            width = max(200, event.width-24)
            hint.configure(wraplength=width)
            limits_hint.configure(wraplength=width)
            self.applied.configure(wraplength=width)
            self.sensor.configure(wraplength=width)
            self.phase.configure(wraplength=width)
            self.mode_help.configure(wraplength=width)
        panel.bind("<Configure>", wrap)
        self.cam = CameraPanel(parent, self.ctx)
        self.cam.pack(fill="both", expand=True, pady=(0, 6))
        try:
            self.runner.configure(self.read_settings())
        except ValueError as exc:
            self.ctx.log("巡航设置需修正："+str(exc))
        self._show_applied()

    def read_settings(self):
        try:
            cfg = self.ctx.cfg.get("cruise", {})
            return CruiseSettings(detection_distance=float(self.detection_distance.get()),
                                  speed=float(self.speed.get()), mode=self.mode.get(),
                                  scan_angle=float(cfg.get("scan_angle", 90.)),
                                  lidar_type=cfg.get("lidar_type", "standard"))
        except (ValueError, TypeError, OverflowError) as exc:
            raise ValueError("参数无效："+str(exc)) from exc

    def _show_applied(self):
        cfg = self.runner.settings
        text = "当前已应用：%s，检测距离 %.2f m，移动速度 %.2f m/s" % (
            self.MODES[cfg.mode], cfg.detection_distance, cfg.speed)
        self.applied.config(text=text, fg=theme.ACCENT)

    def _mode_changed(self):
        """A mode selection always ends the previous session; it never starts motors."""
        self.stop_all()
        self.mode_help.config(text=self.MODE_HELP[self.mode.get()])
        if self.apply_settings():
            self.runner.phase = "%s已选择，请点击启动巡航" % self.MODES[self.mode.get()]
            self.ctx.log(self.runner.phase)
        else:
            self.mode.set(self.runner.settings.mode)
            self.mode_help.config(text=self.MODE_HELP[self.mode.get()])

    def apply_settings(self):
        try:
            settings = self.read_settings()
            old = self.ctx.cfg.get("cruise", {})
            updated = dict(old, **settings.__dict__)
            updated.pop("front_back_distance", None)
            updated.pop("side_distance", None)
            self.ctx.cfg["cruise"] = updated
            try:
                save_config(self.ctx.cfg)
            except OSError:
                self.ctx.cfg["cruise"] = old
                raise
            if settings.mode != self.runner.settings.mode:
                self.stop_all()
            self.runner.configure(settings)
            self._show_applied()
            self.ctx.log("巡航参数已应用：%s，检测距离 %.2f m，速度 %.2f m/s" %
                         (self.MODES[settings.mode], settings.detection_distance, settings.speed))
            return True
        except (ValueError, OSError) as exc:
            self.ctx.log("未应用巡航参数："+str(exc))
            return False

    def ros_subscriptions(self):
        return {"/scan_raw": 100}

    def on_ros_message(self, topic, msg, received_at):
        if topic == "/scan_raw":
            self.monitor.update(msg, received_at)

    def start(self):
        if not self.active or self.runner.running:
            return
        if self.ctx.motion_in_use(exclude=self):
            self.ctx.log("请先停止其他底盘或夹取动作")
            return
        if not self.apply_settings():
            return
        try:
            scan = self.monitor.snapshot()
            check = CruisePolicy().decide(scan, self.runner.settings, self.runner.clock())
            if check[3]:
                raise ValueError(check[2])
            camera = self.ctx.app.camera
            if not camera or not self.ctx.state.get("cam_connected"):
                raise ValueError("等待实时画面就绪后才能启动并记录巡航")
            self.record_dir = self.ctx.save_dir("自动巡航/"+datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
            self.record_camera, self.owns_recording = camera, not camera.recording
            self.record_settings = asdict(self.runner.settings)
            self.events = []
            self.pending_frames = []
            if self.owns_recording:
                camera.start_record(self.record_dir)
            self.runner.start(self.runner.settings)
        except (ValueError, OSError) as exc:
            self.ctx.log("未启动巡航："+str(exc))
            self._finish_record()

    def _notify(self, text):
        self.ctx.log("自动巡航："+text)
        with self.record_lock:
            if not self.record_dir:
                return
            scan = self.runner.policy.last_scan or self.monitor.snapshot()
            self.events.append(dict(time=time.time(), phase=text, raw_scan=scan,
                                    mode=self.runner.settings.mode, settings=asdict(self.runner.settings),
                                    pose=self.ctx.state.get("pose"), base=self.ctx.state.get("base_cmd"),
                                    lateral=self.ctx.state.get("base_vy"),
                                    decision=self.runner.policy.diagnostics))
            frame = getattr(getattr(self, "cam", None), "_last_frame", None)
            if frame:
                # Disk I/O belongs on the UI side, never inside the motor watchdog.
                self.pending_frames.append((len(self.events), frame))

    def _flush_frames(self):
        with self.record_lock:
            if not self.record_dir:
                return
            pending, self.pending_frames = self.pending_frames, []
            directory = self.record_dir
        for index, frame in pending:
            ext = "jpg" if frame[2] == "jpeg" else "png"
            try:
                with open(os.path.join(directory, "阶段_%03d.%s" % (index, ext)), "wb") as out:
                    out.write(frame[1])
            except OSError as exc:
                self.ctx.log("巡航关键帧保存失败："+str(exc))

    def _finish_record(self):
        with self.record_lock:
            if not self.record_dir:
                return
            self._flush_frames()
            if self.owns_recording and self.record_camera:
                self.record_camera.stop_record()
            path = self.record_dir
            settings = self.record_settings or asdict(self.runner.settings)
            report = dict(host=self.ctx.cfg["car"]["host"], settings=settings,
                          algorithm="vendor_lidar_controller", mode=settings["mode"],
                          events=self.events, result=self.runner.phase, vehicle_files_modified=[])
            try:
                with open(os.path.join(path, "运行记录.json"), "w", encoding="utf-8") as out:
                    json.dump(report, out, ensure_ascii=False, indent=2)
            except OSError as exc:
                self.ctx.log("巡航记录保存失败："+str(exc))
            finally:
                self.record_dir = self.record_camera = None
                self.record_settings = None
                self.owns_recording = False
            self.ctx.log("巡航录像及关键帧已保存到电脑："+path)

    def stop_all(self):
        self.runner.stop()
        self._finish_record()

    def on_activate(self):
        self.active = True

    def on_deactivate(self):
        self.active = False
        self.stop_all()

    def on_vehicle_change(self):
        self.stop_all()
        self.monitor.reset()

    def on_tick(self, st):
        self._flush_frames()
        if self.record_dir and not self.runner.running:
            self._finish_record()
        scan = self.monitor.snapshot()
        age = time.monotonic()-scan["received_at"]
        fresh = 0 <= age <= LaserMonitor.TIMEOUT and not scan["error"]
        decision = self.runner.policy.diagnostics
        display_scan = self.runner.policy.last_scan
        if self.runner.running and display_scan and decision.get("scan_sequence") == display_scan["sequence"]:
            values = decision
        else:
            # Use the same vendor slicing as control, including G4 indexing and
            # the configured angle; never mix a new scan with an old PID target.
            values = dict(left=None, right=None, target_distance=None, target_angle=None)
            if not scan["error"]:
                left, right = split_scan(scan, self.runner.settings.scan_angle, self.runner.settings.lidar_type)
                for key, ranges in (("left", left), ("right", right)):
                    valid = [value for value in ranges if isinstance(value, (int, float))
                             and not isinstance(value, bool) and math.isfinite(value) and value > 0]
                    values[key] = min(valid) if valid else None
        def distance(key):
            value = values.get(key)
            return "%.2f m" % value if isinstance(value, (int, float)) else "未知"
        if values.get("target_distance") is not None:
            target_text = "%s（%.1f°）" % (distance("target_distance"), values["target_angle"])
        else:
            nearest = [values[key] for key in ("left", "right") if values.get(key) is not None]
            target_text = "%.2f m" % min(nearest) if nearest else "未知"
        self.phase.config(text=self.runner.phase, fg=theme.OK if self.runner.running else theme.TEXT)
        self.sensor.config(text="雷达 %s｜左前 %s｜右前 %s｜最近目标 %s｜%s" %
                           ("就绪" if fresh else "不可用", distance("left"), distance("right"), target_text,
                            "延迟 %.2f s" % age if scan["received_at"] else "等待数据"),
                           fg=theme.OK if fresh else theme.ERR)
        self.cam.on_tick(st)

    def on_close(self):
        self.stop_all()
        self.cam.on_close()
