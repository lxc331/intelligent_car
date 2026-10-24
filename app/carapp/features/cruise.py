"""Cruise controls, live camera and PC-only operation evidence."""
import json
import os
import threading
import time
import tkinter as tk
from datetime import datetime

from carapp.config import save_config
from carapp.core.cruise import CruisePolicy, CruiseRunner, CruiseSettings, LaserMonitor
from carapp.features.base import Feature
from carapp.ui import theme
from carapp.ui.camerapanel import CameraPanel


class CruiseFeature(Feature):
    key = "cruise"
    title = "自动巡航避障 + 实时画面"

    def __init__(self, ctx):
        super().__init__(ctx)
        self.active = False
        self.monitor = LaserMonitor()
        self.runner = CruiseRunner(ctx, self.monitor, self._notify)
        self.record_dir = None
        self.record_camera = None
        self.owns_recording = False
        self.events = []
        self.pending_frames = []
        self.record_lock = threading.RLock()

    @property
    def _busy(self):
        return self.runner.running

    @property
    def _motion_thread(self):
        return self.runner.thread

    def build(self, parent):
        cfg = self.ctx.cfg.get("cruise", {})
        panel = tk.Frame(parent, bg=theme.PANEL, padx=12, pady=10)
        panel.pack(fill="x", padx=6, pady=6)
        row = tk.Frame(panel, bg=theme.PANEL)
        row.pack(fill="x")
        self.front_back_distance = tk.StringVar(value=str(cfg.get("front_back_distance", cfg.get("detection_distance", .60))))
        self.side_distance = tk.StringVar(value=str(cfg.get("side_distance", .35)))
        self.speed = tk.StringVar(value=str(cfg.get("speed", .06)))
        for column, label, variable, lower, upper, step in (
            (0, "前后检测距离（米）", self.front_back_distance, .10, 2.0, .05),
            (2, "左右检测距离（米）", self.side_distance, .10, 2.0, .05),
            (4, "巡航速度（米/秒）", self.speed, .02, .15, .01),
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
        tk.Label(panel, text="两项距离均为 0.10–2.00 m，独立保存；速度 0.02–0.15 m/s，按用户设定距离判断绕障。",
                 bg=theme.PANEL, fg=theme.TEXT_DIM, font=theme.FONT_SMALL, anchor="w").pack(fill="x", pady=(8, 3))
        self.applied = tk.Label(panel, text="", bg=theme.PANEL, fg=theme.ACCENT,
                                font=theme.FONT_SMALL, anchor="w", justify="left", wraplength=1050)
        self.applied.pack(fill="x")
        self.phase = tk.Label(panel, text="待机：等待雷达和画面就绪后启动", bg=theme.PANEL,
                              fg=theme.TEXT, font=theme.FONT_BOLD, anchor="w")
        self.phase.pack(fill="x", pady=(8, 3))
        self.sensor = tk.Label(panel, text="雷达：等待 /scan_raw", bg=theme.PANEL,
                               fg=theme.TEXT_DIM, font=theme.FONT_SMALL, anchor="w")
        self.sensor.pack(fill="x")
        hint = tk.Label(panel, text="前方或侧方进入对应检测距离时前进转弯；后方距离用于检查车尾转弯空间，不自动倒车。"
                            "连续三帧前方和两侧畅通后直行；转弯受阻时停车，20 秒未绕开则停止。切页、失焦、数据失效会取消巡航。",
                 bg=theme.PANEL, fg=theme.TEXT_DIM, font=theme.FONT_SMALL,
                 wraplength=1050, justify="left", anchor="w")
        hint.pack(fill="x", pady=(6, 0))
        def wrap(event):
            width = max(200, event.width-24)
            hint.configure(wraplength=width)
            self.applied.configure(wraplength=width)
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
            return CruiseSettings(front_back_distance=float(self.front_back_distance.get()),
                                  side_distance=float(self.side_distance.get()), speed=float(self.speed.get()))
        except (ValueError, TypeError, OverflowError) as exc:
            raise ValueError("参数无效："+str(exc)) from exc

    def _show_applied(self):
        cfg = self.runner.settings
        text = "当前已应用：前后 %.2f m，左右 %.2f m，巡航速度 %.2f m/s" % (cfg.front_back_distance, cfg.side_distance, cfg.speed)
        self.applied.config(text=text, fg=theme.ACCENT)

    def apply_settings(self):
        try:
            settings = self.read_settings()
            old = self.ctx.cfg.get("cruise", {})
            updated = dict(old, **settings.__dict__)
            updated.pop("detection_distance", None)
            self.ctx.cfg["cruise"] = updated
            try:
                save_config(self.ctx.cfg)
            except OSError:
                self.ctx.cfg["cruise"] = old
                raise
            self.runner.configure(settings)
            self._show_applied()
            self.ctx.log("巡航参数已应用：前后 %.2f m，左右 %.2f m，速度 %.2f m/s" %
                         (settings.front_back_distance, settings.side_distance, settings.speed))
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
        others = [f for f in self.ctx.app.features if f is not self]
        if any(getattr(f, "_busy", False) or
               (getattr(f, "_motion_thread", None) and f._motion_thread.is_alive()) for f in others):
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
            scan = self.monitor.snapshot()
            self.events.append(dict(time=time.time(), phase=text, scan=scan,
                                    pose=self.ctx.state.get("pose"), base=self.ctx.state.get("base_cmd"),
                                    lateral=self.ctx.state.get("base_vy")))
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
            report = dict(host=self.ctx.cfg["car"]["host"], settings=self.runner.settings.__dict__,
                          events=self.events, result=self.runner.phase, vehicle_files_modified=[])
            try:
                with open(os.path.join(path, "运行记录.json"), "w", encoding="utf-8") as out:
                    json.dump(report, out, ensure_ascii=False, indent=2)
            except OSError as exc:
                self.ctx.log("巡航记录保存失败："+str(exc))
            finally:
                self.record_dir = self.record_camera = None
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
        def distance(key):
            return "%.2f m" % scan[key] if scan.get(key) is not None else "未知"
        self.phase.config(text=self.runner.phase, fg=theme.OK if self.runner.running else theme.TEXT)
        self.sensor.config(text="雷达 %s｜前方通道 %s｜左侧 %s｜右侧 %s｜后方 %s｜%s" %
                           ("就绪" if fresh else "不可用", distance("front"), distance("left"), distance("right"),
                            distance("rear"),
                            "延迟 %.2f s" % age if scan["received_at"] else "等待数据"),
                           fg=theme.OK if fresh else theme.ERR)
        self.cam.on_tick(st)

    def on_close(self):
        self.stop_all()
        self.cam.on_close()
