"""Import a saved map, localize, preview a path and explicitly navigate to a goal."""
import math
import os
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import filedialog, ttk

from carapp.config import APP_DIR, save_config
from carapp.core.credentials import local_password
from carapp.core.laser_scan import LaserMonitor
from carapp.core.map_data import MapModel
from carapp.core.navigation_map import load_navigation_map, valid_target
from carapp.core.navigation_record import NavigationRecord
from carapp.core.navigation_remote import NavigationRemote
from carapp.core.navigation_session import NavigationSession
from carapp.features.base import Feature
from carapp.ui import theme
from carapp.ui.mapping_camera import MappingCamera
from carapp.ui.navigation_map import NavigationMapPanel


class NavigationFeature(Feature):
    key = "navigation"
    title = "沙盘导航 + 实时展示"
    stop_on_focus_loss = False

    def __init__(self, ctx):
        super().__init__(ctx)
        self.model = MapModel()
        self.monitor = LaserMonitor()
        self.session = None
        self._retired_sessions = []
        self.bundle = None
        self.goal = None
        self.active = self.closed = self.engaged = False
        self._identity = self._car_identity()
        self._was_connected = False
        self._loads = queue.Queue()
        self._load_generation = 0
        self._loading = False
        self._navigate_after_plan = None
        self._planned_goal = None
        self._record = None
        self._last_frame = None
        self._last_goal_state = ""
        self._reported_paths = set()
        self._stop_requested = False
        self._map_label_text = "未导入地图"
        self._status_text = "导入已保存的 map.yaml，再连接并启动定位；点选目标后开始导航。"
        self._status_error = False

    @property
    def motion_active(self):
        return bool(self._navigate_after_plan is not None or
                    (self.session and self.session.snapshot().get("motion_active")) or
                    any(session.snapshot().get("motion_active") for session in self._retired_sessions))

    def _car_identity(self):
        car = self.ctx.cfg["car"]
        return car.get("profile_id"), car.get("host"), car.get("transport")

    def build(self, parent):
        self.parent = parent
        panel = tk.Frame(parent, bg=theme.PANEL, padx=8, pady=4)
        panel.pack(fill="x", padx=6, pady=4)
        row = tk.Frame(panel, bg=theme.PANEL)
        row.pack(fill="x")
        self.btn_import = self._button(row, "导入地图（YAML）", self.import_map, theme.ACCENT)
        self.map_label = tk.Label(row, text="未导入地图", bg=theme.PANEL, fg=theme.TEXT_DIM,
                                  anchor="w", font=theme.FONT_SMALL)
        self.map_label.pack(side="left", fill="x", expand=True, padx=6)
        row = tk.Frame(panel, bg=theme.PANEL)
        row.pack(fill="x")
        self.peer = tk.Label(row, text="当前车：%s" % self.ctx.cfg["car"]["host"],
                             bg=theme.PANEL, fg=theme.ACCENT, font=theme.FONT_SMALL)
        self.peer.pack(side="left", padx=4)
        self.username = tk.StringVar(value=self.ctx.cfg.get("terminal", {}).get("username", "pi"))
        self.password = tk.StringVar(value=local_password())
        for label, variable, width, show in (("SSH 用户", self.username, 8, ""),
                                              ("密码", self.password, 12, "●")):
            tk.Label(row, text=label, bg=theme.PANEL, fg=theme.TEXT_DIM).pack(side="left")
            tk.Entry(row, textvariable=variable, width=width, show=show, bg=theme.PANEL_2,
                     fg=theme.TEXT, insertbackground=theme.TEXT, relief="flat").pack(side="left", padx=4)
        self.btn_connect = self._button(row, "连接 / 检查", self.connect)
        self._button(row, "断开", self.disconnect)
        self.btn_start = self._button(row, "启动导航定位", self.start, theme.OK)
        self._button(row, "结束导航服务", self.finish)
        row = tk.Frame(panel, bg=theme.PANEL)
        row.pack(fill="x", pady=2)
        self.mode = tk.StringVar(value="goal")
        for label, value in (("标记目标", "goal"), ("标定当前位置 / 朝向", "initial"), ("平移地图", "pan")):
            tk.Radiobutton(row, text=label, variable=self.mode, value=value,
                           command=lambda: self.map_panel.set_mode(self.mode.get()),
                           bg=theme.PANEL, fg=theme.TEXT, selectcolor=theme.BG,
                           activebackground=theme.PANEL, font=theme.FONT_SMALL).pack(side="left")
        tk.Label(row, text="速度（m/s）", bg=theme.PANEL, fg=theme.TEXT_DIM).pack(side="left", padx=(8, 2))
        self.speed = tk.StringVar(value=str(self.ctx.cfg.get("navigation", {}).get("speed", .06)))
        tk.Spinbox(row, textvariable=self.speed, from_=.02, to=.15, increment=.01,
                   format="%.2f", width=5, bg=theme.PANEL_2, fg=theme.TEXT,
                   buttonbackground=theme.PANEL_2, relief="flat").pack(side="left")
        self.btn_plan = self._button(row, "仅规划路径", self.plan)
        self.btn_navigate = self._button(row, "▶ 开始导航", self.navigate, theme.OK)
        self._button(row, "■ 停止导航", self.stop_all, theme.ERR)
        self.hint = tk.Label(panel, text="先结束建图。标定或目标模式：左键点选位置，按住拖动设置朝向；右键拖动平移，滚轮缩放。"
                 "导航期间切页可继续；顶部“停止全部”随时取消。速度在启动服务时生效。",
                 bg=theme.PANEL, fg=theme.TEXT_DIM, anchor="w", justify="left", wraplength=1100, font=theme.FONT_SMALL)
        self.hint.pack(fill="x")
        self.status = tk.Label(panel, text=self._status_text, anchor="w", justify="left",
                               bg=theme.PANEL, fg=theme.ACCENT, font=theme.FONT_SMALL)
        self.status.pack(fill="x", pady=(3, 0))
        split = ttk.Panedwindow(parent, orient="horizontal")
        split.pack(fill="both", expand=True, padx=6, pady=(0, 4))
        self.map_panel = NavigationMapPanel(split, self.model, on_pose=self.select_pose)
        self.map_panel.set_localization(False, "请启动导航定位，并标定实际位置与朝向")
        self.cam = MappingCamera(split, self.ctx)
        split.add(self.map_panel, weight=1)
        split.add(self.cam, weight=1)
        self.ctx.add_frame_listener(self._on_frame)

    @staticmethod
    def _button(parent, text, command, color=None):
        button = tk.Button(parent, text=text, command=command, bg=theme.PANEL_2,
                           fg=color or theme.TEXT, relief="flat", activebackground=theme.BORDER,
                           font=theme.FONT_SMALL, padx=7, pady=4)
        button.pack(side="left", padx=3, pady=2)
        return button

    def _notice(self, text, error=False):
        self._status_text, self._status_error = str(text), bool(error)
        self.ctx.log("沙盘导航：" + str(text))
        if self._record:
            try:
                self._record.event(str(text))
            except OSError as exc:
                self.ctx.log("导航记录写入失败：" + str(exc))
                self.stop_all()

    def import_map(self, path=None):
        remote = self.session.snapshot() if self.session else {}
        if self.motion_active or remote.get("running") or remote.get("busy"):
            self._notice("请先停止导航并结束导航服务，再更换地图。", True)
            return False
        if not path:
            path = filedialog.askopenfilename(parent=self.ctx.root, title="选择已保存的地图 YAML",
                initialdir=str(Path(APP_DIR) / "savedMap"), filetypes=[("ROS 地图", "*.yaml *.yml")])
        if not path:
            return False
        self._load_generation += 1
        generation = self._load_generation
        self._loading = True
        def work():
            try:
                bundle = load_navigation_map(path)
                model = MapModel()
                if not model.accept_map(bundle.grid_message, time.monotonic()):
                    raise ValueError(model.snapshot().get("error"))
                result, error = (bundle, model), None
            except Exception as exc:
                result, error = None, str(exc)
            self._loads.put((generation, result, error))
        threading.Thread(target=work, name="navigation-map-import", daemon=True).start()
        self._notice("正在读取地图…")
        return True

    def _install_map(self, bundle, model):
        self.bundle, self.goal = bundle, None
        self._planned_goal = None
        self.model = model
        self.map_panel.set_model(model)
        self.map_panel.set_goal(None)
        self.map_panel.set_plan([])
        self.map_panel.set_initial(None)
        self.map_panel.reset_view()
        self._map_label_text = "%s | %d × %d | %.3f m/格" % (
            bundle.source_path.name, bundle.width, bundle.height, bundle.resolution)
        self._notice("地图已导入；连接并启动导航定位后，先标定小车实际位置和朝向。")

    def connect(self):
        if self._loading or self.motion_active or self.ctx.motion_in_use(exclude=self):
            self._notice("请等待地图导入或停车完成，并停止其他运动后再连接。", True)
            return
        self.disconnect()
        car = self.ctx.cfg["car"]
        guard = None
        if car.get("transport") == "wifi":
            from carapp.core.connection import require_wifi
            profile = next((p for p in self.ctx.cfg.get("cars", []) if p.get("id") == car.get("profile_id")), {})
            guard = lambda: require_wifi(profile.get("ssid"))
        try:
            remote = NavigationRemote(car["host"], self.username.get().strip(), self.password.get(),
                port=int(self.ctx.cfg.get("terminal", {}).get("port", 22)), profile_id=car.get("profile_id", ""),
                known_hosts=os.path.join(APP_DIR, "config", "ssh_known_hosts"), connection_guard=guard)
            self.session = NavigationSession(remote)
            self.session.submit("connect")
            self._notice("正在检查车端 Nav2、定位和传感器…")
        except Exception as exc:
            self._notice(str(exc), True)

    def _reset_pose(self):
        self.model.reset(keep_map=True)
        if hasattr(self, "map_panel"):
            self.map_panel.set_localization(False, "请启动导航定位，并标定实际位置与朝向")

    def disconnect(self):
        self.stop_all()
        if self.session:
            retired = self.session
            retired.close_async()
            self._retired_sessions.append(retired)
            self.session = None
        self.engaged = False
        self._last_frame = None
        self._reset_pose()
        self.monitor.reset()
        if not self.motion_active:
            self._finish_record()

    def start(self):
        remote = self.session.snapshot() if self.session else {}
        st = self.ctx.state.snapshot()
        if not self.bundle or self._loading:
            return self._notice("请先导入地图。", True)
        if not remote.get("connected") or not st.get("ws_connected"):
            return self._notice("请先连接 ROS 和导航服务。", True)
        if remote.get("running") or self.motion_active or self.ctx.motion_in_use(exclude=self):
            return self._notice("请先停止其他运动；导航服务已启动时请先结束服务。", True)
        if not st.get("cam_connected"):
            return self._notice("请等待实时画面恢复，以便直接在电脑记录导航。", True)
        try:
            speed = float(self.speed.get())
            if not math.isfinite(speed) or not .02 <= speed <= .15:
                raise ValueError("导航速度需在 0.02–0.15 m/s")
            self.ctx.cfg.setdefault("navigation", {})["speed"] = speed
            save_config(self.ctx.cfg)
            upload = self.bundle.as_upload()
            self._start_record()
            if not self.session.submit("start", upload, speed=speed):
                self._finish_record()
                return self._notice("导航服务正在处理操作，请稍后重试。", True)
            self._reset_pose()
            self._notice("正在上传地图并启动定位；此步骤不会驱动车轮。")
        except Exception as exc:
            self._finish_record()
            self._notice(str(exc), True)

    def finish(self):
        self.engaged = False
        self._navigate_after_plan = None
        if self.session:
            self.session.request_cancel(stop=True)
        self.ctx.stop_base()
        self._notice("已请求结束导航服务，等待车端确认停止。")

    def select_pose(self, mode, x, y, yaw):
        if self.motion_active:
            return self._notice("请先停止当前导航，再修改目标或定位。", True)
        if not self.bundle or not valid_target(self.bundle, x, y) or not math.isfinite(yaw):
            return self._notice("请选择地图内的空闲区域；障碍物和未知区域不能作为位置。", True)
        pose = float(x), float(y), float(yaw)
        if mode == "initial":
            remote = self.session.snapshot() if self.session else {}
            if not self.engaged or not remote.get("running") or self.ctx.motion_in_use(exclude=self):
                return self._notice("先启动导航定位，并停止其他运动，再标定当前位置。", True)
            if self.session.submit("initial_pose", *pose):
                self._reset_pose()
                self.map_panel.set_initial(pose)
                self.map_panel.set_plan([])
                self._notice("已发送当前位置和朝向，等待雷达定位确认。")
        else:
            self.goal = pose
            self._planned_goal = None
            self.map_panel.set_goal(pose)
            self.map_panel.set_plan([])
            self._notice("目标：%.2f, %.2f m / %.0f°；可预览路径或点击开始导航。" %
                         (pose[0], pose[1], math.degrees(pose[2])))
            if self._ready():
                self.plan()

    def _ready(self):
        remote = self.session.snapshot() if self.session else {}
        scan, now = self.monitor.snapshot(), time.monotonic()
        return bool(self.engaged and remote.get("connected") and remote.get("running")
            and remote.get("ready") and self.ctx.state.get("ws_connected")
            and self.ctx.state.get("cam_connected") and self._record
            and self._record.camera is self.ctx.app.camera and self._record.camera.recording
            and not scan.get("error") and now - scan.get("received_at", 0) < .8
            and not self.ctx.motion_in_use(exclude=self))

    def plan(self):
        if not self.goal or not self._ready() or self.motion_active:
            self._notice("规划未就绪：请先导入地图、启动服务、确认定位并标记空闲目标。", True)
            return False
        accepted = self.session.submit("plan", *self.goal)
        if accepted:
            self._notice("正在计算可通行路径…")
        return accepted

    def navigate(self):
        if not self.plan():
            return False
        self._stop_requested = False
        self._navigate_after_plan = self.goal
        return True

    def stop_all(self):
        self._navigate_after_plan = None
        self._stop_requested = True
        if self.session:
            self.session.request_cancel()
        self.ctx.stop_base()

    def ros_subscriptions(self):
        return {"/tf": 100, "/tf_static": 0, "/scan_raw": 100}

    def realtime_ros_handlers(self):
        return {"/scan_raw": self._accept_scan}

    def _accept_scan(self, msg, received_at):
        if not self.closed:
            self.monitor.update(msg, received_at)

    def reset_realtime_ros(self):
        self.monitor.reset()

    def on_ros_message(self, topic, msg, received_at):
        if self.closed or not self.engaged:
            return
        if topic in ("/tf", "/tf_static"):
            self.model.accept_tf(msg, received_at, static=topic == "/tf_static")
        elif topic == "/odom":
            self.model.accept_odom(msg, received_at)

    def on_activate(self):
        self.active = True

    def on_deactivate(self):
        self.active = False

    def on_vehicle_change(self):
        self.disconnect()
        self.goal = None
        self.map_panel.set_goal(None)
        self.map_panel.set_plan([])
        self.map_panel.set_initial(None)
        self._identity = self._car_identity()

    def on_background_tick(self, st):
        if self.closed:
            return
        self._retired_sessions = [session for session in self._retired_sessions
                                  if session.snapshot().get("busy") or session.snapshot().get("motion_active")]
        if self._identity != self._car_identity():
            self.on_vehicle_change()
        connected = bool(st.get("ws_connected"))
        if self._was_connected and not connected:
            self.disconnect()
            self._notice("ROS 已断开，导航已取消；恢复连接后需重新手动启动。", True)
        self._was_connected = connected
        while True:
            try:
                generation, bundle, error = self._loads.get_nowait()
            except queue.Empty:
                break
            if generation != self._load_generation:
                continue
            self._loading = False
            if error:
                self._notice("地图导入失败：" + error, True)
            else:
                self._install_map(*bundle)
        if self.session:
            for event in self.session.drain():
                self._handle_event(event)
        if self._record and self._record.camera is not self.ctx.app.camera:
            self.finish()
            self._finish_record()
            self._notice("摄像头连接已更换，已结束本次导航；请等待新画面后重新启动定位。", True)
        if self.motion_active and not self._stop_requested and not self._ready():
            self.stop_all()
            self._notice("定位、雷达、画面或运动许可失效，已请求取消导航。", True)
        remote = self.session.snapshot() if self.session else {}
        status = dict(navigation_running=self.motion_active,
                      navigation_status=remote.get("goal_state", "未启动") if self.engaged else "未启动")
        st.update(status)
        self.ctx.state.update(**status)
        if self._record and (not self.engaged and not remote.get("busy") and not remote.get("running")):
            self._finish_record()

    def _handle_event(self, event):
        if event.get("stale") or event.get("canceled"):
            return
        op, result, error = event["operation"], event.get("result") or {}, event.get("error")
        if error:
            self.stop_all()
            self._notice("%s 失败：%s" % (op, error), True)
            if op in ("start", "connect", "status"):
                self.engaged = False
                self.model.clear_map_pose()
                self.map_panel.set_localization(False, "定位服务未启动或连接失效，请查看上方状态")
            return
        if self._record:
            paths = result.get("temporary_paths") or result.get("remote_files") or []
            paths = [path for path in paths if path not in self._reported_paths]
            if paths:
                try:
                    self._record.event("车端导航临时文件", remote_paths=paths)
                    self._reported_paths.update(paths)
                    for path in paths:
                        self.ctx.log("车端导航临时文件：" + path)
                except OSError:
                    self.stop_all()
        if op == "start":
            self.engaged = bool(result.get("running"))
            self._stop_requested = False
            self._notice("导航定位已启动；请选择“标定当前位置 / 朝向”，在地图上按住拖出车头方向。")
        elif op == "connect":
            self._notice(result.get("detail", "检查完成，请导入地图并启动导航定位。"))
        elif op == "plan":
            target = event.get("target")
            if target is None or tuple(target) != self.goal:
                return
            path = result.get("path", [])
            self.map_panel.set_plan(path)
            self._planned_goal = self.goal if path else None
            pending, self._navigate_after_plan = self._navigate_after_plan, None
            self._notice("路径已规划，长度约 %.2f m。" % float(result.get("length", 0)))
            if pending and pending == self.goal and path and self._ready():
                if not self.session.submit("navigate", *pending):
                    self._notice("导航请求未提交，服务正在处理操作；请再次点击开始导航。", True)
        elif op == "initial_pose":
            self._notice("当前位置已提交；等待定位就绪后，标记目标。")
        elif op == "stop":
            self.engaged = False
            self._notice("导航服务已结束。")
        elif op == "cancel":
            self._notice("导航已取消；再次移动需点击开始导航。")
        elif op == "status" and not result.get("running", True):
            self.engaged = False
            self._notice("导航服务已停止；需手动重新启动定位。", True)
        if result.get("connected") is False:
            self.engaged = False
            self.stop_all()
        if op in ("start", "status", "initial_pose", "stop"):
            self._accept_navigation_pose(event)
        if "path" in result and op == "status" and (self.motion_active or self._planned_goal == self.goal):
            self.map_panel.set_plan(result["path"])
        phase = result.get("goal_state")
        if phase and phase != self._last_goal_state:
            self._last_goal_state = phase
            labels = {"navigating": "正在前往目标", "succeeded": "已到达目标", "canceled": "导航已取消",
                      "aborted": "导航失败，车辆已停", "error": "导航异常，车辆已停"}
            if phase in labels:
                self._notice(labels[phase], phase in ("aborted", "error"))
                self._keyframe(phase)

    def _accept_navigation_pose(self, event):
        """Show the car-side map localization even if the UI missed a TF edge."""
        result = event.get("result") or {}
        now = time.monotonic()
        try:
            received_at = float(event.get("received_at", now))
            age = float(result.get("pose_age", 0.))
            valid_age = math.isfinite(age) and math.isfinite(received_at) and 0. <= age <= self.model.POSE_TIMEOUT
            valid_age = valid_age and 0. <= now - received_at + age <= self.model.POSE_TIMEOUT
            confirmed = self.engaged and result.get("localized") is True and valid_age
            accepted = confirmed and self.model.accept_map_pose(result.get("pose"), received_at-age,
                frame_id=result.get("pose_frame", "map"))
        except (TypeError, ValueError, OverflowError):
            accepted = False
        if accepted:
            self.map_panel.set_localization(True)
        else:
            self.model.clear_map_pose()
            detail = "请启动导航定位，并标定实际位置与朝向" if not self.engaged else "等待雷达确认定位；标定点不是实时位置"
            self.map_panel.set_localization(False, detail)

    def on_tick(self, st):
        if self.closed:
            return
        width = max(300, self.parent.winfo_width() - 30)
        self.hint.config(wraplength=width)
        self.status.config(text=self._status_text, fg=theme.ERR if self._status_error else theme.ACCENT,
                           wraplength=width)
        self.map_label.config(text=self._map_label_text)
        self.peer.config(text="当前车：%s" % self.ctx.cfg["car"]["host"])
        self.btn_import.config(state="disabled" if self._loading or self.engaged or self.motion_active else "normal")
        self.btn_navigate.config(state="normal" if self.goal and self._ready() and not self.motion_active else "disabled")
        self.map_panel.on_tick(st)
        self.cam.on_tick(st)

    def _on_frame(self, timestamp, payload, mode):
        self._last_frame = timestamp, payload, mode

    def _start_record(self):
        self._finish_record()
        self._reported_paths.clear()
        folder = self.ctx.save_dir("沙盘导航/" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
        self._record = NavigationRecord(folder, self.ctx.app.camera, self._identity)
        self._keyframe("导航准备")
        self.ctx.log("导航录像和关键帧直接保存在电脑：" + str(self._record.folder))

    def _keyframe(self, label):
        if self._record and self._last_frame:
            try:
                self._record.frame(label, self._last_frame)
            except OSError:
                self.stop_all()

    def _finish_record(self):
        record, self._record = self._record, None
        if record:
            try:
                record.frame("导航结束", self._last_frame)
            except OSError as exc:
                self.ctx.log("导航记录写入失败：" + str(exc))
            finally:
                try:
                    record.finish()
                except OSError as exc:
                    self.ctx.log("导航记录写入失败：" + str(exc))

    def on_close(self):
        self.closed = True
        self._load_generation += 1
        self.disconnect()
        self._finish_record()
        self.map_panel.on_close()
        self.cam.on_close()
