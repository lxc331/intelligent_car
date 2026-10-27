"""Sandbox mapping page: orchestration, UI and existing public control interfaces."""
import copy
import os
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import ttk

from carapp.config import APP_DIR, save_config
from carapp.core.credentials import local_password
from carapp.core.map_data import MapModel
from carapp.core.mapping_remote import MappingRemote
from carapp.core.mapping_session import MapFeed, MappingSession
from carapp.core.mapping_record import MappingRecord
from carapp.features.base import Feature
from carapp.ui import theme
from carapp.ui.mapping_camera import MappingCamera
from carapp.ui.mappanel import MapPanel
from carapp.ui.remotedirectory import RemoteDirectory
from carapp.ui.teleoppanel import TeleopPanel


class MappingFeature(Feature):
    key = "mapping"
    title = "沙盘建模 + 实时展示"
    SCOPES = {"ClawPi 容器": "container", "小车宿主机": "host"}

    def __init__(self, ctx):
        super().__init__(ctx)
        self.model = MapModel()
        self.feed = MapFeed(self.model)
        self.session = None
        self.active = False
        self.closed = False
        self.browser = None
        self._identity = None
        self._last_poll = 0.
        self._was_connected = False
        self._record = None
        self._last_frame = None
        self._scan_at = 0.
        self._record_error = False
        self.engaged = False

    @property
    def motion_active(self):
        return hasattr(self, "teleop") and self.teleop.motion_active

    def build(self, parent):
        self.parent = parent
        cfg = self.ctx.cfg.get("mapping", {})
        terminal = self.ctx.cfg.get("terminal", {})
        panel = tk.Frame(parent, bg=theme.PANEL, padx=8, pady=3)
        panel.pack(fill="x", padx=6, pady=4)
        connection = tk.Frame(panel, bg=theme.PANEL)
        connection.pack(fill="x")
        self.peer = tk.Label(connection, bg=theme.PANEL, fg=theme.ACCENT, font=theme.FONT_SMALL)
        self.peer.pack(side="left", padx=(0, 8))
        self.username = tk.StringVar(value=terminal.get("username", "pi"))
        self.password = tk.StringVar(value=local_password())
        for text, variable, width, show in (("SSH 用户", self.username, 8, ""),
                                             ("密码", self.password, 13, "●")):
            tk.Label(connection, text=text, bg=theme.PANEL, fg=theme.TEXT_DIM).pack(side="left")
            tk.Entry(connection, textvariable=variable, width=width, show=show, bg=theme.PANEL_2,
                     fg=theme.TEXT, insertbackground=theme.TEXT, relief="flat").pack(side="left", padx=4)
        self.btn_connect = self._button(connection, "连接 / 检查", self.connect)
        self.btn_disconnect = self._button(connection, "断开", self.disconnect)
        actions = tk.Frame(panel, bg=theme.PANEL)
        actions.pack(fill="x", pady=2)
        self.btn_start = self._button(actions, "▶ 开始建图", self.start, theme.OK)
        self.btn_stop = self._button(actions, "■ 结束建图", self.finish, theme.ERR)
        self.btn_save = self._button(actions, "保存到小车和电脑", self.save_map, theme.ACCENT)
        self._button(actions, "打开本地地图目录", self.open_local)
        tk.Label(actions, text="完成后先保存，再结束建图。", bg=theme.PANEL,
                 fg=theme.TEXT_DIM, font=theme.FONT_SMALL).pack(side="left", padx=8)
        save = tk.Frame(panel, bg=theme.PANEL)
        save.pack(fill="x")
        self.scope = tk.StringVar(value=next((key for key, val in self.SCOPES.items()
                                              if val == cfg.get("scope")), "ClawPi 容器"))
        ttk.Combobox(save, textvariable=self.scope, values=list(self.SCOPES), width=13,
                     state="readonly").pack(side="left", padx=(0, 6))
        self.remote_dir = tk.StringVar(value=cfg.get("remote_dir", "/home/ubuntu/ros2_ws/src/slam/maps"))
        tk.Entry(save, textvariable=self.remote_dir, bg=theme.PANEL_2, fg=theme.TEXT,
                 insertbackground=theme.TEXT, relief="flat").pack(side="left", fill="x", expand=True)
        self.btn_browse = self._button(save, "选择目录", self.browse)
        tk.Label(save, text="地图名", bg=theme.PANEL, fg=theme.TEXT_DIM).pack(side="left", padx=4)
        self.map_name = tk.StringVar(value=self.new_map_name())
        tk.Entry(save, textvariable=self.map_name, width=24, bg=theme.PANEL_2, fg=theme.TEXT,
                 insertbackground=theme.TEXT, relief="flat").pack(side="left")
        self.serialize = tk.BooleanVar(value=bool(cfg.get("serialize", True)))
        self.status = tk.Label(panel, text="未连接建图服务；二维雷达建图，摄像头用于辅助观察。",
                               bg=theme.PANEL, fg=theme.TEXT_DIM, anchor="w", font=theme.FONT_SMALL)
        self.status.pack(fill="x", pady=(3, 0))
        self.teleop = TeleopPanel(parent, self.ctx, ready=self._ready)
        self.teleop.pack(fill="x", padx=6, pady=(0, 4))
        self.split = ttk.Panedwindow(parent, orient="horizontal")
        self.split.pack(fill="both", expand=True, padx=6, pady=(0, 4))
        self.map_panel = MapPanel(self.split, self.model)
        self.cam = MappingCamera(self.split, self.ctx)
        self.split.add(self.map_panel, weight=1)
        self.split.add(self.cam, weight=1)
        self.local_root = Path(APP_DIR) / "savedMap"
        self.local_root.mkdir(parents=True, exist_ok=True)
        self.ctx.add_frame_listener(self._on_frame)
        self._update_peer()

    @staticmethod
    def new_map_name():
        return "sandbox_" + datetime.now().strftime("%Y%m%d_%H%M%S")

    def _button(self, parent, text, command, color=None):
        button = tk.Button(parent, text=text, command=command, bg=theme.PANEL_2,
                           fg=color or theme.TEXT, activebackground=theme.BORDER,
                           relief="flat", font=theme.FONT_SMALL, padx=8, pady=4)
        button.pack(side="left", padx=3, pady=2)
        return button

    def _update_peer(self):
        car = self.ctx.cfg["car"]
        self._identity = (car.get("profile_id"), car.get("host"), car.get("transport"))
        self.peer.config(text="当前车：%s" % car.get("host", "--"))

    def _notice(self, text, error=False):
        self.status.config(text=text, fg=theme.ERR if error else theme.ACCENT)
        self.ctx.log("沙盘建图：" + text)
        if self._record:
            try:
                self._record.event(text)
            except OSError as exc:
                self._record_error = True
                self.stop_all()
                self.ctx.log("建图记录写入失败：" + str(exc))

    def _guard(self):
        car = self.ctx.cfg["car"]
        if car.get("transport") == "wifi":
            from carapp.core.connection import require_wifi
            profile = next((p for p in self.ctx.cfg.get("cars", [])
                            if p.get("id") == car.get("profile_id")), {})
            return lambda: require_wifi(profile.get("ssid"))
        return None

    def connect(self):
        if self.session and self._blocking_operation(self.session.snapshot()):
            return
        self.disconnect()
        car = self.ctx.cfg["car"]
        try:
            remote = MappingRemote(car["host"], self.username.get().strip(), self.password.get(),
                                   port=int(self.ctx.cfg.get("terminal", {}).get("port", 22)),
                                   profile_id=car.get("profile_id", ""),
                                   known_hosts=os.path.join(APP_DIR, "config", "ssh_known_hosts"),
                                   connection_guard=self._guard())
            self.session = MappingSession(remote)
            self.session.submit("connect")
            self._notice("正在检查车端 SLAM、雷达、里程计和坐标变换…")
        except Exception as exc:
            self._notice(str(exc), True)

    def disconnect(self):
        self.engaged = False
        self.stop_all()
        if self.session:
            self.session.close_async()
            self.session = None
        self._finish_record()
        self.feed.reset()
        self._last_frame = None
        self._scan_at = 0.
        if self.browser and self.browser.winfo_exists():
            self.browser.destroy()
        self.browser = None

    def start(self):
        if not self.active or not self.session or not self.ctx.state.get("ws_connected"):
            self._notice("请先连接 ROS 和建图服务，再点击开始建图。", True)
            return
        if self.ctx.motion_in_use(exclude=self):
            self._notice("请先停止其他底盘或夹取动作。", True)
            return
        state = self.session.snapshot()
        if not state["connected"]:
            self._notice("建图 SSH 未连接，请先点击连接 / 检查。", True)
            return
        if self._blocking_operation(state):
            self._notice("建图操作正在执行，请等待完成。")
            return
        if self.engaged:
            return
        if not self.ctx.state.get("cam_connected"):
            self._notice("请等待实时画面就绪，以记录本次建图操作。", True)
            return
        self.stop_all()
        try:
            self._start_record()
            if self.session.submit("start"):
                self.feed.reset()
                self._notice("正在启动车端 SLAM；请等待地图就绪后遥控。")
            else:
                self._finish_record()
                self._notice("建图服务尚未就绪，请稍后重新开始。", True)
        except Exception as exc:
            self._finish_record()
            self._notice(str(exc), True)

    def finish(self):
        self.engaged = False
        self.stop_all()
        if self.session and self.session.submit("stop"):
            self._notice("正在结束本页面启动的建图；外部建图进程不会被终止。")

    def save_map(self):
        self.stop_all()
        if not self.session or not self.session.snapshot()["running"]:
            self._notice("请先启动建图并等待有效地图。", True)
            return
        snap = self.model.snapshot()
        if not snap.get("grid"):
            self._notice("尚未收到有效地图，不能保存。", True)
            return
        scope = self.SCOPES[self.scope.get()]
        if self.session.submit("save_map", self.remote_dir.get().strip(), self.map_name.get().strip(),
                               str(self.local_root), scope=scope, serialize=self.serialize.get()):
            self._persist_preferences(scope)
            self._notice("正在保存车端地图并下载校验，期间遥控暂停…")

    def _persist_preferences(self, scope):
        old = copy.deepcopy(self.ctx.cfg.get("mapping", {}))
        self.ctx.cfg["mapping"] = dict(remote_dir=self.remote_dir.get().strip(), scope=scope,
                                       serialize=self.serialize.get())
        try:
            save_config(self.ctx.cfg)
        except OSError as exc:
            self.ctx.cfg["mapping"] = old
            self.ctx.log("建图参数未写入配置：%s" % exc)

    def browse(self):
        if not self.session or not self.session.snapshot()["connected"]:
            self._notice("先连接建图服务后可浏览小车目录。", True)
            return
        if self.browser and self.browser.winfo_exists():
            self.browser.lift()
            return
        self.stop_all()
        self.browser = RemoteDirectory(self.parent, self._list_directory, self._choose_directory,
                                       self.remote_dir.get(), self.SCOPES[self.scope.get()])

    def _list_directory(self, path, scope):
        return bool(self.session and self.session.submit("list_directories", path, scope=scope))

    def _choose_directory(self, path, scope):
        self.remote_dir.set(path)
        self.scope.set(next(label for label, value in self.SCOPES.items() if value == scope))

    def open_local(self):
        if hasattr(os, "startfile"):
            os.startfile(str(self.local_root))
        else:
            self.ctx.log("本地地图目录：" + str(self.local_root))

    def _ready(self):
        remote = self.session.snapshot() if self.session else {}
        snap = self.model.snapshot()
        grid = snap.get("grid")
        return bool(self.active and self.engaged and remote.get("connected") and remote.get("running")
                    and (not remote.get("busy") or remote.get("operation") == "status")
                    and self.ctx.state.get("ws_connected") and self.ctx.state.get("cam_connected")
                    and grid and snap.get("map_age", float("inf")) < 10.
                    and snap.get("pose") and not snap.get("pose_stale", True)
                    and time.monotonic() - self._scan_at < 1.
                    and not self._record_error
                    and (not self._record or bool(self._record.camera and self._record.camera.recording))
                    and not self.ctx.motion_in_use(exclude=self))

    def stop_all(self):
        if hasattr(self, "teleop"):
            self.teleop.stop()

    def ros_subscriptions(self):
        return {"/map": 1000, "/tf": 100, "/tf_static": 0, "/scan_raw": 100}

    def ros_subscription_types(self):
        return {"/map": "nav_msgs/msg/OccupancyGrid"}

    def realtime_ros_handlers(self):
        return {"/scan_raw": self._accept_scan}

    def reset_realtime_ros(self):
        self._scan_at = 0.

    def _accept_scan(self, msg, received_at):
        if not self.closed and isinstance(msg, dict) and isinstance(msg.get("ranges"), list) and msg["ranges"]:
            self._scan_at = received_at

    def on_ros_message(self, topic, msg, received_at):
        if self.closed:
            return
        if topic == "/map":
            self.feed.put(msg, received_at)
        elif topic in ("/tf", "/tf_static"):
            self.model.accept_tf(msg, received_at, static=topic == "/tf_static")
        elif topic == "/odom":
            self.model.accept_odom(msg, received_at)
        elif topic == "/scan_raw":
            self._accept_scan(msg, received_at)

    def on_activate(self):
        self.active = True
        self.teleop.set_active(True)

    def on_deactivate(self):
        self.active = False
        self.teleop.set_active(False)

    def on_vehicle_change(self):
        self.disconnect()
        self._identity = None

    def on_background_tick(self, st):
        if self.closed:
            return
        car = self.ctx.cfg["car"]
        identity = (car.get("profile_id"), car.get("host"), car.get("transport"))
        if identity != self._identity:
            self.disconnect()
            self._update_peer()
            self._notice("已切换车辆，建图服务需重新连接。")
        connected = bool(st.get("ws_connected"))
        if self._was_connected and not connected:
            self.disconnect()
            self._notice("ROS 已断开；本次建图不自动恢复。", True)
        self._was_connected = connected
        if self._record and self._record.camera is not self.ctx.app.camera:
            self.engaged = False
            self.stop_all()
            self._finish_record()
            self._notice("摄像头连接已更换；请等待新画面后点击开始建图，重新接入并记录。")
        if self.session:
            for event in self.session.drain():
                self._handle_event(event)
            remote = self.session.snapshot()
            now = time.monotonic()
            if remote["connected"] and not remote["busy"] and now - self._last_poll > 4:
                self._last_poll = now
                self.session.submit("status")
        self.teleop.on_background_tick(st)

    @staticmethod
    def _blocking_operation(remote):
        """Routine status checks do not lock user controls; queued actions do."""
        return remote.get("busy", False) and remote.get("operation") != "status"

    def on_tick(self, st):
        if self.closed:
            return
        if self.session:
            remote = self.session.snapshot()
            controls = "disabled" if self._blocking_operation(remote) else "normal"
            for button in (self.btn_connect, self.btn_start, self.btn_stop, self.btn_save, self.btn_browse):
                button.config(state=controls)
        else:
            for button in (self.btn_connect, self.btn_start, self.btn_stop, self.btn_save, self.btn_browse):
                button.config(state="normal")
        self.teleop.on_tick(st)
        self.map_panel.on_tick(st)
        self.cam.on_tick(st)

    def _handle_event(self, event):
        operation, result, error = event["operation"], event["result"], event["error"]
        if operation in ("connect", "status"):
            # Measure the polling gap from completion, not submission. A slow
            # query must not immediately schedule another one and starve input.
            self._last_poll = time.monotonic()
        if operation == "list_directories":
            if self.browser and self.browser.winfo_exists():
                self.browser.update_listing(result, error)
            return
        if error:
            self.stop_all()
            partial = result.get("remote_dir") or result.get("remote_path")
            self._notice("%s失败：%s%s" % (operation, error,
                         "；车端已保存：" + partial if partial else ""), True)
            if operation in ("start", "status"):
                self.engaged = False
                self._finish_record()
            return
        if self._record:
            paths = list(result.get("temporary_paths", []))
            if result.get("remote_dir"):
                paths += [result["remote_dir"] + "/" + item["name"] for item in result.get("files", [])]
            try:
                self._record.event(operation, remote_paths=paths)
            except OSError as exc:
                self._record_error = True
                self.stop_all()
                self.ctx.log("建图记录写入失败：" + str(exc))
        if operation == "save_map":
            label = "双端保存完成" if result.get("serialized") else "双端地图保存完成（未保存继续建图状态）"
            self._notice("%s：电脑 %s；小车 %s" % (label, result.get("local_dir"), result.get("remote_dir")))
            for warning in result.get("warnings", []):
                self.ctx.log("建图保存提示：" + warning)
            self.map_name.set(self.new_map_name())
            self._keyframe("地图已保存")
        elif operation == "start":
            self.engaged = True
            # Retry after SLAM is available as well as pre-subscribing with a
            # type: the initial request may have predated the /map publisher.
            self.ctx.bridge.subscribe("/map", throttle_ms=1000,
                                      message_type=self.ros_subscription_types()["/map"])
            self._notice("建图已启动，等待地图；请低速遥控扫描沙盘。" if result.get("owned") else
                         "已接入车端已有建图，本页面结束操作不会终止外部进程。")
            self._keyframe("开始建图")
            for path in result.get("temporary_paths", []):
                self.ctx.log("车端临时建图文件：" + path)
        elif operation == "stop":
            if result.get("running"):
                self._notice("建图进程尚未停止，请重试结束或断开连接等待租约结束。", True)
                return
            self._notice("已结束本页面启动的建图；地图画面保留。" if result.get("status") == "stopped" else
                         "已退出本页面建图；外部建图进程保留。")
            self._finish_record()
        elif operation == "connect":
            self._notice(result.get("detail", "检查完成，可以开始建图。"))
            for warning in result.get("warnings", []):
                self._notice("车端检查提示：" + warning, True)
        elif operation == "status" and not result.get("running") and self._record:
            self.engaged = False
            self.stop_all()
            self._notice("建图节点已停止，请检查车端后手动开始。", True)
            self._finish_record()

    def _start_record(self):
        self._finish_record()
        camera = self.ctx.app.camera
        self._record_error = False
        folder = self.ctx.save_dir("沙盘建图/" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
        self._record = MappingRecord(folder, camera, self._identity)
        self._keyframe("建图准备")
        self.ctx.log("建图操作记录直接保存在电脑：" + str(self._record.folder))

    def _keyframe(self, label):
        frame = self._last_frame
        if self._record and frame:
            self._record.frame(label, frame)

    def _on_frame(self, timestamp, payload, mode):
        self._last_frame = (timestamp, payload, mode)

    def _finish_record(self):
        if not self._record:
            return
        record, self._record = self._record, None
        try:
            record.frame("结束建图", self._last_frame)
        except OSError as exc:
            self.ctx.log("建图记录写入失败：" + str(exc))
        finally:
            try:
                record.finish()
            except OSError as exc:
                self.ctx.log("建图记录写入失败：" + str(exc))

    def on_close(self):
        self.closed = True
        self.disconnect()
        self.feed.close()
        self.map_panel.on_close()
        self.teleop.close()
        self.cam.on_close()
