# -*- coding: utf-8 -*-
"""小程序主入口：组装窗口、启动后台连接、驱动 UI 刷新。

对外只暴露一个 main()，双击 run.bat 或 `python -m carapp.main` 都会走到这里。
"""

import math
import os
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

# 允许 `python carapp/main.py` 与 `python -m carapp.main` 两种启动方式
APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if APP_DIR not in sys.path:
    sys.path.insert(0, APP_DIR)

from carapp.config import load_config, save_config  # noqa: E402
from carapp.core.camera import CameraStream  # noqa: E402
from carapp.core.discovery import scan as scan_net  # noqa: E402
from carapp.core.rosbridge import RosBridge  # noqa: E402
from carapp.core.connection import diagnose, require_wifi  # noqa: E402
from carapp.features import FEATURES  # noqa: E402
from carapp.features.base import AppContext  # noqa: E402
from carapp.model.state import CarState  # noqa: E402
from carapp.ui import theme  # noqa: E402
from carapp.ui.logpanel import LogPanel  # noqa: E402
from carapp.ui.topbar import TopBar  # noqa: E402

try:
    from PIL import Image  # noqa: F401
    HAS_PIL = True
except Exception:
    HAS_PIL = False


class App:
    def __init__(self, cfg):
        self.cfg = cfg
        self.state = CarState()
        self.root = tk.Tk()
        self.root.title("小车控制台  ·  ClawPi")
        self.root.geometry("1240x980")
        self.root.minsize(1000, 720)
        self.root.configure(bg=theme.BG)
        theme.style_ttk(self.root)

        self.ctx = AppContext(self)
        self.bridge = None
        self.camera = None
        self.cam_q = queue.Queue(maxsize=1)
        self.cam_topic = None
        self.frame_listeners = []
        self.logpanel = None
        self._log_queue = queue.Queue()
        self._ui_queue = queue.Queue()
        self._connection_generation = 0
        self._scan_running = False
        self._connection_notice = None
        self._nodes_timer = None
        self._recover_timer = None
        self._tick_timer = None
        self._closing = False
        self._active_feature = None
        self._had_window_focus = False
        self._focus_timer = None

        self._build_ui()
        self.root.bind("<FocusIn>", self._window_focus_in, add="+")
        self.root.bind("<FocusOut>", self._window_focus_out, add="+")
        self._start_bridge()
        self.start_camera(cfg["camera"].get("default_topic", "/image_raw"))

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._tick_timer = self.root.after(100, self._tick)
        self._poll_nodes()
        self._recover_timer = self.root.after(5000, self._auto_recover)

    # ---------------- 界面 ----------------
    def _build_ui(self):
        self.topbar = TopBar(self.root, self)
        self.topbar.pack(fill="x")

        # 日志先占住底部（side=bottom），否则内容一高就被挤没了
        self.logpanel = LogPanel(self.root, lines=self.cfg["status"]["log_lines"])
        self.logpanel.pack(side="bottom", fill="x", pady=(4, 0))

        body = tk.Frame(self.root, bg=theme.BG)
        body.pack(fill="both", expand=True, padx=8, pady=(6, 0))

        self.nb = ttk.Notebook(body)
        self.nb.pack(fill="both", expand=True)

        self.features = []
        for cls in FEATURES:
            frame = tk.Frame(self.nb, bg=theme.BG)
            feat = cls(self.ctx)
            feat.build(frame)
            self.nb.add(frame, text=cls.title)
            self.features.append(feat)

        self.nb.bind("<<NotebookTabChanged>>", self._tab_changed)

        self.log("配置：%s" % os.path.join(APP_DIR, "config", "settings.json"))
        self.log("图像模式：%s" % ("JPEG/Pillow" if HAS_PIL else "PNG/Tk 原生"))

    def log(self, msg):
        self._log_queue.put(str(msg))

    def _post(self, callback, *args):
        if not self._closing:
            self._ui_queue.put((callback, args))

    def _connection_guard(self):
        car = self.cfg["car"]
        if car.get("transport") == "wifi":
            profile = next((p for p in self.cfg["cars"] if p["id"] == car.get("profile_id")), {})
            ssid = profile.get("ssid")
            return lambda: require_wifi(ssid)
        return None

    def _session_callback(self, generation, callback, *args):
        if not self._closing and generation == self._connection_generation:
            callback(*args)

    # ---------------- 后台连接 ----------------
    def _start_bridge(self):
        car = self.cfg["car"]
        self._connection_generation += 1
        generation = self._connection_generation
        if self.bridge:
            self.bridge.stop()
        def post(callback):
            return lambda *args: self._post(self._session_callback, generation, callback, *args)
        self.bridge = RosBridge(
            car["host"], int(car["ws_port"]),
            # Timestamp at transport receipt: queued old scans must not look fresh.
            on_message=lambda topic, msg: self._post(self._session_callback, generation,
                self._on_ros_message, topic, msg, time.monotonic()),
            on_status=post(self._on_ros_status),
            log=post(self.log),
            connection_guard=self._connection_guard(),
        )
        self.bridge.start()

    def _on_ros_status(self, connected, detail=""):
        if self._closing:
            return
        was_connected = self.state.get("ws_connected")
        self.state.update(ws_connected=connected, ws_detail=detail)
        if not connected:
            self.state.update(node_count=None, battery_mv=None, pose=None, pose_seen_at=0,
                              servo_targets={}, gripper_target=None, base_cmd=(0.0, 0.0), base_vy=0.0)
            if was_connected:
                for feature in self.features:
                    if hasattr(feature, "stop_all"):
                        feature.stop_all()
                    tracking = getattr(feature, "tracking", None)
                    if tracking:
                        tracking.reset()
                self.log("连接已断开，动作已取消；恢复连接后需手动重新发起动作")
        else:
            # 重连后只恢复状态订阅，不恢复运动或舵机动作。
            self.ctx.stop_base()
            self.bridge.subscribe("/odom", throttle_ms=100)
            self.bridge.subscribe("/ros_robot_controller/battery", throttle_ms=1000)
            for feature in getattr(self, "features", []):
                for topic, throttle in getattr(feature, "ros_subscriptions", lambda: {})().items():
                    self.bridge.subscribe(topic, throttle_ms=throttle)
            self._request_nodes()

    def _on_ros_message(self, topic, msg, received_at=None):
        if not isinstance(msg, dict):
            return
        received_at = time.monotonic() if received_at is None else received_at
        for feature in getattr(self, "features", []):
            callback = getattr(feature, "on_ros_message", None)
            if callback:
                callback(topic, msg, received_at)
        if topic.endswith("/odom"):
            try:
                p = msg["pose"]["pose"]
                x = float(p["position"]["x"])
                y = float(p["position"]["y"])
                q = p["orientation"]
                yaw = math.degrees(2 * math.atan2(float(q["z"]), float(q["w"])))
                self.state.update(pose=(x, y, yaw), pose_seen_at=time.monotonic())
            except Exception:
                pass
        elif topic.endswith("battery"):
            try:
                raw = msg.get("voltage")
                if raw is None:
                    raw = msg.get("data")
                if raw is not None:
                    self.state.update(battery_mv=float(raw))
            except Exception:
                pass

    def _poll_nodes(self):
        """单条自循环的定时链：init 时启动一次，之后自己续期。"""
        self._request_nodes()
        self._nodes_timer = self.root.after(
            int(self.cfg["status"]["nodes_interval"] * 1000), self._poll_nodes)

    def _request_nodes(self):
        if self.bridge and self.bridge.connected:
            generation = self._connection_generation
            self.bridge.call_service("/rosapi/nodes", {},
                lambda *args: self._post(self._session_callback, generation, self._on_nodes, *args))

    def _on_nodes(self, values, result):
        nodes = (values or {}).get("nodes") or []
        self.state.update(node_count=len(nodes) if result else None)

    # ---------------- 摄像头 ----------------
    def start_camera(self, topic):
        self.stop_camera()
        car = self.cfg["car"]
        self.cam_q = queue.Queue(maxsize=1)
        self.cam_topic = topic
        self.state.update(cam_connected=False, cam_size=None, cam_fps=0, cam_error="等待摄像头服务")
        mode = "jpeg" if HAS_PIL else "png"
        self.camera = CameraStream(car["host"], int(car["http_port"]), topic, mode,
                                   self.cam_q, log=self.log,
                                   connection_guard=self._connection_guard())
        self.camera.start()

    def stop_camera(self):
        if self.camera:
            self.camera.stop_record()
            self.camera.stop()
            self.camera = None

    # ---------------- 主机切换 / 自动发现 ----------------
    def change_host(self, host, profile_id=None, transport=None):
        for feature in self.features:
            if getattr(feature, "_busy", False):
                self.log("动作序列执行中，请先中止并等待结束后切换小车")
                return False
        for feature in self.features:
            if hasattr(feature, "stop_all"):
                feature.stop_all()
            tracking = getattr(feature, "tracking", None)
            if tracking:
                tracking.reset()
        if any(getattr(f, "_motion_thread", None) and f._motion_thread.is_alive() for f in self.features):
            self.log("正在停止上台车的运动线程，请稍后再切换")
            return False
        for feature in self.features:
            if hasattr(feature,"on_vehicle_change"):
                feature.on_vehicle_change()
        profiles = self.cfg["cars"]
        profile = next((p for p in profiles if p["id"] == profile_id), None)
        if not profile_id:
            profile = next((p for p in profiles if p.get("host") == host), None)
        transport = transport or ("wifi" if host == "192.168.149.1" else "wired")
        if transport == "wifi" and not profile:
            self.log("两车热点地址相同，请先选好车辆，再选择 Wi-Fi 热点连接")
            return False
        self.cfg["car"]["profile_id"] = profile["id"] if profile else ""
        self.cfg["car"]["transport"] = transport
        self.cfg["car"]["host"] = host
        save_config(self.cfg)
        self.state.update(ws_connected=False, cam_size=None, node_count=None,
                          battery_mv=None, pose=None, pose_seen_at=0, servo_targets={}, gripper_target=None,
                          base_cmd=(0.0, 0.0), base_vy=0.0, ws_detail="正在连接")
        self._connection_notice = None
        self.topbar.sync_selection()
        self.log("切换小车地址：%s" % host)
        self._start_bridge()
        self.start_camera(self.cam_topic or self.cfg["camera"]["default_topic"])
        return True

    def discover_car(self, callback):
        if self._scan_running:
            return
        self._scan_running = True
        generation = self._connection_generation
        # 两条接入路径都要扫：网线 192.168.150.x、车自己发的 WiFi 热点 192.168.149.x
        prefixes = self.cfg["car"].get("subnet_prefixes")
        if not prefixes:
            prefixes = [self.cfg["car"].get("subnet_prefix", "192.168.150"), "192.168.149"]

        def worker():
            try:
                found = scan_net(prefixes, http_port=int(self.cfg["car"]["http_port"]),
                                 ws_port=int(self.cfg["car"]["ws_port"]),
                                 cancel=lambda: self._closing)
            except Exception as exc:
                self.log("搜索失败：%s" % exc)
                found = []
            self._post(done, found)
        def done(found):
            self._scan_running = False
            if generation != self._connection_generation:
                self.topbar.btn_scan.config(state="normal", text="自动搜索")
                self.log("搜索期间已切换车辆，已忽略旧搜索结果")
                return
            callback(found)
        threading.Thread(target=worker, daemon=True).start()

    def _auto_recover(self):
        """持续显示就绪状态；各连接线程持续重试当前车辆，禁止自动换车。"""
        if self._closing:
            return
        ready = (self.state.get("ws_connected"), self.state.get("cam_connected"))
        if ready != self._connection_notice:
            self._connection_notice = ready
            if ready == (True, True):
                self.log("ROS 与画面均已就绪")
            else:
                self.log("正在等待当前车辆的%s，连接会持续重试；可点「连接诊断」查看网段和服务状态" %
                         ("ROS 和画面" if not any(ready) else ("画面" if ready[0] else "ROS")))
        self._recover_timer = self.root.after(5000, self._auto_recover)

    def diagnose_connection(self, callback):
        import copy
        car, profiles = copy.deepcopy(self.cfg["car"]), copy.deepcopy(self.cfg["cars"])
        def worker():
            try:
                report = diagnose(car, profiles)
            except Exception as exc:
                report = "诊断失败：%s" % exc
            self._post(callback, report)
        threading.Thread(target=worker, name="connection-diagnostics", daemon=True).start()

    # ---------------- 主循环 ----------------
    def _tick(self):
        if self._closing:
            return
        while not self._ui_queue.empty():
            callback, args = self._ui_queue.get_nowait()
            try:
                callback(*args)
            except Exception as exc:
                self.log("连接事件处理失败：%s" % exc)
        while not self._log_queue.empty():
            try:
                self.logpanel.add(self._log_queue.get_nowait())
            except queue.Empty:
                break
        # 摄像头最新帧分发给各功能（谁注册谁拿到）
        try:
            ts, payload = self.cam_q.get_nowait()
            mode = "jpeg" if HAS_PIL else "png"
            for cb in list(self.frame_listeners):
                try:
                    cb(ts, payload, mode)
                except Exception:
                    pass
        except queue.Empty:
            pass

        if self.camera:
            fresh = self.camera.connected and time.monotonic() - self.camera.last_frame_at < 3
            self.state.update(cam_connected=fresh, cam_error=self.camera.last_error or "等待新画面",
                              cam_fps=self.camera.fps() if fresh else 0,
                              cam_size=self.camera.size if fresh else None)

        st = self.state.snapshot()
        try:
            self.topbar.update_state(st)
            for feat in self.features:
                feat.on_tick(st)
        except Exception as exc:
            self.log("界面刷新异常：%s" % exc)

        self._tick_timer = self.root.after(100, self._tick)

    # ---------------- 选项卡 ----------------
    def _tab_changed(self, _event=None):
        selected = self.features[self.nb.index("current")]
        if selected is self._active_feature:
            return
        if self._active_feature:
            self._active_feature.on_deactivate()
        self._active_feature = selected
        selected.on_activate()

    def _window_focus_in(self, _event=None):
        self._had_window_focus = True

    def _window_focus_out(self, _event=None):
        if self._focus_timer:
            self.root.after_cancel(self._focus_timer)
        self._focus_timer = self.root.after_idle(self._check_window_focus)

    def _check_window_focus(self):
        self._focus_timer = None
        if self._closing or not self._had_window_focus:
            return
        focus = self.root.focus_get()
        if focus is None or focus.winfo_toplevel() is not self.root:
            self._had_window_focus = False
            for feature in self.features:
                if hasattr(feature, "stop_all"):
                    feature.stop_all()

    def select_tab(self, key):
        """按功能 key 选中选项卡，key 找不到就保持默认。"""
        for idx, feat in enumerate(self.features):
            if feat.key == key:
                self.nb.select(idx)
                return True
        return False

    # ---------------- 退出 ----------------
    def on_close(self):
        self._closing = True
        for timer in (self._nodes_timer, self._recover_timer, self._focus_timer, self._tick_timer):
            if timer:
                try:
                    self.root.after_cancel(timer)
                except Exception:
                    pass
        try:
            for feat in self.features:
                feat.on_close()
            self.stop_camera()
            if self.bridge:
                self.bridge.stop()
        finally:
            self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(argv=None):
    import argparse

    ap = argparse.ArgumentParser(description="ClawPi 小车控制台")
    ap.add_argument("--host", help="临时指定小车地址，覆盖 config/settings.json")
    ap.add_argument("--tab", help="启动选项卡：gripper、remote、cruise 或 terminal")
    args = ap.parse_args(argv)

    cfg = load_config()
    if args.host:
        cfg["car"]["host"] = args.host
        cfg["car"]["transport"] = "wifi" if args.host == "192.168.149.1" else "wired"
        if cfg["car"]["transport"] == "wired":
            profile = next((p for p in cfg["cars"] if p["host"] == args.host), {})
            cfg["car"]["profile_id"] = profile.get("id", "")

    app = App(cfg)
    if args.tab:
        app.select_tab(args.tab)
    app.run()


if __name__ == "__main__":
    main()
