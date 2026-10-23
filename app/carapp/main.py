# -*- coding: utf-8 -*-
"""小程序主入口：组装窗口、启动后台连接、驱动 UI 刷新。

对外只暴露一个 main()，双击 run.bat 或 `python -m carapp.main` 都会走到这里。
"""

import math
import os
import queue
import sys
import threading
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
        self._nodes_timer = None
        self._recover_timer = None
        self._closing = False

        self._build_ui()
        self._start_bridge()
        self.start_camera(cfg["camera"].get("default_topic", "/image_raw"))

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self._tick)
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

        self.log("配置：%s" % os.path.join(APP_DIR, "config", "settings.json"))
        self.log("图像模式：%s" % ("JPEG/Pillow" if HAS_PIL else "PNG/Tk 原生"))

    def log(self, msg):
        self._log_queue.put(str(msg))

    # ---------------- 后台连接 ----------------
    def _start_bridge(self):
        car = self.cfg["car"]
        if self.bridge:
            self.bridge.stop()
        self.bridge = RosBridge(
            car["host"], int(car["ws_port"]),
            on_message=self._on_ros_message,
            on_status=self._on_ros_status,
            log=lambda m: self.root.after(0, self.log, m),
        )
        self.bridge.start()

    def _on_ros_status(self, connected, detail=""):
        def apply():
            if self._closing:
                return
            self.state.update(ws_connected=connected, ws_detail=detail)
            if connected:
                self.bridge.subscribe("/odom", throttle_ms=500)
                self.bridge.subscribe("/ros_robot_controller/battery", throttle_ms=1000)
                # 连上就立刻取一次节点数，不用等 6 秒的轮询
                self.bridge.call_service("/rosapi/nodes", {}, self._on_nodes)
        self.root.after(0, apply)

    def _on_ros_message(self, topic, msg):
        if not isinstance(msg, dict):
            return
        if topic.endswith("/odom"):
            try:
                p = msg["pose"]["pose"]
                x = float(p["position"]["x"])
                y = float(p["position"]["y"])
                q = p["orientation"]
                yaw = math.degrees(2 * math.atan2(float(q["z"]), float(q["w"])))
                self.state.update(pose=(x, y, yaw))
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
        if self.bridge:
            self.bridge.call_service("/rosapi/nodes", {}, self._on_nodes)
        self._nodes_timer = self.root.after(
            int(self.cfg["status"]["nodes_interval"] * 1000), self._poll_nodes)

    def _on_nodes(self, values, result):
        nodes = (values or {}).get("nodes") or []
        self.state.update(node_count=len(nodes) if result else None)

    # ---------------- 摄像头 ----------------
    def start_camera(self, topic):
        self.stop_camera()
        car = self.cfg["car"]
        self.cam_q = queue.Queue(maxsize=1)
        self.cam_topic = topic
        mode = "jpeg" if HAS_PIL else "png"
        self.camera = CameraStream(car["host"], int(car["http_port"]), topic, mode,
                                   self.cam_q, log=lambda m: self.root.after(0, self.log, m))
        self.camera.start()

    def stop_camera(self):
        if self.camera:
            self.camera.stop_record()
            self.camera.stop()
            self.camera = None

    # ---------------- 主机切换 / 自动发现 ----------------
    def change_host(self, host):
        for feature in self.features:
            if getattr(feature, "_busy", False):
                self.log("动作序列执行中，请先中止并等待结束后切换小车")
                return
        for feature in self.features:
            if hasattr(feature, "stop_all"):
                feature.stop_all()
        self.cfg["car"]["host"] = host
        save_config(self.cfg)
        self.state.update(ws_connected=False, cam_size=None, node_count=None,
                          battery_mv=None, pose=None, servo_targets={}, gripper_target=None)
        self.log("切换小车地址：%s" % host)
        self._start_bridge()
        self.start_camera(self.cam_topic or self.cfg["camera"]["default_topic"])

    def discover_car(self, callback):
        # 两条接入路径都要扫：网线 192.168.150.x、车自己发的 WiFi 热点 192.168.149.x
        prefixes = self.cfg["car"].get("subnet_prefixes")
        if not prefixes:
            prefixes = [self.cfg["car"].get("subnet_prefix", "192.168.150"), "192.168.149"]

        def worker():
            found = scan_net(prefixes)
            self.root.after(0, callback, found)
        threading.Thread(target=worker, daemon=True).start()

    def _auto_recover(self):
        """启动 5 秒还没连上，就自己扫一遍：只找到一台就切过去。

        为什么需要：两台同款车换着用时，最容易犯的错就是"插的是 2 号车、下拉还停在 1 号车"。
        """
        if self.state.get("ws_connected"):
            return
        self.log("启动后仍未连接，自动搜索在线的小车…")

        def done(found):
            if not found:
                self.log("未搜到小车：确认网线插好，且本机在 192.168.150.x 或 192.168.149.x 网段")
                return
            if len(found) > 1:
                self.log("搜到 %d 台（%s），请在左上角下拉里选一台" % (len(found), "、".join(found)))
                return
            host = found[0]
            if host == self.cfg["car"]["host"]:
                self.log("小车 %s 在线，但 ROS 服务未就绪（可能还在开机，等 40~60 秒）" % host)
                return
            self.log("自动切换到在线的小车：%s" % host)
            topbar = getattr(self, "topbar", None)
            if topbar is not None:
                topbar.var_host.set(host)
                for c in (self.cfg.get("cars") or []):
                    if c.get("host") == host and hasattr(topbar, "var_car"):
                        topbar.var_car.set(c.get("name") or host)
                        break
            self.change_host(host)

        self.discover_car(done)

    # ---------------- 主循环 ----------------
    def _tick(self):
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
            if self.camera:
                self.state.update(cam_fps=self.camera.fps(), cam_size=self.camera.size)
        except queue.Empty:
            pass

        st = self.state.snapshot()
        try:
            self.topbar.update_state(st)
            for feat in self.features:
                feat.on_tick(st)
        except Exception as exc:
            self.log("界面刷新异常：%s" % exc)

        self.root.after(100, self._tick)

    # ---------------- 选项卡 ----------------
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
        for timer in (self._nodes_timer, self._recover_timer):
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
    ap.add_argument("--tab", help="启动时选中哪个选项卡（目前只有 gripper）")
    args = ap.parse_args(argv)

    cfg = load_config()
    if args.host:
        cfg["car"]["host"] = args.host

    app = App(cfg)
    if args.tab:
        app.select_tab(args.tab)
    app.run()


if __name__ == "__main__":
    main()
