# -*- coding: utf-8 -*-
"""路侧红绿灯广播展示页：只读显示 UDP 广播的灯态，不参与任何控制。

收包逻辑在 ``carapp.core.signal_receiver``；本页只在进入时启动监听、每 100 ms
读一次线程安全快照并绘制，不向 ROS、底盘或舵机下发任何指令。
"""
import tkinter as tk

from carapp.core.signal_receiver import DEFAULT_PORT, SignalReceiver
from carapp.features.base import Feature
from carapp.ui import theme

LAMPS = ("red", "yellow", "green")
LAMP_COLORS = {"red": theme.ERR, "yellow": theme.WARN, "green": theme.OK}
LAMP_CAPTIONS = {"red": "红", "yellow": "黄", "green": "绿"}
LAMP_NAMES = {"red": "红灯", "yellow": "黄灯", "green": "绿灯"}
OFF_COLOR = "#3a3a3a"
ALIASES = {"red": "red", "r": "red", "红灯": "red",
           "yellow": "yellow", "amber": "yellow", "y": "yellow", "黄灯": "yellow",
           "green": "green", "g": "green", "绿灯": "green"}


def lamp_state(value):
    """广播里的灯色字段转成 red/yellow/green；无法识别时返回空串。"""
    return ALIASES.get(str(value or "").strip().lower(), "")


class TrafficBroadcastFeature(Feature):
    key = "signal"
    title = "广播灯态"

    def __init__(self, ctx):
        super().__init__(ctx)
        self.receiver = None
        self._signature = None
        self._detail_text = None
        self._lamp_ids = {"h": (), "v": ()}
        self._value_ids = {"h": None, "v": None}
        self._message = {}
        self._stale = True

    # ---------- 界面 ----------
    def build(self, parent):
        bar = tk.Frame(parent, bg=theme.PANEL)
        bar.pack(fill="x", padx=6, pady=(6, 0))
        self.status = tk.Label(bar, text="未监听", bg=theme.PANEL, fg=theme.TEXT_DIM,
                               anchor="w", font=theme.FONT_SMALL)
        self.status.pack(side="left", padx=10, pady=7)
        self.button = tk.Button(bar, text="开始监听", command=self.toggle, bg=theme.PANEL_2,
                                fg=theme.TEXT, activebackground=theme.BORDER,
                                activeforeground=theme.ACCENT, relief="flat",
                                font=theme.FONT_SMALL, padx=12, pady=4)
        self.button.pack(side="right", padx=8, pady=4)

        panel = tk.Frame(parent, bg=theme.PANEL)
        panel.pack(fill="both", expand=True, padx=6, pady=6)
        self.canvas = tk.Canvas(panel, bg=theme.PANEL, highlightthickness=0, height=260)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=(8, 0))
        self.canvas.bind("<Configure>", lambda _event: self._layout())
        self.detail = tk.Label(panel, text="等待路侧广播…", bg=theme.PANEL, fg=theme.TEXT_DIM,
                               anchor="w", justify="left", font=theme.FONT_SMALL,
                               padx=12, pady=8)
        self.detail.pack(fill="x")
        tk.Label(parent, text="只读展示：路侧广播只用于观察灯态，不参与停车判定，"
                              "也不会改变任何底盘或舵机指令。",
                 bg=theme.BG, fg=theme.TEXT_DIM, anchor="w",
                 font=theme.FONT_SMALL).pack(fill="x", padx=12, pady=(0, 4))
        self._layout()

    def _layout(self):
        """按当前画布尺寸重画两组灯头；窗口缩放时由 <Configure> 再次调用。"""
        self.canvas.delete("all")
        width = max(self.canvas.winfo_width(), 640)
        height = max(self.canvas.winfo_height(), 260)
        left = max(24., (width - 500) / 2)
        middle = height / 2
        for row, (key, label) in enumerate((("h", "横向"), ("v", "纵向"))):
            y = middle + (row - .5) * 124
            self.canvas.create_text(left, y, text=label, anchor="w",
                                    fill=theme.TEXT_DIM, font=theme.FONT_BOLD)
            lamps = []
            for index, name in enumerate(LAMPS):
                x = left + 120 + index * 92
                lamps.append(self.canvas.create_oval(x - 30, y - 30, x + 30, y + 30,
                                                     fill=OFF_COLOR, outline=theme.BORDER,
                                                     width=2))
                self.canvas.create_text(x, y + 48, text=LAMP_CAPTIONS[name],
                                        fill=theme.TEXT_DIM, font=theme.FONT_SMALL)
            self._lamp_ids[key] = tuple(lamps)
            self._value_ids[key] = self.canvas.create_text(left + 400, y, text="未知", anchor="w",
                                                           fill=theme.TEXT_DIM,
                                                           font=theme.FONT_TITLE)
        self._paint(self._message, self._stale)

    def _paint(self, message, stale):
        """亮起报文里给出的灯；超时只提示，不抹掉最后一次收到的灯态。"""
        for key in ("h", "v"):
            name = lamp_state(message.get(key))
            for lamp, item in zip(LAMPS, self._lamp_ids[key]):
                self.canvas.itemconfigure(item, fill=LAMP_COLORS[lamp] if lamp == name else OFF_COLOR)
            item = self._value_ids[key]
            if item is None:
                continue
            text = LAMP_NAMES.get(name, "未知")
            color = LAMP_COLORS.get(name, theme.TEXT_DIM)
            if message and stale:
                text += "（超时）"
                color = theme.WARN
            self.canvas.itemconfigure(item, text=text, fill=color)

    # ---------- 监听 ----------
    def _make_receiver(self):
        settings = self.ctx.cfg.get("signal") or {}
        port, timeout = settings.get("port", DEFAULT_PORT), settings.get("timeout", 3.0)
        try:
            port, timeout = int(port), float(timeout)
        except (TypeError, ValueError):
            port, timeout = DEFAULT_PORT, 3.0
        return SignalReceiver(port, timeout=timeout)

    def _listen(self):
        if self.receiver is None:
            self.receiver = self._make_receiver()
        if self.receiver.start():
            self.ctx.log("红绿灯广播：监听 UDP %s，只做展示" % self.receiver.port)
        else:
            self.ctx.log("红绿灯广播：%s" % self.receiver.last_error)
        self._render(force=True)

    def toggle(self):
        if self.receiver is not None and self.receiver.running:
            self.receiver.stop()
            self.ctx.log("红绿灯广播：已停止监听")
            self._render(force=True)
        else:
            self._listen()

    # ---------- 页面生命周期 ----------
    def on_activate(self):
        # 首次进入才占端口；之后切走再回来沿用同一实例，灯态连续。
        if self.receiver is None:
            self._listen()

    def on_tick(self, st):
        self._render()

    def on_close(self):
        if self.receiver is not None:
            self.receiver.stop()

    # ---------- 刷新 ----------
    def _render(self, force=False):
        receiver = self.receiver
        if receiver is None:
            return
        state = receiver.snapshot()
        message = state["state"] or {}
        signature = (receiver.running, state["stale"], state["error"], state["packets"],
                     state["bad"], state["peer"], message.get("h"), message.get("v"),
                     message.get("seq"), message.get("phase"), message.get("mask_hex"))
        if force or signature != self._signature:
            self._signature = signature
            self._message, self._stale = message, state["stale"]
            self._paint(message, state["stale"])
            self._show_status(receiver, state)
        self._show_detail(receiver, state)

    def _show_status(self, receiver, state):
        if receiver.running:
            text = "监听 UDP %s · 已收 %d 包 · 坏包 %d" % (receiver.port, state["packets"], state["bad"])
            if state["peer"]:
                text += " · 对端 " + state["peer"]
            self.status.configure(text=text, fg=theme.WARN if state["stale"] else theme.OK)
            self.button.configure(text="停止监听")
        elif state["error"]:
            self.status.configure(text="监听失败：" + state["error"], fg=theme.ERR)
            self.button.configure(text="重试监听")
        else:
            self.status.configure(text="未监听 UDP %s" % receiver.port, fg=theme.TEXT_DIM)
            self.button.configure(text="开始监听")

    def _show_detail(self, receiver, state):
        message = state["state"] or {}
        if not message:
            text = "等待路侧广播（UDP %s）…" % receiver.port
        else:
            age = state["age"]
            text = "序号 %s · 掩码 %s · 阶段 %s · 最后更新 %s" % (
                message.get("seq", "—"), message.get("mask_hex") or "—",
                message.get("phase") or "—",
                "—" if age is None else "%.1f s 前" % age)
            if state["stale"]:
                text += " · 广播已超时，显示最后一次收到的灯态"
        if text != self._detail_text:
            self._detail_text = text
            self.detail.configure(text=text,
                                  fg=theme.WARN if message and state["stale"] else theme.TEXT_DIM)
