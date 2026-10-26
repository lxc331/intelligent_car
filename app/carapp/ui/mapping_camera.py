"""A narrow toolbar for the shared camera component in a split view."""
import tkinter as tk

from carapp.ui import theme
from carapp.ui.camerapanel import CameraPanel


class MappingCamera(CameraPanel):
    def build_bar(self, parent):
        bar = tk.Frame(parent, bg=theme.PANEL)
        bar.pack(fill="x", pady=(0, 4))
        top = tk.Frame(bar, bg=theme.PANEL)
        top.pack(fill="x")
        tk.Label(top, text="摄像头实时画面", bg=theme.PANEL, fg=theme.ACCENT,
                 font=theme.FONT_BOLD).pack(side="left", padx=8, pady=4)
        cfg = self.ctx.cfg["camera"]
        self.var_topic = tk.StringVar(value=cfg.get("default_topic", "/image_raw"))
        self.cb = tk.OptionMenu(top, self.var_topic, *cfg.get("topics", ["/image_raw"]),
                                command=lambda *_: self.reconnect())
        self.cb.config(bg=theme.PANEL_2, fg=theme.TEXT, width=16, relief="flat",
                       highlightthickness=0, font=theme.FONT_SMALL)
        self.cb["menu"].config(bg=theme.PANEL_2, fg=theme.TEXT)
        self.cb.pack(side="right", padx=4)
        actions = tk.Frame(bar, bg=theme.PANEL)
        actions.pack(fill="x")
        self.btn_conn = self._btn(actions, "重连", self.reconnect)
        self.btn_snap = self._btn(actions, "抓帧", self.snapshot)
        self.btn_rec = self._btn(actions, "开始录制", self.toggle_record)
        self.lbl_info = tk.Label(actions, bg=theme.PANEL, fg=theme.TEXT_DIM,
                                 font=("Consolas", 8))
        self.lbl_info.pack(side="right", padx=4)
        return bar

    def _fit_box(self):
        width, height = self.winfo_width(), self.winfo_height()
        if width < 80 or height < 100:
            return None
        return width - 4, max(40, height - 85)
