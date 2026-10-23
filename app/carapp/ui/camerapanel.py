# -*- coding: utf-8 -*-
"""可嵌入的实时画面面板。

不单独占一个选项卡，而是作为一个普通 Frame 放到任意页面里
（当前是嵌在「夹取」页的最下方，方便一边操作一边看画面）。

用法：
    panel = CameraPanel(parent, ctx)
    panel.pack(fill="both", expand=True)
    # 主循环里每 100ms 调一次 panel.on_tick(state)；退出时调 panel.on_close()

图像解码放在 UI 主线程：Tk 的 PhotoImage 不能跨线程创建。
后台线程只负责拆帧，主线程每 100ms 取最新一帧渲染。
"""

import base64
import io
import time
import tkinter as tk

try:
    from PIL import Image, ImageTk
    HAS_PIL = True
except Exception:
    HAS_PIL = False

from carapp.ui import theme


class CameraPanel(tk.Frame):
    def __init__(self, parent, ctx, compact=True):
        super().__init__(parent, bg=theme.BG)
        self.ctx = ctx
        self.compact = compact
        self._pending = None
        self._last_frame = None
        self._imgtk = None
        self._recording = False
        self._last_size = None
        self._build()
        ctx.add_frame_listener(self._on_frame)

    # ---------- 界面 ----------
    def build_bar(self, parent):
        """工具条：话题下拉 + 重连 + 抓帧 + 录制 + 右上角帧率。"""
        cfg = self.ctx.cfg["camera"]
        bar = tk.Frame(parent, bg=theme.PANEL)
        bar.pack(fill="x", padx=0, pady=(0, 4))

        tk.Label(bar, text="实时画面", bg=theme.PANEL, fg=theme.ACCENT,
                 font=theme.FONT_BOLD).pack(side="left", padx=(10, 10), pady=5)

        self.var_topic = tk.StringVar(value=cfg.get("default_topic", "/image_raw"))
        topics = cfg.get("topics", ["/image_raw"])
        self.cb = tk.OptionMenu(bar, self.var_topic, *topics,
                                command=lambda *_: self.reconnect())
        self.cb.config(bg=theme.PANEL_2, fg=theme.TEXT, relief="flat", bd=0,
                       highlightthickness=1, highlightbackground=theme.BORDER,
                       font=theme.FONT_SMALL, width=30, anchor="w",
                       activebackground=theme.BORDER, activeforeground=theme.TEXT)
        self.cb["menu"].config(bg=theme.PANEL_2, fg=theme.TEXT,
                               activebackground=theme.BORDER, font=theme.FONT_SMALL)
        self.cb.pack(side="left", padx=(0, 8), pady=4)

        self.btn_conn = self._btn(bar, "重新连接", self.reconnect)
        self.btn_snap = self._btn(bar, "抓帧", self.snapshot)
        self.btn_rec = self._btn(bar, "开始录制", self.toggle_record)

        self.lbl_info = tk.Label(bar, text="", bg=theme.PANEL, fg=theme.TEXT_DIM,
                                 font=theme.FONT_MONO)
        self.lbl_info.pack(side="right", padx=10)
        return bar

    def _build(self):
        self.build_bar(self)
        self.lbl_img = tk.Label(self, bg="#0b0b0b", fg=theme.TEXT_DIM,
                                text="等待摄像头数据…", font=theme.FONT)
        self.lbl_img.pack(fill="both", expand=True)

    def _btn(self, parent, text, cmd, fg=None):
        b = tk.Button(parent, text=text, command=cmd, bg=theme.PANEL_2,
                      fg=fg or theme.TEXT, relief="flat", font=theme.FONT_SMALL,
                      activebackground=theme.BORDER, padx=10, pady=1)
        b.pack(side="left", padx=3, pady=4)
        return b

    # ---------- 动作 ----------
    def reconnect(self):
        self.ctx.app.start_camera(self.var_topic.get())
        self._recording = False
        self._pending = None
        self._last_frame = None
        self.btn_rec.config(text="开始录制", fg=theme.TEXT)
        self.ctx.log("切换图像话题：%s" % self.var_topic.get())

    def snapshot(self):
        if not self._last_frame:
            self.ctx.log("抓帧失败：还没有画面")
            return
        ts, payload, mode = self._last_frame
        ext = "jpg" if mode == "jpeg" else "png"
        path = "%s/抓帧_%s.%s" % (self.ctx.save_dir(), time.strftime("%Y%m%d_%H%M%S"), ext)
        with open(path, "wb") as f:
            f.write(payload)
        self.ctx.log("抓帧已保存：%s（%.1f KB）" % (path, len(payload) / 1024))

    def toggle_record(self):
        cam = self.ctx.app.camera
        if not cam:
            self.ctx.log("录制失败：摄像头未连接")
            return
        if not self._recording:
            path = cam.start_record(self.ctx.save_dir("预览录像"))
            self._recording = True
            self.btn_rec.config(text="停止录制", fg=theme.WARN)
            self.ctx.log("开始录制（保存在本机）：%s" % path)
        else:
            path = cam.stop_record()
            self._recording = False
            self.btn_rec.config(text="开始录制", fg=theme.TEXT)
            self.ctx.log("录制结束：%s" % path)

    # ---------- 渲染 ----------
    def _on_frame(self, ts, payload, mode):
        self._pending = (ts, payload, mode)
        self._last_frame = self._pending

    def _fit_box(self):
        """画面区的可用像素（按面板本身算，不看图像标签，避免缩放反馈循环）。"""
        try:
            w = self.winfo_width()
            h = self.winfo_height()
        except Exception:
            return None
        if w < 80 or h < 80:
            return None
        return w - 4, max(80, h - 40)   # 40 ≈ 工具条高度

    def on_tick(self, st):
        if not self._pending:
            return
        ts, payload, mode = self._pending
        self._pending = None

        try:
            if mode == "jpeg" and HAS_PIL:
                img = Image.open(io.BytesIO(payload))
                box = self._fit_box()
                if box:
                    aw, ah = box
                    scale = min(aw / float(img.width), ah / float(img.height), 1.0)
                    if scale < 0.999:
                        img = img.resize((max(1, int(img.width * scale)),
                                          max(1, int(img.height * scale))), Image.BILINEAR)
                photo = ImageTk.PhotoImage(img)
                self._last_size = img.size
            else:
                photo = tk.PhotoImage(data=base64.b64encode(payload).decode("ascii"))
                self._last_size = (photo.width(), photo.height())
            self._imgtk = photo  # 必须持有引用，否则被 GC 后图片消失
            self.lbl_img.config(image=photo, text="")
        except Exception as exc:
            self.lbl_img.config(text="图像解码失败：%s" % exc)

        cam = self.ctx.app.camera
        if cam:
            size = st.get("cam_size") or self._last_size
            self.lbl_info.config(text="%s | %.1f fps | %.1f Mbps" % (
                ("%dx%d" % size) if size else "--", cam.fps(), cam.mbps()))

    def on_close(self):
        if self._recording and self.ctx.app.camera:
            self.ctx.app.camera.stop_record()
