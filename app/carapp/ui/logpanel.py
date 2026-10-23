# -*- coding: utf-8 -*-
"""底部日志面板：所有操作与异常都往这里写，方便事后回看。"""

import time
import tkinter as tk

from carapp.ui import theme


class LogPanel(tk.Frame):
    def __init__(self, parent, lines=300):
        super().__init__(parent, bg=theme.BG)
        self.max_lines = lines

        head = tk.Frame(self, bg=theme.BG)
        head.pack(fill="x")
        tk.Label(head, text="运行日志", bg=theme.BG, fg=theme.TEXT_DIM,
                 font=theme.FONT_SMALL).pack(side="left", padx=8)
        tk.Button(head, text="清空", command=self.clear, relief="flat",
                  bg=theme.PANEL_2, fg=theme.TEXT_DIM, font=theme.FONT_SMALL,
                  activebackground=theme.BORDER, padx=8).pack(side="right", padx=8)

        body = tk.Frame(self, bg=theme.PANEL)
        body.pack(fill="both", expand=True, padx=8, pady=(2, 8))
        self.text = tk.Text(body, height=5, bg=theme.PANEL, fg=theme.TEXT_DIM,
                            insertbackground=theme.TEXT, relief="flat",
                            font=theme.FONT_MONO, wrap="none")
        self.text.pack(side="left", fill="both", expand=True)
        sb = tk.Scrollbar(body, command=self.text.yview, bg=theme.PANEL_2)
        sb.pack(side="right", fill="y")
        self.text.config(yscrollcommand=sb.set, state="disabled")

    def add(self, msg):
        stamp = time.strftime("%H:%M:%S")
        self.text.config(state="normal")
        self.text.insert("end", "[%s] %s\n" % (stamp, msg))
        # 超长就裁掉最早的
        total = int(self.text.index("end-1c").split(".")[0])
        if total > self.max_lines:
            self.text.delete("1.0", "%d.0" % (total - self.max_lines + 1))
        self.text.see("end")
        self.text.config(state="disabled")

    def clear(self):
        self.text.config(state="normal")
        self.text.delete("1.0", "end")
        self.text.config(state="disabled")
