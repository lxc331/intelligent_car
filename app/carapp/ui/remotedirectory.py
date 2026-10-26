"""Choose a directory in the car's explicit host/container filesystem."""
import posixpath
import tkinter as tk
from tkinter import ttk

from carapp.ui import theme


class RemoteDirectory(tk.Toplevel):
    def __init__(self, parent, request, choose, path, scope):
        super().__init__(parent)
        self.title("选择小车端保存目录 · " + ("ClawPi 容器" if scope == "container" else "宿主机"))
        self.geometry("660x420")
        self.configure(bg=theme.BG)
        self.request, self.choose, self.scope = request, choose, scope
        self.path = tk.StringVar(value=path)
        self._paths = {}
        row = tk.Frame(self, bg=theme.PANEL)
        row.pack(fill="x", padx=8, pady=8)
        self.entry = tk.Entry(row, textvariable=self.path, bg=theme.PANEL_2, fg=theme.TEXT,
                             insertbackground=theme.TEXT)
        self.entry.pack(side="left", fill="x", expand=True)
        self.entry.bind("<Return>", lambda _e: self.refresh())
        tk.Button(row, text="打开", command=self.refresh).pack(side="left", padx=5)
        tk.Button(row, text="上一级", command=self.up).pack(side="left")
        self.tree = ttk.Treeview(self, columns=(), show="tree", selectmode="browse")
        self.tree.pack(fill="both", expand=True, padx=8)
        self.tree.bind("<Double-1>", self.open_selected)
        self.tree.bind("<Return>", self.open_selected)
        self.info = tk.Label(self, text="正在读取目录…", bg=theme.BG, fg=theme.TEXT_DIM, anchor="w")
        self.info.pack(fill="x", padx=8, pady=5)
        bottom = tk.Frame(self, bg=theme.BG)
        bottom.pack(fill="x", padx=8, pady=8)
        tk.Button(bottom, text="选择当前目录", command=self.select).pack(side="right")
        tk.Button(bottom, text="取消", command=self.destroy).pack(side="right", padx=8)
        self.transient(parent.winfo_toplevel())
        self.refresh()

    def refresh(self):
        if self.request(self.path.get(), self.scope):
            self.info.config(text="正在读取目录…")
        else:
            self.info.config(text="后台操作进行中，请稍后点击打开")

    def update_listing(self, result, error=None):
        if not self.winfo_exists():
            return
        if error:
            self.info.config(text=error, fg=theme.ERR)
            return
        self.path.set(result.get("path", self.path.get()))
        self.tree.delete(*self.tree.get_children())
        self._paths.clear()
        for item in result.get("entries", []):
            key = self.tree.insert("", "end", text="📁 " + item["name"])
            self._paths[key] = item.get("path", posixpath.join(self.path.get(), item["name"]))
        self.info.config(text="双击进入子目录；也可以手动输入绝对目录。", fg=theme.TEXT_DIM)

    def open_selected(self, _event=None):
        selection = self.tree.selection()
        if selection and selection[0] in self._paths:
            self.path.set(self._paths[selection[0]])
            self.refresh()

    def up(self):
        self.path.set(posixpath.dirname(self.path.get().rstrip("/")) or "/")
        self.refresh()

    def select(self):
        self.choose(self.path.get(), self.scope)
        self.destroy()
