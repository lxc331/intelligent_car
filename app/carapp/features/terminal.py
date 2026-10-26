"""Integrated SSH terminal and a read-only host/container file explorer."""
import base64
import importlib.util
import os
import posixpath
import queue
import re
import time
import tkinter as tk
from tkinter import ttk

from carapp.config import APP_DIR
from carapp.core.ssh_session import SSHSession
from carapp.core.credentials import local_password as local_login
from carapp.features.base import Feature
from carapp.ui import theme


class TerminalFeature(Feature):
    key="terminal"
    title="集成终端"

    def __init__(self,ctx):
        super().__init__(ctx)
        self.session=None
        self.model=None
        self._identity=None
        self._auto_attempted=False
        self._request=0
        self._listing_request=0
        self._items={}
        self._closed=False
        self._last_dimensions=None

    def build(self,parent):
        self.parent=parent
        self.available=all(importlib.util.find_spec(name) is not None for name in ("paramiko","pyte"))
        cfg=self.ctx.cfg["terminal"]
        toolbar=tk.Frame(parent,bg=theme.PANEL)
        toolbar.pack(fill="x",pady=(0,6))
        self.peer=tk.Label(toolbar,bg=theme.PANEL,fg=theme.ACCENT,font=theme.FONT_SMALL)
        self.peer.pack(side="left",padx=8)
        self.user=tk.StringVar(value=cfg["username"])
        self.password=tk.StringVar(value=local_login())
        tk.Label(toolbar,text="用户",bg=theme.PANEL,fg=theme.TEXT_DIM).pack(side="left")
        tk.Entry(toolbar,textvariable=self.user,width=9,bg=theme.PANEL_2,fg=theme.TEXT,
                 insertbackground=theme.TEXT,relief="flat").pack(side="left",padx=4,ipady=4)
        tk.Label(toolbar,text="密码",bg=theme.PANEL,fg=theme.TEXT_DIM).pack(side="left")
        password_entry=tk.Entry(toolbar,textvariable=self.password,show="●",width=15,
                                bg=theme.PANEL_2,fg=theme.TEXT,insertbackground=theme.TEXT,relief="flat")
        password_entry.pack(side="left",padx=4,ipady=4)
        password_entry.bind("<Return>",lambda e:self.connect())
        self.connect_button=self.button(toolbar,"连接 SSH",self.connect)
        self.button(toolbar,"断开",self.disconnect)
        self.status=tk.Label(toolbar,text="未连接",bg=theme.PANEL,fg=theme.TEXT_DIM,font=theme.FONT_SMALL)
        self.status.pack(side="right",padx=10)

        panes=tk.PanedWindow(parent,orient="horizontal",bg=theme.BORDER,sashwidth=6,
                             showhandle=True,bd=0,opaqueresize=True)
        panes.pack(fill="both",expand=True)
        left,right=tk.Frame(panes,bg=theme.PANEL),tk.Frame(panes,bg=theme.PANEL)
        panes.add(left,minsize=400,stretch="always")
        panes.add(right,minsize=330,width=430,stretch="always")
        bar=tk.Frame(left,bg=theme.PANEL)
        bar.pack(fill="x")
        tk.Label(bar,text="远端 Linux 命令行",bg=theme.PANEL,fg=theme.ACCENT,
                 font=theme.FONT_BOLD).pack(side="left",padx=8,pady=8)
        self.button(bar,"Ctrl+C 中断",lambda:self.send("\x03"))
        self.button(bar,"清屏",self.clear)
        self.button(bar,"进入 ClawPi",self.enter_container)
        terminal_frame=tk.Frame(left,bg="#0c0c0c")
        terminal_frame.pack(fill="both",expand=True,padx=4)
        self.text=tk.Text(terminal_frame,bg="#0c0c0c",fg="#d6e4f0",insertbackground=theme.ACCENT,
                          font=theme.FONT_MONO,wrap="none",undo=False,relief="flat",takefocus=True,
                          padx=6,pady=6,width=65,height=25,exportselection=False)
        self.text.pack(side="left",fill="both",expand=True)
        scroll=ttk.Scrollbar(terminal_frame,command=self.text.yview)
        scroll.pack(side="right",fill="y")
        self.text.configure(yscrollcommand=scroll.set)
        self.text.bind("<KeyPress>",self.keypress)
        self.text.bind("<<Paste>>",self.paste)
        self.text.bind("<<Cut>>",lambda e:"break")
        self.text.bind("<Control-Shift-C>",self.copy)
        self.text.bind("<Control-Shift-c>",self.copy)
        self.text.bind("<Control-Shift-V>",self.paste)
        self.text.bind("<Control-Shift-v>",self.paste)
        menu=tk.Menu(self.text,tearoff=False)
        menu.add_command(label="复制所选",command=self.copy)
        menu.add_command(label="粘贴到终端",command=self.paste)
        self.text.bind("<Button-3>",lambda e:menu.tk_popup(e.x_root,e.y_root))
        tk.Label(left,text="Ctrl+C 中断命令 · Ctrl+Shift+C/V 复制/粘贴 · 进入容器按钮须在宿主机提示符下使用",
                 bg=theme.PANEL,fg=theme.TEXT_DIM,font=theme.FONT_SMALL,wraplength=650,anchor="w").pack(fill="x",padx=6,pady=5)

        browser_bar=tk.Frame(right,bg=theme.PANEL)
        browser_bar.pack(fill="x",padx=6,pady=5)
        tk.Label(browser_bar,text="文件系统",bg=theme.PANEL,fg=theme.ACCENT,font=theme.FONT_BOLD).pack(side="left")
        self.scope=tk.StringVar(value="宿主机")
        scope=ttk.Combobox(browser_bar,textvariable=self.scope,values=["宿主机","ClawPi 容器"],state="readonly",width=13)
        scope.pack(side="left",padx=6)
        scope.bind("<<ComboboxSelected>>",lambda e:self.navigate("~"))
        self.button(browser_bar,"刷新",self.refresh)
        paths=tk.Frame(right,bg=theme.PANEL)
        paths.pack(fill="x",padx=6)
        self.button(paths,"上一级",self.up)
        self.path=tk.StringVar(value="~")
        address=tk.Entry(paths,textvariable=self.path,bg=theme.PANEL_2,fg=theme.TEXT,
                         insertbackground=theme.TEXT,relief="flat",font=theme.FONT_SMALL)
        address.pack(side="left",fill="x",expand=True,ipady=4)
        address.bind("<Return>",lambda e:self.navigate(self.path.get()))
        self.button(paths,"转到",lambda:self.navigate(self.path.get()))
        quick=tk.Frame(right,bg=theme.PANEL)
        quick.pack(fill="x",padx=6,pady=4)
        self.button(quick,"用户目录",lambda:self.navigate("~"))
        self.button(quick,"ROS 源码",self.open_ros_source)
        self.button(quick,"预览文件",self.preview)
        self.button(quick,"复制路径",self.copy_path)
        tree_frame=tk.Frame(right,bg=theme.PANEL)
        tree_frame.pack(fill="both",expand=True,padx=6)
        self.tree=ttk.Treeview(tree_frame,columns=("size","modified"),selectmode="browse",show="tree headings")
        self.tree.heading("#0",text="名称（双击打开）")
        self.tree.heading("size",text="大小")
        self.tree.heading("modified",text="修改时间")
        self.tree.column("#0",width=185,minwidth=100,stretch=True)
        self.tree.column("size",width=65,minwidth=50,stretch=False)
        self.tree.column("modified",width=115,minwidth=85,stretch=False)
        self.tree.pack(side="left",fill="both",expand=True)
        tree_scroll=ttk.Scrollbar(tree_frame,command=self.tree.yview)
        tree_scroll.pack(side="right",fill="y")
        self.tree.configure(yscrollcommand=tree_scroll.set)
        self.tree.bind("<Double-1>",self.open_selected)
        self.tree.bind("<Return>",self.open_selected)
        self.file_status=tk.Label(right,text="连接后可查看宿主机及容器；双击目录进入，双击普通文件预览。",
                                  bg=theme.PANEL,fg=theme.TEXT_DIM,wraplength=400,anchor="w",justify="left",font=theme.FONT_SMALL)
        self.file_status.pack(fill="x",padx=6,pady=6)
        if self.available:
            from carapp.core.terminal_buffer import TerminalBuffer
            self.model=TerminalBuffer()
            self.session=SSHSession(os.path.join(APP_DIR,"config","ssh_known_hosts"))
            self.model.feed("集成终端就绪。连接后可输入小车 Linux 命令。\r\n")
        else:
            self.text.insert("end","集成终端需要 paramiko 和 pyte。\n请在电脑安装：\npython -m pip install -r requirements.txt\n")
            self.text.configure(state="disabled")
        self._identity=self.identity()
        self.update_peer()

    def button(self,parent,text,command):
        button=tk.Button(parent,text=text,command=command,bg=theme.PANEL_2,fg=theme.TEXT,
                         activebackground=theme.BORDER,relief="flat",font=theme.FONT_SMALL,padx=7,pady=3)
        button.pack(side="left",padx=3,pady=3)
        return button

    def identity(self):
        car=self.ctx.cfg["car"]
        return car["host"],car.get("profile_id",""),car.get("transport","")

    def update_peer(self):
        host,profile,_=self.identity()
        self.peer.configure(text="%s  %s:%s"%(profile or "手动地址",host,self.ctx.cfg["terminal"]["port"]))

    def connect(self):
        if not self.available:
            self.ctx.log("请先在电脑安装集成终端依赖，见 requirements.txt")
            return
        self._auto_attempted=True
        self.status.configure(text="SSH 连接中…",fg=theme.WARN)
        self.connect_button.configure(state="disabled")
        self.model.reset()
        self.model.feed("正在连接 %s…\r\n"%self.ctx.cfg["car"]["host"])
        self.session.connection_guard=self.ctx.app._connection_guard()
        self.session.connect(self.ctx.cfg["car"]["host"],self.user.get().strip(),self.password.get(),
                             port=int(self.ctx.cfg["terminal"]["port"]),profile_id=self.ctx.cfg["car"].get("profile_id",""))

    def disconnect(self):
        self._auto_attempted=True
        if self.session:
            self.session.disconnect()
        self.status.configure(text="SSH 未连接",fg=theme.TEXT_DIM)
        self.connect_button.configure(state="normal")

    def on_vehicle_change(self):
        self.disconnect()
        self._listing_request=-1
        self._items.clear()
        self.tree.delete(*self.tree.get_children())
        self.path.set("~")
        if self.model:
            self.model.reset()
            self.model.feed("已切换车辆。旧 SSH 会话已关闭，请点击连接 SSH。\r\n")

    def send(self,text):
        if not self.session or not self.session.send(text):
            self.ctx.log("SSH 未连接或发送队列繁忙")
        return "break"

    def keypress(self,event):
        if event.state & 4:
            key=event.keysym.lower()
            if event.state & 1 and key in ("c","v"):
                return self.copy() if key=="c" else self.paste()
            if len(key)==1 and 'a'<=key<='z':
                return self.send(chr(ord(key)-96))
            return "break"
        keys={"Return":"\r","KP_Enter":"\r","BackSpace":"\x7f","Tab":"\t","ISO_Left_Tab":"\x1b[Z",
              "Escape":"\x1b","Up":"\x1b[A","Down":"\x1b[B","Right":"\x1b[C","Left":"\x1b[D",
              "Home":"\x1b[H","End":"\x1b[F","Delete":"\x1b[3~","Prior":"\x1b[5~","Next":"\x1b[6~"}
        if event.keysym in keys:
            return self.send(keys[event.keysym])
        if event.char and event.char.isprintable():
            return self.send(("\x1b" if event.state & 8 else "")+event.char)
        return "break"

    def copy(self,event=None):
        try:
            text=self.text.get("sel.first","sel.last")
            self.text.clipboard_clear();self.text.clipboard_append(text)
        except tk.TclError:
            pass
        return "break"

    def paste(self,event=None):
        try:
            self.send(self.text.clipboard_get())
        except tk.TclError:
            pass
        return "break"

    def clear(self):
        if self.model:
            self.model.reset()
            if self.session.connected:self.session.send("\x0c")

    def enter_container(self):
        command=("sudo -n docker exec -it -u ubuntu -w /home/ubuntu ClawPi bash -lc "
                 "'source /opt/ros/humble/setup.bash; source /home/ubuntu/ros2_ws/install/setup.bash; exec bash -i'\r")
        self.send(command)
        self.text.focus_set()

    def remote_scope(self):
        return "container" if self.scope.get()=="ClawPi 容器" else "host"

    def navigate(self,path):
        if not self.session or not self.session.connected:
            self.file_status.configure(text="请先连接 SSH")
            return
        self._request+=1
        self._listing_request=self._request
        self.file_status.configure(text="正在读取目录…")
        self.session.list_directory(path,self.remote_scope(),self._request)

    def refresh(self):
        self.navigate(self.path.get())

    def up(self):
        current=self.path.get()
        self.navigate(posixpath.dirname(current.rstrip('/')) or '/' if current!='~' else '~')

    def open_ros_source(self):
        self.scope.set("ClawPi 容器")
        self.navigate("/home/ubuntu/ros2_ws/src")

    def selected(self):
        rows=self.tree.selection()
        return self._items.get(rows[0]) if rows else None

    def open_selected(self,event=None):
        item=self.selected()
        if item:
            self.navigate(item["path"]) if item["is_dir"] else self.preview()
        return "break"

    def preview(self):
        item=self.selected()
        if not item or item["is_dir"] or not self.session.connected:
            return
        self._request+=1
        self.session.read_file(item["path"],self.remote_scope(),self._request)

    def copy_path(self):
        item=self.selected()
        self.parent.clipboard_clear()
        self.parent.clipboard_append(item["path"] if item else self.path.get())

    def directory_result(self,data):
        if data["request_id"]!=self._listing_request or data["scope"]!=self.remote_scope():
            return
        self.path.set(data["path"])
        self.tree.delete(*self.tree.get_children())
        self._items.clear()
        for index,item in enumerate(data["entries"]):
            item["path"]=posixpath.join(data["path"],item["name"])
            iid="f%d"%index
            self._items[iid]=item
            size="目录" if item["is_dir"] else self.size(item["size"])
            modified=time.strftime("%m-%d %H:%M",time.localtime(item["mtime"])) if item["mtime"] else "--"
            self.tree.insert("","end",iid=iid,text=("▸ " if item["is_dir"] else "  ")+item["name"],values=(size,modified))
        self.file_status.configure(text="%d 项%s · 双击目录进入，文件只读预览"%
                                  (len(data["entries"]),"（仅显示前 5000 项）" if data.get("truncated") else ""))

    @staticmethod
    def size(value):
        return "%.1f M"%(value/1048576) if value>=1048576 else ("%.1f K"%(value/1024) if value>=1024 else str(value))

    def file_result(self,data):
        raw=base64.b64decode(data["data"])
        if b'\x00' in raw[:8192] and not raw.startswith((b'\xff\xfe',b'\xfe\xff')):
            self.file_status.configure(text="二进制文件：%s（%s 字节）"%(posixpath.basename(data["path"]),data["size"]))
            return
        text=raw.decode("utf-16" if raw.startswith((b'\xff\xfe',b'\xfe\xff')) else "utf-8-sig","replace")
        window=tk.Toplevel(self.parent)
        window.title("只读预览 · "+data["path"])
        window.geometry("900x650")
        window.configure(bg=theme.BG)
        tk.Label(window,text=data["path"]+("（仅预览前 256 KB）" if data.get("truncated") else ""),
                 bg=theme.PANEL,fg=theme.ACCENT,anchor="w").pack(fill="x",padx=8,pady=6)
        body=tk.Frame(window,bg=theme.BG);body.pack(fill="both",expand=True,padx=8,pady=8)
        viewer=tk.Text(body,bg=theme.PANEL,fg=theme.TEXT,font=theme.FONT_MONO,wrap="none")
        viewer.pack(side="left",fill="both",expand=True)
        scroll=ttk.Scrollbar(body,command=viewer.yview);scroll.pack(side="right",fill="y")
        viewer.configure(yscrollcommand=scroll.set)
        viewer.insert("1.0",text);viewer.configure(state="disabled")

    def on_tick(self,st):
        if self._closed:return
        if self.identity()!=self._identity:
            self.on_vehicle_change()
            self._identity=self.identity()
            self.update_peer()
        if not self.available:return
        if not self._auto_attempted and self.ctx.cfg["terminal"].get("auto_connect",True):
            if self.ctx.app.nb.index("current")==self.ctx.app.features.index(self) and self.password.get():
                self.connect()
        for _ in range(150):
            try:generation,kind,data=self.session.events.get_nowait()
            except queue.Empty:break
            if generation!=self.session.generation:continue
            if kind=="data":self.model.feed(data)
            elif kind=="connected":
                self.status.configure(text="SSH 已连接",fg=theme.OK)
                self.connect_button.configure(state="normal")
                self.model.reset()
                self.navigate("~")
            elif kind=="notice":self.ctx.log("SSH："+data)
            elif kind in ("error","disconnected"):
                self.status.configure(text="SSH 未连接",fg=theme.ERR if kind=="error" else theme.TEXT_DIM)
                self.connect_button.configure(state="normal")
                self.ctx.log("SSH："+data)
                self.model.feed("\r\n["+data+"]\r\n")
            elif kind=="directory":self.directory_result(data)
            elif kind=="file":self.file_result(data)
            elif kind=="operation_error":self.file_status.configure(text="读取失败："+data["error"])
        self.fit_terminal()
        if self.model.dirty:
            content,cursor=self.model.render()
            bottom=self.text.yview()[1]>=.98
            position=self.text.yview()[0]
            selection=self.text.tag_ranges("sel")
            self.text.delete("1.0","end");self.text.insert("1.0",content)
            self.text.mark_set("insert","%d.%d"%cursor)
            if selection:
                self.text.tag_add("sel",*selection)
            if bottom:self.text.see("insert")
            else:self.text.yview_moveto(position)

    def fit_terminal(self):
        import tkinter.font as font
        width,height=self.text.winfo_width()-14,self.text.winfo_height()-14
        if width<100 or height<80:return
        metrics=font.Font(font=self.text.cget("font"))
        dimensions=(max(30,min(240,width//max(1,metrics.measure("M")))),max(8,min(100,height//metrics.metrics("linespace"))))
        if dimensions!=self._last_dimensions:
            self._last_dimensions=dimensions
            self.model.resize(*dimensions);self.session.resize(*dimensions)

    def on_close(self):
        self._closed=True
        self.password.set("")
        if self.session:self.session.close()
