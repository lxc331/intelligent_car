# -*- coding: utf-8 -*-
"""顶部状态条：左边是小车地址与控制按钮，右边是实时状态灯。"""

import tkinter as tk
from tkinter import ttk

from carapp.ui import theme


def _chip(parent, title):
    box = tk.Frame(parent, bg=theme.PANEL)
    box.pack(side="left", padx=(0, 14))
    tk.Label(box, text=title, bg=theme.PANEL, fg=theme.TEXT_DIM,
             font=theme.FONT_SMALL).pack(anchor="w")
    val = tk.Label(box, text="--", bg=theme.PANEL, fg=theme.TEXT, font=theme.FONT_MONO)
    val.pack(anchor="w")
    return val


class TopBar(tk.Frame):
    def __init__(self, parent, app):
        super().__init__(parent, bg=theme.PANEL, height=64)
        self.app = app

        left = tk.Frame(self, bg=theme.PANEL)
        left.pack(side="left", padx=10, pady=8)

        tk.Label(left, text="小车", bg=theme.PANEL, fg=theme.TEXT_DIM,
                 font=theme.FONT_SMALL).pack(anchor="w")
        row = tk.Frame(left, bg=theme.PANEL)
        row.pack(anchor="w")

        # ---- 多车下拉：配置里 cars 有几项就列几项 ----
        self.cars = list(app.cfg.get("cars") or [])
        self.var_car = tk.StringVar()
        names = [c.get("name") or c.get("host") for c in self.cars]
        if self.cars:
            cur = app.cfg["car"]["host"]
            for c in self.cars:
                if c.get("host") == cur:
                    self.var_car.set(c.get("name") or cur)
                    break
            else:
                self.var_car.set(names[0])
            self.om_car = tk.OptionMenu(row, self.var_car, *names, command=self._pick_car)
            self.om_car.config(bg=theme.PANEL_2, fg=theme.ACCENT, relief="flat", bd=0,
                               highlightthickness=1, highlightbackground=theme.BORDER,
                               font=theme.FONT_SMALL, width=7, anchor="w",
                               activebackground=theme.BORDER, activeforeground=theme.TEXT)
            self.om_car["menu"].config(bg=theme.PANEL_2, fg=theme.TEXT,
                                       activebackground=theme.BORDER,
                                       font=theme.FONT_SMALL)
            self.om_car.pack(side="left", padx=(0, 6))

        self.var_host = tk.StringVar(value=app.cfg["car"]["host"])
        entry = tk.Entry(row, textvariable=self.var_host, width=16,
                         bg=theme.PANEL_2, fg=theme.TEXT, insertbackground=theme.TEXT,
                         relief="flat", font=theme.FONT_MONO)
        entry.pack(side="left", ipady=3, padx=(0, 6))

        self.btn_apply = tk.Button(row, text="应用", command=self._apply,
                                   bg=theme.PANEL_2, fg=theme.TEXT, relief="flat",
                                   activebackground=theme.BORDER, font=theme.FONT_SMALL,
                                   padx=10)
        self.btn_apply.pack(side="left", padx=2)

        self.btn_scan = tk.Button(row, text="自动搜索", command=self._scan,
                                  bg=theme.PANEL_2, fg=theme.ACCENT, relief="flat",
                                  activebackground=theme.BORDER, font=theme.FONT_SMALL,
                                  padx=10)
        self.btn_scan.pack(side="left", padx=2)

        right = tk.Frame(self, bg=theme.PANEL)
        right.pack(side="right", padx=10)

        self.lbl_conn = _chip(right, "ROS 连接")
        self.lbl_node = _chip(right, "节点数")
        self.lbl_batt = _chip(right, "电池")
        self.lbl_pose = _chip(right, "位姿 x,y")
        self.lbl_cam = _chip(right, "摄像头")
        self.lbl_grip = _chip(right, "夹爪目标")
        self.lbl_base = _chip(right, "底盘")

    # ---------- 交互 ----------
    def _pick_car(self, name):
        """下拉选车：把该车的 host 填进地址框并立刻重连。"""
        for c in self.cars:
            if (c.get("name") or c.get("host")) == name:
                host = c.get("host")
                if host:
                    self.var_host.set(host)
                    self.app.log("切换小车：%s（%s）" % (name, host))
                    self.app.change_host(host)
                return

    def _apply(self):
        host = self.var_host.get().strip()
        if not host:
            return
        # 手填的地址如果正好是档案里的某台车，把下拉同步过去
        for c in self.cars:
            if c.get("host") == host:
                self.var_car.set(c.get("name") or host)
                break
        self.app.change_host(host)

    def _scan(self):
        self.btn_scan.config(state="disabled", text="搜索中…")
        self.app.discover_car(self._scan_done)

    def _scan_done(self, found):
        self.btn_scan.config(state="normal", text="自动搜索")
        if not found:
            self.app.log("自动搜索未找到小车，请检查网线与本机网段")
            return
        if len(found) == 1:
            self._use(found[0])
            return
        # 多台：列出来让用户选（两台同款车很常见）
        self.app.log("搜索到 %d 台小车：%s" % (len(found), "、".join(found)))
        self._ask_which(found)

    def _use(self, host):
        self.var_host.set(host)
        for c in self.cars:
            if c.get("host") == host:
                self.var_car.set(c.get("name") or host)
                break
        self.app.change_host(host)
        self.app.log("自动搜索到小车：%s" % host)

    def _ask_which(self, found):
        win = tk.Toplevel(self)
        win.title("选择要连接的小车")
        win.configure(bg=theme.BG)
        win.transient(self.winfo_toplevel())
        win.resizable(False, False)

        tk.Label(win, text="找到多台小车，选一台连接：", bg=theme.BG, fg=theme.TEXT,
                 font=theme.FONT).pack(anchor="w", padx=14, pady=(12, 6))

        var = tk.StringVar(value=found[0])
        for ip in found:
            tk.Radiobutton(win, text=ip, variable=var, value=ip, bg=theme.BG,
                           fg=theme.TEXT, selectcolor=theme.PANEL_2,
                           activebackground=theme.BG, activeforeground=theme.TEXT,
                           font=theme.FONT_MONO, anchor="w").pack(fill="x", padx=18)

        def ok():
            win.destroy()
            self._use(var.get())

        tk.Button(win, text="连接", command=ok, bg=theme.PANEL_2, fg=theme.ACCENT,
                  relief="flat", font=theme.FONT_SMALL, padx=18, pady=3,
                  activebackground=theme.BORDER).pack(pady=10)

    # ---------- 刷新 ----------
    def update_state(self, st):
        if st["ws_connected"]:
            self.lbl_conn.config(text="● 已连接", fg=theme.OK)
        else:
            self.lbl_conn.config(text="● 未连接", fg=theme.ERR)

        nc = st.get("node_count")
        self.lbl_node.config(text=str(nc) if nc is not None else "--",
                             fg=theme.TEXT if nc else theme.TEXT_DIM)

        mv = st.get("battery_mv")
        if mv:
            volt = mv / 1000.0 if mv > 100 else float(mv)
            color = theme.OK if volt >= 7.0 else (theme.WARN if volt >= 6.6 else theme.ERR)
            self.lbl_batt.config(text="%.2f V" % volt, fg=color)
        else:
            self.lbl_batt.config(text="--", fg=theme.TEXT_DIM)

        pose = st.get("pose")
        if pose:
            self.lbl_pose.config(text="%.2f, %.2f" % (pose[0], pose[1]))
        else:
            self.lbl_pose.config(text="--", fg=theme.TEXT_DIM)

        size = st.get("cam_size")
        if size:
            self.lbl_cam.config(text="%dx%d %.1ffps" % (size[0], size[1], st.get("cam_fps", 0)))
        else:
            self.lbl_cam.config(text="--", fg=theme.TEXT_DIM)

        g = st.get("gripper_target")
        self.lbl_grip.config(text=str(g) if g is not None else "--")

        cmd = st.get("base_cmd") or (0.0, 0.0)
        vx, wz = float(cmd[0]), float(cmd[1])
        if vx or wz:
            if abs(vx) >= abs(wz):
                self.lbl_base.config(text="vx %.2f" % vx, fg=theme.WARN)
            else:
                self.lbl_base.config(text="wz %.2f" % wz, fg=theme.WARN)
        else:
            self.lbl_base.config(text="停", fg=theme.TEXT_DIM)
