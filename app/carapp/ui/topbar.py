# -*- coding: utf-8 -*-
"""顶部状态条：左边是小车地址与控制按钮，右边是实时状态灯。"""

import tkinter as tk
from tkinter import ttk, simpledialog

from carapp.ui import theme
from carapp.core.connection_profiles import TRANSPORT_LABELS, endpoint_for


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
        left.pack(fill="x", padx=10, pady=(6, 2))

        self.lbl_connection_help = tk.Label(left, bg=theme.PANEL, fg=theme.TEXT_DIM,
                                            font=theme.FONT_SMALL, anchor="w")
        self.lbl_connection_help.pack(anchor="w")
        row = tk.Frame(left, bg=theme.PANEL)
        row.pack(anchor="w")

        # ---- 多车下拉：配置里 cars 有几项就列几项 ----
        self.cars = list(app.cfg.get("cars") or [])
        self.var_car = tk.StringVar()
        names = [self._car_label(c) for c in self.cars]
        if self.cars:
            cur = app.cfg["car"].get("profile_id")
            for c in self.cars:
                if c.get("id") == cur:
                    self.var_car.set(self._car_label(c))
                    break
            else:
                self.var_car.set("请选择车辆" if app.cfg["car"].get("transport") == "wifi" else "手动地址")
            self.om_car = tk.OptionMenu(row, self.var_car, *names, command=self._pick_car)
            self.om_car.config(bg=theme.PANEL_2, fg=theme.ACCENT, relief="flat", bd=0,
                               highlightthickness=1, highlightbackground=theme.BORDER,
                               font=theme.FONT_SMALL, width=24, anchor="w",
                               activebackground=theme.BORDER, activeforeground=theme.TEXT)
            self.om_car["menu"].config(bg=theme.PANEL_2, fg=theme.TEXT,
                                       activebackground=theme.BORDER,
                                       font=theme.FONT_SMALL)
            self.om_car.pack(side="left", padx=(0, 6))

        self.var_transport = tk.StringVar(value=TRANSPORT_LABELS.get(app.cfg["car"].get("transport"), "有线网口"))
        ttk.Style(self).map("Connection.TCombobox",
                            fieldbackground=[("readonly", theme.PANEL_2)],
                            foreground=[("readonly", theme.TEXT)],
                            selectbackground=[("readonly", theme.PANEL_2)],
                            selectforeground=[("readonly", theme.TEXT)])
        self.cb_transport = ttk.Combobox(row, textvariable=self.var_transport,
                                        values=list(TRANSPORT_LABELS.values()), width=12, state="readonly",
                                        style="Connection.TCombobox")
        self.cb_transport.pack(side="left", padx=(0, 6))
        self.cb_transport.bind("<<ComboboxSelected>>", lambda _: self._pick_car(self.var_car.get()))

        self.var_host = tk.StringVar(value=app.cfg["car"]["host"])
        entry = tk.Entry(row, textvariable=self.var_host, width=16,
                         bg=theme.PANEL_2, fg=theme.TEXT, insertbackground=theme.TEXT,
                         relief="flat", font=theme.FONT_MONO)
        entry.pack(side="left", ipady=3, padx=(0, 6))

        self.sta_options = tk.Frame(row, bg=theme.PANEL)
        tk.Label(self.sta_options, text="Wi-Fi", bg=theme.PANEL, fg=theme.TEXT_DIM,
                 font=theme.FONT_SMALL).pack(side="left", padx=(0, 3))
        self.var_sta_ssid = tk.StringVar()
        tk.Entry(self.sta_options, textvariable=self.var_sta_ssid, width=18,
                 bg=theme.PANEL_2, fg=theme.TEXT, insertbackground=theme.TEXT,
                 relief="flat", font=theme.FONT_SMALL).pack(side="left", ipady=3, padx=(0, 6))

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

        self.btn_diagnose = tk.Button(row, text="连接诊断", command=self._diagnose,
                                     bg=theme.PANEL_2, fg=theme.ACCENT, relief="flat",
                                     activebackground=theme.BORDER, font=theme.FONT_SMALL, padx=10)
        self.btn_diagnose.pack(side="left", padx=2)

        right = tk.Frame(self, bg=theme.PANEL)
        right.pack(fill="x", padx=10, pady=(3, 7))

        self.lbl_conn = _chip(right, "ROS 连接")
        self.lbl_node = _chip(right, "节点数")
        self.lbl_batt = _chip(right, "电池")
        self.lbl_pose = _chip(right, "位姿 x,y")
        self.lbl_cam = _chip(right, "摄像头")
        self.lbl_grip = _chip(right, "夹爪目标")
        self.lbl_base = _chip(right, "底盘")
        self.sync_selection()

    # ---------- 交互 ----------
    @staticmethod
    def _car_label(car):
        return "%s · %s" % (car.get("name", ""), car.get("id", ""))

    def sync_selection(self):
        car = self.app.cfg["car"]
        self.var_host.set(car["host"])
        self.var_transport.set(TRANSPORT_LABELS.get(car.get("transport"), "有线网口"))
        profile = next((p for p in self.cars if p["id"] == car.get("profile_id")), None)
        self.var_car.set(self._car_label(profile) if profile else "手动地址")
        self.var_sta_ssid.set((profile or {}).get("sta_ssid", ""))
        self._refresh_transport_help()

    def _selected_transport(self):
        return next((key for key, label in TRANSPORT_LABELS.items() if label == self.var_transport.get()), "wired")

    def _refresh_transport_help(self):
        mode = self._selected_transport()
        self.sta_options.pack_forget()
        if mode == "sta":
            self.sta_options.pack(side="left", before=self.btn_apply)
            text = "STA 局域网：电脑和小车接入同一网络，填写小车无线 IP；Wi-Fi 名称可选，仅用于提示"
        elif mode == "wifi":
            text = "AP 热点：先在系统连接所选车辆的 HW-* 热点，再连接 192.168.149.1"
        else:
            text = "有线网口：连接网线，并确保电脑有线网卡与小车处于同一网段"
        self.lbl_connection_help.config(text=text)

    def _pick_car(self, name):
        """下拉选车：把该车的 host 填进地址框并立刻重连。"""
        for c in self.cars:
            if self._car_label(c) == name:
                transport = self._selected_transport()
                host = endpoint_for(c, transport)
                if transport == "sta" and not host:
                    host = simpledialog.askstring("STA 局域网地址", "%s\n请输入小车在路由器/热点中获得的 IPv4 地址：" % name,
                                                  parent=self.winfo_toplevel())
                    if not host:
                        self.sync_selection()
                        return
                if host:
                    self.app.log("切换小车：%s（%s）" % (name, host))
                    if not self.app.change_host(host, c["id"], transport,
                                                sta_ssid=c.get("sta_ssid", "") if transport == "sta" else None):
                        self.sync_selection()
                return

    def _apply(self):
        host = self.var_host.get().strip()
        if not host:
            return
        profile = next((c for c in self.cars if self._car_label(c) == self.var_car.get()), {})
        transport = self._selected_transport()
        profile_id = profile.get("id") if transport in ("wifi", "sta") else None
        if not self.app.change_host(host, profile_id, transport,
                                    sta_ssid=self.var_sta_ssid.get().strip() if transport == "sta" else None):
            self.sync_selection()

    def _scan(self):
        self.btn_scan.config(state="disabled", text="搜索中…")
        self.app.discover_car(self._scan_done)

    def _scan_done(self, found):
        self.btn_scan.config(state="normal", text="自动搜索")
        if not found:
            self.app.log("自动搜索未找到小车，请检查所选模式、连接网络和小车当前 IP")
            return
        # 即使只搜到一个端口候选也由用户选择，不能把别台车自动当作当前车。
        self.app.log("搜索到 %d 个服务候选：%s" % (len(found), "、".join(found)))
        self._ask_which(found)

    def _use(self, host):
        profile = next((c for c in self.cars if self._car_label(c) == self.var_car.get()), {})
        transport = self._selected_transport()
        profile_id = profile.get("id") if transport in ("wifi", "sta") else None
        if self.app.change_host(host, profile_id, transport,
                                sta_ssid=self.var_sta_ssid.get().strip() if transport == "sta" else None):
            self.app.log("使用所选服务地址：%s" % host)

    def _ask_which(self, found):
        win = tk.Toplevel(self)
        win.title("选择要连接的小车")
        win.configure(bg=theme.BG)
        win.transient(self.winfo_toplevel())
        win.resizable(False, False)

        tk.Label(win, text="选择要连接的地址（候选可能只有部分服务就绪）：", bg=theme.BG, fg=theme.TEXT,
                 font=theme.FONT).pack(anchor="w", padx=14, pady=(12, 6))

        var = tk.StringVar(value=found[0])
        for ip in found:
            profile = next((c for c in self.cars if endpoint_for(c, self._selected_transport()) == ip
                            and self._selected_transport() != "wifi"), None)
            label = "%s  %s" % (ip, self._car_label(profile) if profile else
                                  ("共享热点，请先选对车辆与 SSID" if ip == "192.168.149.1" else "未识别设备"))
            tk.Radiobutton(win, text=label, variable=var, value=ip, bg=theme.BG,
                           fg=theme.TEXT, selectcolor=theme.PANEL_2,
                           activebackground=theme.BG, activeforeground=theme.TEXT,
                           font=theme.FONT_MONO, anchor="w").pack(fill="x", padx=18)

        def ok():
            win.destroy()
            self._use(var.get())

        tk.Button(win, text="连接", command=ok, bg=theme.PANEL_2, fg=theme.ACCENT,
                  relief="flat", font=theme.FONT_SMALL, padx=18, pady=3,
                  activebackground=theme.BORDER).pack(pady=10)

    def _diagnose(self):
        self.btn_diagnose.config(state="disabled", text="诊断中…")
        self.app.diagnose_connection(self._diagnose_done)

    def _diagnose_done(self, report):
        self.btn_diagnose.config(state="normal", text="连接诊断")
        win = tk.Toplevel(self)
        win.title("连接诊断（只读）")
        win.transient(self.winfo_toplevel())
        text = tk.Text(win, width=85, height=23, wrap="word", font=theme.FONT_SMALL,
                       bg=theme.PANEL, fg=theme.TEXT, padx=12, pady=12)
        text.pack(fill="both", expand=True)
        text.insert("1.0", report)
        text.config(state="disabled")

    # ---------- 刷新 ----------
    def update_state(self, st):
        if st["ws_connected"]:
            self.lbl_conn.config(text="● 已连接", fg=theme.OK)
        else:
            self.lbl_conn.config(text="● 重连中", fg=theme.WARN)

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
        vy = float(st.get("base_vy", 0.0))
        if vx or vy or wz:
            axis, value = max((("vx", vx), ("vy", vy), ("wz", wz)), key=lambda p: abs(p[1]))
            self.lbl_base.config(text="%s %.2f" % (axis, value), fg=theme.WARN)
        else:
            self.lbl_base.config(text="停", fg=theme.TEXT_DIM)
