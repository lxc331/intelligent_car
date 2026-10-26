"""Mouse/keyboard teleoperation with a shared live camera stream."""
import tkinter as tk

from carapp.core.teleop import CONTROL_KEYS, TeleopController
from carapp.features.base import Feature
from carapp.ui import theme
from carapp.ui.camerapanel import CameraPanel


class RemoteFeature(Feature):
    key = "remote"
    title = "远程遥控 + 实时画面"

    def __init__(self, ctx):
        super().__init__(ctx)
        self._active = False
        self._armed = False
        self._mouse_key = None
        self._releases = {}
        self.controller = None

    def build(self, parent):
        self.controller = TeleopController(self.ctx)
        mode = tk.Frame(parent, bg=theme.PANEL)
        mode.pack(fill="x", padx=6, pady=6)
        tk.Label(mode, text="操控模式", bg=theme.PANEL, fg=theme.TEXT,
                 font=theme.FONT_BOLD).pack(side="left", padx=10)
        self.var_mode = tk.StringVar(value="mouse")
        for name, value in (("鼠标点击操控", "mouse"), ("键盘操控", "keyboard")):
            tk.Radiobutton(mode, text=name, variable=self.var_mode, value=value,
                           command=self.change_mode, bg=theme.PANEL, fg=theme.TEXT,
                           selectcolor=theme.BG, activebackground=theme.PANEL,
                           activeforeground=theme.ACCENT, font=theme.FONT).pack(side="left", padx=8)
        tk.Label(mode, text="速度", bg=theme.PANEL, fg=theme.TEXT_DIM,
                 font=theme.FONT_SMALL).pack(side="left", padx=(18, 4))
        base = self.ctx.cfg.get("base", {})
        levels = list(base.get("speed_levels", {"慢": .06, "中": .1, "快": .15}))
        default = base.get("default_level", "慢")
        self.var_level = tk.StringVar(value=default if default in levels else levels[0])
        option = tk.OptionMenu(mode, self.var_level, *levels, command=self.speed_changed)
        option.config(bg=theme.PANEL_2, fg=theme.TEXT, highlightthickness=0,
                      activebackground=theme.BORDER, relief="flat", font=theme.FONT_SMALL, width=4)
        option["menu"].config(bg=theme.PANEL_2, fg=theme.TEXT, font=theme.FONT_SMALL)
        option.pack(side="left")
        self._button(mode, "■ 停止 / 退出键盘", self.stop_all, theme.ERR).pack(side="right", padx=8)

        self.mouse_panel = tk.Frame(parent, bg=theme.BG)
        self._build_mouse(self.mouse_panel)
        self.keyboard_panel = tk.Frame(parent, bg=theme.BG)
        self._build_keyboard(self.keyboard_panel)
        self.mouse_panel.pack(fill="x", padx=6)

        self.status = tk.Label(parent, text="鼠标模式：按住方向按钮移动，松开或移出按钮即停。",
                               bg=theme.BG, fg=theme.ACCENT, anchor="w", font=theme.FONT_SMALL)
        self.status.pack(fill="x", padx=12, pady=(5, 0))
        self.targets = tk.Label(parent, text="舵机目标：--（非位置回读）", bg=theme.BG,
                                fg=theme.TEXT_DIM, anchor="w", font=theme.FONT_SMALL)
        self.targets.pack(fill="x", padx=12, pady=(2, 0))
        self.cam = CameraPanel(parent, self.ctx)
        self.cam.pack(fill="both", expand=True, pady=(6, 6))
        self.speed_changed()

    def _button(self, parent, label, command=None, color=None):
        return tk.Button(parent, text=label, command=command, bg=theme.PANEL_2,
                         fg=color or theme.TEXT, activebackground=theme.BORDER,
                         activeforeground=theme.TEXT, relief="flat", font=theme.FONT_SMALL,
                         padx=10, pady=6)

    def _build_mouse(self, parent):
        left = tk.LabelFrame(parent, text="车轮 · 按住持续移动，松开停止",
                             bg=theme.PANEL, fg=theme.TEXT_DIM, font=theme.FONT_SMALL,
                             padx=8, pady=6)
        left.pack(side="left", fill="both", expand=True, padx=(0, 6))
        for label, key, row, col in (
            ("⇤ 平移左转", "q", 0, 0), ("↑ 前进", "w", 0, 1), ("平移右转 ⇥", "e", 0, 2),
            ("↶ 左转", "a", 1, 0), ("↓ 后退", "s", 1, 1), ("右转 ↷", "d", 1, 2),
        ):
            self._hold(left, label, key).grid(row=row, column=col, sticky="ew", padx=3, pady=3)
        for col in range(3):
            left.columnconfigure(col, weight=1)
        tk.Label(left, text="平移为横向移动，车头朝向保持不变。", bg=theme.PANEL,
                 fg=theme.TEXT_DIM, font=theme.FONT_SMALL).grid(row=2, column=0, columnspan=3, pady=4)

        right = tk.LabelFrame(parent, text="云台 / 升降臂 / 夹爪",
                              bg=theme.PANEL, fg=theme.TEXT_DIM, font=theme.FONT_SMALL,
                              padx=8, pady=6)
        right.pack(side="left", fill="both", expand=True)
        for label, key, row, col in (
            ("云台仰视", "i", 0, 0), ("云台俯视", "k", 0, 1),
            ("云台水平左转", "j", 0, 2), ("云台水平右转", "l", 0, 3),
            ("升降臂抬起", "t", 1, 0), ("升降臂降下", "g", 1, 1),
            ("张开夹爪", "f", 1, 2), ("闭合夹爪", "h", 1, 3),
        ):
            self._hold(right, label, key).grid(row=row, column=col, sticky="ew", padx=3, pady=3)
        for col in range(4):
            right.columnconfigure(col, weight=1)
        tk.Label(right, text="云台按住微调；升降臂、夹爪单击执行。", bg=theme.PANEL,
                 fg=theme.TEXT_DIM, font=theme.FONT_SMALL).grid(row=2, column=0, columnspan=4, pady=4)

    def _hold(self, parent, text, key):
        button = self._button(parent, text)
        button.bind("<ButtonPress-1>", lambda event: self.mouse_press(key))
        button.bind("<ButtonRelease-1>", lambda event: self.mouse_release())
        button.bind("<Leave>", lambda event: self.mouse_release())
        return button

    def _build_keyboard(self, parent):
        header = tk.Frame(parent, bg=theme.BG)
        header.pack(fill="x")
        self._button(header, "点击启用键盘操控", self.enable_keyboard, theme.ACCENT).pack(side="left")
        tk.Label(header, text="启用后按住控制键；空格 / Esc 停止。点击其他输入框即退出键盘操控。",
                 bg=theme.BG, fg=theme.TEXT_DIM, font=theme.FONT_SMALL).pack(side="left", padx=10)
        self.key_surface = tk.Frame(parent, bg=theme.PANEL, takefocus=True,
                                    highlightthickness=2, highlightbackground=theme.BORDER,
                                    highlightcolor=theme.ACCENT, padx=12, pady=8)
        self.key_surface.pack(fill="x", pady=(6, 0))
        for text in (
            "W 前进    S 后退    A 左转    D 右转    Q 平移左转    E 平移右转",
            "I 云台仰视    K 云台俯视    J 云台水平左转    L 云台水平右转",
            "T 升降臂抬起    G 升降臂降下    F 张开夹爪    H 闭合夹爪",
        ):
            tk.Label(self.key_surface, text=text, bg=theme.PANEL, fg=theme.TEXT,
                     font=theme.FONT, anchor="w").pack(fill="x", pady=3)
        self.key_surface.bind("<KeyPress>", self.key_press)
        self.key_surface.bind("<KeyRelease>", self.key_release)
        self.key_surface.bind("<FocusOut>", self.focus_lost)

    def focus_lost(self, _event=None):
        self.stop_all()

    def _ready(self):
        if not self._active or not self.ctx.state.get("ws_connected"):
            self.ctx.log("请进入遥控页并等待 ROS 连接后再操作")
            return False
        if self.ctx.motion_in_use(exclude=self):
            self.ctx.log("请等待其他底盘或夹取动作停止后再遥控")
            return False
        return True

    def speed_changed(self, _value=None):
        cfg = self.ctx.cfg.get("base", {})
        self.controller.set_speed(cfg.get("speed_levels", {}).get(self.var_level.get(), .06),
                                  cfg.get("turn_speed", .4))

    def change_mode(self):
        self.stop_all()
        self.mouse_panel.pack_forget()
        self.keyboard_panel.pack_forget()
        panel = self.keyboard_panel if self.var_mode.get() == "keyboard" else self.mouse_panel
        panel.pack(fill="x", padx=6, before=self.status)

    def enable_keyboard(self):
        if self.var_mode.get() != "keyboard" or not self._ready():
            return
        self.stop_all()
        with self.controller.lock:
            self.controller.blocked.clear()
        self._armed = True
        self.key_surface.focus_set()

    def key_press(self, event):
        key = event.keysym.lower()
        if key in ("space", "escape"):
            self.stop_all()
            return "break"
        if not self._active or not self._armed or self.var_mode.get() != "keyboard":
            return
        # Ctrl/Alt combinations belong to the desktop, never to vehicle motion.
        if event.state & (0x0004 | 0x0008 | 0x20000):
            self.stop_all()
            return
        if key not in CONTROL_KEYS:
            return
        pending = self._releases.pop(key, None)
        if pending:
            self.ctx.root.after_cancel(pending)
        elif self._ready():
            self.controller.press(key)
        return "break"

    def key_release(self, event):
        key = event.keysym.lower()
        if key in CONTROL_KEYS:
            if key in self._releases:
                self.ctx.root.after_cancel(self._releases[key])
            # X11/Tk may emit a release/press pair for OS repeat; debounce the pair.
            self._releases[key] = self.ctx.root.after(20, lambda: self._release_key(key))
            return "break"

    def _release_key(self, key):
        self._releases.pop(key, None)
        self.controller.release(key)

    def mouse_press(self, key):
        if self.var_mode.get() != "mouse" or not self._ready():
            return
        self.mouse_release()
        self._mouse_key = key
        self.controller.press(key)

    def mouse_release(self):
        if self._mouse_key:
            self.controller.release(self._mouse_key)
            self._mouse_key = None

    def stop_all(self):
        self._armed = False
        for timer in self._releases.values():
            self.ctx.root.after_cancel(timer)
        self._releases.clear()
        if self.controller:
            self.controller.stop()
        self.mouse_release()

    def on_activate(self):
        self._active = True

    def on_deactivate(self):
        self._active = False
        self.stop_all()

    def on_vehicle_change(self):
        self.stop_all()

    def on_tick(self, st):
        with self.controller.lock:
            keys = " / ".join(sorted(self.controller.pressed)).upper()
        if self.var_mode.get() == "keyboard":
            text = "键盘已启用：%s" % (keys or "等待按键，空格 / Esc 停止") if self._armed else "键盘未启用，请点击「点击启用键盘操控」。"
        else:
            text = "鼠标模式：按住方向按钮持续移动，松开或移出按钮即停；连续操控无时长限制。"
        self.status.config(text=text, fg=theme.OK if self._armed else theme.ACCENT)
        values = st.get("servo_targets") or {}
        names = self.ctx.cfg["servo"]["names"]
        self.targets.config(text="舵机目标：%s（非位置回读；云台首次以俯视 %d / 水平 1500 为基准）" %
                            ("  ".join("%s %s" % (names[k], v) for k, v in values.items() if k in names) or "--",
                             self.ctx.cfg["servo"]["actions"]["look_down"]))
        self.cam.on_tick(st)

    def on_close(self):
        self.stop_all()
        if self.controller:
            self.controller.close()
        self.cam.on_close()
