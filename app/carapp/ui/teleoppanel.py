"""Compact base controls with background permission checks and visible rendering.

The page calls on_background_tick every 100 ms to refresh motion permission;
on_tick only updates the controls' text while that page is selected.
"""
import math
import threading
import tkinter as tk

from carapp.core.teleop import BASE_KEYS, TeleopController
from carapp.core.key_state import key_is_down
from carapp.ui import theme
from carapp.ui.focus import widget_has_focus


class _MotionGate:
    """The worker consumes permission; Tk callbacks are evaluated by the UI only.

    Held commands have no elapsed-time limit. Connection and page permission
    remain required for every nonzero publication.
    """
    def __init__(self, ctx):
        self.ctx = ctx
        self.cfg = ctx.cfg
        self.state = ctx.state
        self.allowed = threading.Event()
        self.lock = threading.Lock()

    def log(self, message):
        self.ctx.log(message)

    def publish_twist(self, vx=0., wz=0., quiet=False, *, vy=0.):
        with self.lock:
            if any((vx, vy, wz)):
                if not self.allowed.is_set():
                    return False
            return self.ctx.publish_twist(vx, wz, quiet=quiet, vy=vy)

    def stop_base(self):
        self.ctx.stop_base()


class TeleopPanel(tk.Frame):
    """Mouse/key base control, active only after the enclosing page enables it.

    ``ready`` runs on the Tk thread and must query the enclosing feature's public
    motion ownership and connection state. ``on_background_tick`` checks these
    every 100 ms; ``on_tick`` only renders the currently selected page.
    No key bindings are installed on the root window or other input fields.
    """
    def __init__(self, parent, ctx, ready=None):
        super().__init__(parent, bg=theme.PANEL)
        self.ctx = ctx
        self.ready = ready or (lambda: True)
        self._active = False
        self._armed = False
        self._closed = False
        self._mouse_key = None
        self._releases = {}
        self._unfocused_keys = set()
        self._gate = _MotionGate(ctx)
        self.controller = TeleopController(self._gate)
        self.mode = tk.StringVar(value="mouse")
        self.speed = tk.StringVar(value="0.04")
        self._build()
        self.controller.set_speed(.04, .25)

    def _button(self, parent, label, command=None, color=None):
        return tk.Button(parent, text=label, command=command, bg=theme.PANEL_2,
                         fg=color or theme.TEXT, activebackground=theme.BORDER,
                         activeforeground=theme.TEXT, relief="flat", font=theme.FONT_SMALL,
                         padx=8, pady=4)

    def _build(self):
        bar = tk.Frame(self, bg=theme.PANEL)
        bar.pack(fill="x", padx=6, pady=(4, 2))
        tk.Label(bar, text="建图遥控", bg=theme.PANEL, fg=theme.TEXT,
                 font=theme.FONT_BOLD).pack(side="left", padx=(0, 8))
        for text, value in (("鼠标", "mouse"), ("键盘", "keyboard")):
            tk.Radiobutton(bar, text=text, variable=self.mode, value=value,
                           command=self.change_mode, bg=theme.PANEL, fg=theme.TEXT,
                           selectcolor=theme.BG, activebackground=theme.PANEL,
                           activeforeground=theme.ACCENT, font=theme.FONT_SMALL).pack(side="left")
        tk.Label(bar, text="速度（m/s）", bg=theme.PANEL, fg=theme.TEXT_DIM,
                 font=theme.FONT_SMALL).pack(side="left", padx=(12, 3))
        speed = tk.Spinbox(bar, textvariable=self.speed, from_=.02, to=.10,
                           increment=.01, format="%.2f", width=5,
                           command=self.apply_speed, bg=theme.PANEL_2, fg=theme.TEXT,
                           insertbackground=theme.TEXT, buttonbackground=theme.PANEL_2,
                           relief="flat", font=theme.FONT_MONO)
        speed.pack(side="left", padx=4)
        speed.bind("<Return>", lambda _event: self.apply_speed())
        speed.bind("<FocusOut>", lambda _event: self.apply_speed())
        self._button(bar, "■ 停止 / 退出键盘", self.stop, theme.ERR).pack(side="right")

        self.mouse_panel = tk.Frame(self, bg=theme.PANEL)
        self.mouse_panel.pack(fill="x", padx=6, pady=2)
        self.mouse_buttons = {}
        for key, label in (("q", "Q 平移左"), ("w", "W 前进"), ("e", "E 平移右"),
                           ("a", "A 左转"), ("s", "S 后退"), ("d", "D 右转")):
            button = self._button(self.mouse_panel, label)
            button.pack(side="left", expand=True, fill="x", padx=2)
            button.bind("<ButtonPress-1>", lambda _event, k=key: self.mouse_press(k))
            button.bind("<ButtonRelease-1>", self.mouse_release)
            button.bind("<Leave>", self.mouse_release)
            self.mouse_buttons[key] = button

        self.keyboard_panel = tk.Frame(self, bg=theme.PANEL)
        self._button(self.keyboard_panel, "点击启用键盘操控", self.enable_keyboard,
                     theme.ACCENT).pack(side="left", padx=(0, 6))
        self.key_surface = tk.Frame(self.keyboard_panel, bg=theme.PANEL_2, takefocus=True,
                                    highlightthickness=2, highlightbackground=theme.BORDER,
                                    highlightcolor=theme.ACCENT)
        self.key_surface.pack(side="left", fill="x", expand=True)
        tk.Label(self.key_surface, text="W/S 前后  A/D 转向  Q/E 平移  空格/Esc 停止",
                 bg=theme.PANEL_2, fg=theme.TEXT, font=theme.FONT_SMALL,
                 pady=5).pack(fill="x")
        self.key_surface.bind("<KeyPress>", self.key_press)
        self.key_surface.bind("<KeyRelease>", self.key_release)
        self.key_surface.bind("<FocusOut>", self.focus_lost)
        self.status = tk.Label(self, text="进入本页后可遥控；按住移动，松开停止。",
                               anchor="w", bg=theme.PANEL, fg=theme.TEXT_DIM,
                               font=theme.FONT_SMALL)
        self.status.pack(fill="x", padx=8, pady=(2, 4))

    @property
    def motion_active(self):
        with self.controller.lock:
            return bool(self.controller.pressed.intersection(BASE_KEYS)
                        or any(self.controller.last_twist))

    def _refresh_permission(self):
        permitted = self._active and not self._closed and bool(self.ctx.state.get("ws_connected"))
        if permitted:
            try:
                permitted = bool(self.ready())
            except Exception:
                permitted = False
        if permitted:
            self._gate.allowed.set()
        else:
            self._gate.allowed.clear()
        return permitted

    def apply_speed(self):
        self.stop()
        try:
            value = float(self.speed.get())
            if not math.isfinite(value) or not .02 <= value <= .10:
                raise ValueError("速度需在 0.02–0.10 m/s")
        except (ValueError, TypeError, OverflowError, tk.TclError):
            self.ctx.log("建图遥控速度无效，请输入 0.02–0.10 m/s")
            return False
        self.controller.set_speed(value, .25)
        return True

    def change_mode(self):
        self.stop()
        self.mouse_panel.pack_forget()
        self.keyboard_panel.pack_forget()
        selected = self.keyboard_panel if self.mode.get() == "keyboard" else self.mouse_panel
        selected.pack(fill="x", padx=6, pady=2, before=self.status)

    def focus_lost(self, _event=None):
        if _event is not None and widget_has_focus(self.key_surface):
            return
        # Keep an existing hold alive; Tk may no longer receive its KeyRelease.
        # Poll only keys already accepted here, never start motion from global input.
        with self.controller.lock:
            self._unfocused_keys.update(self.controller.pressed.intersection(BASE_KEYS))

    def set_active(self, active):
        self._active = bool(active)
        if not active:
            if self._armed or self.motion_active:
                self.stop()
            else:
                self._gate.allowed.clear()

    def enable_keyboard(self):
        self.stop()
        if self.mode.get() != "keyboard":
            return False
        if not self._refresh_permission():
            self._not_ready()
            return False
        # A held key blocked by an earlier stop must be released before reuse.
        self._armed = True
        self.key_surface.focus_set()
        return True

    def key_press(self, event):
        key = event.keysym.lower()
        if key in ("space", "escape"):
            self.stop()
            return "break"
        if event.state & (0x0004 | 0x0008 | 0x20000):
            self.stop()
            return "break"
        if (key not in BASE_KEYS or not self._armed or self.mode.get() != "keyboard"
                or self.ctx.root.focus_get() is not self.key_surface):
            return
        pending = self._releases.pop(key, None)
        if pending:
            self.ctx.root.after_cancel(pending)
        if self._refresh_permission():
            self.controller.press(key)
        else:
            self.stop()
        return "break"

    def key_release(self, event):
        key = event.keysym.lower()
        if key in BASE_KEYS:
            old = self._releases.pop(key, None)
            if old:
                self.ctx.root.after_cancel(old)
            self._releases[key] = self.ctx.root.after(20, lambda: self._release_key(key))
            return "break"

    def _release_key(self, key):
        self._releases.pop(key, None)
        self._unfocused_keys.discard(key)
        self.controller.release(key)

    def mouse_press(self, key):
        if key not in BASE_KEYS or self.mode.get() != "mouse":
            return False
        self.mouse_release()
        if not self._refresh_permission():
            self._not_ready()
            return False
        self._mouse_key = key
        # A new mouse press is a distinct action even if an earlier keyboard
        # stop left this direction blocked while the keyboard lacked focus.
        self.controller.release(key)
        return self.controller.press(key)

    def _not_ready(self):
        self.ctx.log("建图遥控未就绪：请检查建图连接、地图/定位、雷达与画面，或等待其他动作结束")

    def mouse_release(self, _event=None):
        if self._mouse_key:
            self.controller.release(self._mouse_key)
            self._mouse_key = None

    def stop(self):
        self._armed = False
        self._gate.allowed.clear()
        for handle in self._releases.values():
            try:
                self.ctx.root.after_cancel(handle)
            except tk.TclError:
                pass
        self._releases.clear()
        self._unfocused_keys.clear()
        # Keep keyboard directions blocked until a real release. A stopped held
        # key must not resume on OS repeat after focus returns or re-arming.
        with self.controller.lock:
            self.controller.stop()
            if self._mouse_key:
                self.controller.release(self._mouse_key)
        self._mouse_key = None

    def on_background_tick(self, state):
        for key in tuple(self._unfocused_keys):
            if key_is_down(key) is False:
                pending = self._releases.pop(key, None)
                if pending:
                    self.ctx.root.after_cancel(pending)
                self._release_key(key)
        if not self._refresh_permission():
            if self.motion_active or self._armed:
                self.stop()

    def on_tick(self, state):
        with self.controller.lock:
            keys = " / ".join(sorted(self.controller.pressed)).upper()
        if self.mode.get() == "keyboard":
            text = ("键盘已启用：%s" % (keys or "等待按键，空格/Esc 停止")) if self._armed else "键盘未启用；点击启用后操作，失焦不停车，松键即停。"
        else:
            text = "鼠标：按住持续移动，松开或移出即停；无行驶时长限制。"
        self.status.config(text=text, fg=theme.OK if keys else theme.TEXT_DIM)

    def close(self):
        self._closed = True
        self._active = False
        self.stop()
        self.controller.close()
