"""Reusable speed input with quick preset buttons.

Every page that drives the car shows the same control: a short label, an editable
value, and a row of one-click presets. The owner passes ``on_apply`` to react on
Enter, focus-out or a preset click; pages that only read the value when their own
"apply/start" button is pressed leave ``on_apply`` empty and read ``var``.
"""
import math
import tkinter as tk

from carapp.ui import theme


class SpeedPicker(tk.Frame):
    """Label + numeric input (clamped to a range) + preset buttons."""

    def __init__(self, parent, *, label="速度（m/s）", low=.02, high=.15, step=.01,
                 value=.06, presets=(), on_apply=None, width=6, label_bold=False,
                 bg=theme.PANEL):
        super().__init__(parent, bg=bg)
        self.low, self.high = float(low), float(high)
        self.on_apply = on_apply
        self._last = self._clamp_initial(value)
        tk.Label(self, text=label, bg=bg, fg=theme.TEXT,
                 font=theme.FONT_BOLD if label_bold else theme.FONT_SMALL).pack(
                     side="left", padx=(8, 3))
        self.var = tk.StringVar(value="%.2f" % self._last)
        self.spin = tk.Spinbox(self, textvariable=self.var, from_=self.low, to=self.high,
                               increment=step, format="%.2f", width=width,
                               command=self.apply, bg=theme.PANEL_2, fg=theme.TEXT,
                               insertbackground=theme.TEXT, buttonbackground=theme.PANEL_2,
                               relief="flat", font=theme.FONT_MONO)
        self.spin.pack(side="left", padx=(2, 4))
        self.spin.bind("<Return>", lambda _event: self.apply())
        self.spin.bind("<FocusOut>", lambda _event: self.apply())
        for preset in presets:
            tk.Button(self, text="%.2f" % float(preset),
                      command=lambda p=preset: self.set_value(p),
                      bg=theme.PANEL_2, fg=theme.TEXT, activebackground=theme.BORDER,
                      activeforeground=theme.ACCENT, relief="flat",
                      font=theme.FONT_SMALL, padx=4, pady=1).pack(side="left", padx=1)

    def _clamp(self, value):
        """Clamp to the allowed range; unparsable input keeps the last value."""
        try:
            number = float(value)
        except (TypeError, ValueError):
            return self._last
        if not math.isfinite(number):
            return self._last
        return min(self.high, max(self.low, number))

    def _clamp_initial(self, value):
        self._last = .0
        return self._clamp(value)

    def value(self):
        """Current speed as a float, clamped to the range."""
        return self._clamp(self.var.get())

    def apply(self):
        """Normalise the text and notify the owner; returns the applied value."""
        value = self.value()
        self._last = value
        self.var.set("%.2f" % value)
        if self.on_apply is not None:
            self.on_apply(value)
        return value

    def set_value(self, value):
        """Programmatic change used by preset buttons and tests."""
        self.var.set("%.2f" % self._clamp(value))
        return self.apply()
