"""Focus checks that also understand widgets created internally by Tcl/Tk."""

import tkinter as tk


def widget_has_focus(widget):
    """Read current focus without converting native Tcl widgets to Python objects."""
    try:
        return str(widget.tk.call("focus")) == str(widget)
    except tk.TclError:
        return False


def window_has_focus(window):
    """Include a window's combobox popup, but exclude other top-level windows."""
    try:
        # focus_get() converts the path to a Python widget. Native ttk popdown
        # children have no Python wrapper, so that conversion raises KeyError.
        path = window.tk.call("focus")
        if not path or path == "none":
            return False
        top = window.tk.call("winfo", "toplevel", path)
        if window.tk.call("winfo", "class", top) == "ComboboxPopdown":
            owner = window.tk.call("winfo", "parent", top)
            top = window.tk.call("winfo", "toplevel", owner)
        return str(top) == str(window)
    except tk.TclError:
        # A window can disappear between querying focus and looking it up.
        return False
