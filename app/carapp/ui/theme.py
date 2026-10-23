# -*- coding: utf-8 -*-
"""统一配色与字体（深色主题，和 IDE 一致）。"""

BG = "#1e1e1e"
PANEL = "#252526"
PANEL_2 = "#2d2d30"
BORDER = "#3c3c3c"
TEXT = "#e6e6e6"
TEXT_DIM = "#9d9d9d"
ACCENT = "#4fc3f7"
OK = "#66bb6a"
WARN = "#ffb74d"
ERR = "#ef5350"

FONT = ("Microsoft YaHei UI", 10)
FONT_SMALL = ("Microsoft YaHei UI", 9)
FONT_BOLD = ("Microsoft YaHei UI", 10, "bold")
FONT_TITLE = ("Microsoft YaHei UI", 11, "bold")
FONT_MONO = ("Consolas", 10)


def style_ttk(root):
    """给 ttk 控件套上深色皮肤。"""
    from tkinter import ttk

    st = ttk.Style(root)
    try:
        st.theme_use("clam")
    except Exception:
        pass
    st.configure(".", background=BG, foreground=TEXT, fieldbackground=PANEL_2)
    st.configure("TNotebook", background=BG, borderwidth=0)
    st.configure("TNotebook.Tab", background=PANEL, foreground=TEXT_DIM,
                 padding=(16, 7), borderwidth=0)
    st.map("TNotebook.Tab",
           background=[("selected", PANEL_2)],
           foreground=[("selected", ACCENT)])
    st.configure("TFrame", background=BG)
    st.configure("Panel.TFrame", background=PANEL)
    st.configure("TLabel", background=BG, foreground=TEXT)
    st.configure("Dim.TLabel", background=BG, foreground=TEXT_DIM)
    st.configure("TCombobox", fieldbackground=PANEL_2, background=PANEL_2,
                 foreground=TEXT, arrowcolor=TEXT)
    st.configure("Horizontal.TScale", background=BG, troughcolor=PANEL_2)
    st.configure("Treeview",background=PANEL_2,fieldbackground=PANEL_2,foreground=TEXT,
                 rowheight=25,borderwidth=0,font=FONT_SMALL)
    st.configure("Treeview.Heading",background=PANEL,foreground=TEXT_DIM,font=FONT_SMALL)
    st.map("Treeview",background=[("selected","#094771")],foreground=[("selected",TEXT)])
    return st
