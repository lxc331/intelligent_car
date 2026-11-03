"""Standalone window for hand-annotating a sandbox road network.

    py -3 -m carapp.road_editor [path/to/sandbox.json]

Loads the project's localization map and keep-out mask, lets you place nodes and
connect edges with the mouse, validates every edge against the project's own
mask, and writes the result back into ``road_network.yaml`` inside the project.
The main App is not required; the navigation page can open this window too.
"""
import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox

from carapp.config import APP_DIR, load_config, sandbox_footprint
from carapp.core.map_data import MapModel
from carapp.core.network_editor import NetworkEditor
from carapp.core.sandbox_project import load_project
from carapp.ui import theme
from carapp.ui.navigation_map import NavigationMapPanel

TICK_MS = 120
HINT = ("左键点空处＝新增节点；点一个节点＝选中（黄圈），再点另一个＝连边，点自己＝取消。"
        "右键拖动平移，滚轮缩放。节点要离障碍 ≥0.10 m（点在路中间）、彼此 ≥8 cm、"
        "单段 ≤2.0 m；连线的整段走廊净宽必须 ≥0.20 m、且 0.30×0.18 m 车体（可配置）扫得过去，否则拒绝。")


class RoadEditorWindow(tk.Toplevel):
    """One sandbox project's graph, editable and saveable."""

    def __init__(self, master, on_saved=None):
        super().__init__(master)
        self.title("沙盘路网标注")
        self.geometry("1120x780")
        self.configure(bg=theme.BG)
        self.on_saved = on_saved
        self.project = None
        self.editor = None
        self.selected = None
        self.model = MapModel()
        bar = tk.Frame(self, bg=theme.PANEL)
        bar.pack(fill="x")
        self.path_label = tk.Label(bar, text="未打开工程", bg=theme.PANEL, fg=theme.TEXT_DIM,
                                   anchor="w", font=theme.FONT_SMALL)
        self.path_label.pack(side="left", fill="x", expand=True, padx=6)
        self._button(bar, "打开工程…", self.ask_open)
        self._button(bar, "清空网络", self.clear_network, theme.WARN)
        self._button(bar, "保存路网", self.save, theme.OK)
        self._button(bar, "删除选中节点", self.delete_node)
        self._button(bar, "撤销上一条边", self.undo_edge)
        self._button(bar, "取消选中", self.clear_selection)
        self.info = tk.Label(self, text=HINT, bg=theme.PANEL, fg=theme.WARN, anchor="w",
                             justify="left", wraplength=1100, font=theme.FONT_SMALL)
        self.info.pack(fill="x")
        self.map_panel = NavigationMapPanel(self, self.model, on_pose=self._click)
        self.map_panel.pack(fill="both", expand=True)
        self.map_panel.set_mode("network")
        self.after(TICK_MS, self._tick)

    # -- helpers -----------------------------------------------------------------
    @staticmethod
    def _button(parent, text, command, color=None):
        button = tk.Button(parent, text=text, command=command, bg=theme.PANEL_2,
                           fg=color or theme.TEXT, relief="flat", activebackground=theme.BORDER,
                           font=theme.FONT_SMALL, padx=7, pady=4)
        button.pack(side="left", padx=3, pady=2)
        return button

    def _status(self, text, error=False):
        self.info.config(text=str(text), fg=theme.ERR if error else theme.TEXT)

    def _tick(self):
        if not self.winfo_exists():
            return
        self.map_panel.on_tick()
        self.after(TICK_MS, self._tick)

    # -- loading -----------------------------------------------------------------
    def ask_open(self):
        path = filedialog.askopenfilename(parent=self, title="选择沙盘工程 sandbox.json",
            initialdir=os.path.join(APP_DIR, "savedMap"), filetypes=[("沙盘工程", "sandbox.json")])
        if path:
            self.load(path)

    def load(self, target):
        try:
            project = target if hasattr(target, "network") else load_project(target)
            self.model.reset()
            self.model.accept_map(project.localization.grid_message, 0.)
            editor = NetworkEditor(project, footprint=sandbox_footprint(load_config()))
        except Exception as exc:  # noqa: BLE001 - the window reports instead of crashing
            self._status("打开工程失败：" + str(exc), True)
            return False
        self.project, self.editor, self.selected = project, editor, None
        self.map_panel.set_model(self.model)
        self.map_panel.set_mode("network")
        self.map_panel.set_network(project.network)
        self.map_panel.set_route(())
        self.map_panel.set_plan([])
        self.map_panel.set_selection(None)
        self.path_label.config(text=str(project.path))
        self._status("已载入 %s：%d 节点 / %d 边。%s" % (
            project.path.parent.name, len(editor.nodes), len(editor.edges), HINT))
        return True

    # -- editing -----------------------------------------------------------------
    def _refresh(self):
        self.map_panel.set_network(self.editor.network())
        self.map_panel.set_selection(self.selected)
        self.map_panel.set_route(())
        self.map_panel.set_plan([])

    def _click(self, _mode, x, y, _yaw):
        if self.editor is None:
            return self._status("请先点「打开工程…」选择 sandbox.json。", True)
        node = self.editor.node_near(x, y)
        if node is None:
            node, error = self.editor.add_node(x, y)
            if error:
                return self._status("新增节点失败：" + error, True)
            self.selected = node
            self._refresh()
            return self._status("新增节点 %d（%.2f, %.2f）；再点另一个节点就连成一段。"
                                % (node, x, y))
        if self.selected is None:
            self.selected = node
            self._refresh()
            return self._status("选中节点 %d；再点另一个节点连线。" % node)
        if self.selected == node:
            self.selected = None
            self._refresh()
            return self._status("已取消选中。")
        ok, message = self.editor.connect(self.selected, node)
        if not ok:
            self._refresh()
            return self._status("连线失败：" + message, True)
        self.selected = node
        self._refresh()
        return self._status(message)

    def save(self):
        if self.editor is None:
            return self._status("还没有打开工程。", True)
        try:
            project = self.editor.save()
        except (OSError, ValueError) as exc:
            return self._status("保存失败：" + str(exc), True)
        self.project = project
        self.editor = NetworkEditor(project, footprint=sandbox_footprint(load_config()))
        self.selected = None
        self.map_panel.set_network(project.network)
        self.map_panel.set_selection(None)
        if self.on_saved:
            self.on_saved(project)
        return self._status("已保存到 %s：%d 节点 / %d 条有向边，工程校验通过。" % (
            project.path.parent.name, len(project.network),
            len(project.metadata.get("edge_widths") or {})))

    def clear_network(self):
        if self.editor is None:
            return self._status("请先点「打开工程…」选择 sandbox.json。", True)
        if not messagebox.askyesno("清空网络", "把当前编辑里的所有节点和连线清空，从空白开始画？\n"
                                              "（磁盘上的工程要等你点「保存路网」才会变，"
                                              "清空后也可以重新打开工程恢复。）", parent=self):
            return
        message = self.editor.clear()
        self.selected = None
        self._refresh()
        self.map_panel.set_selection(None)
        return self._status(message)

    def delete_node(self):
        if self.editor is None or self.selected is None:
            return self._status("先点一个节点把它选中，再删除。", True)
        node = self.selected
        self.editor.remove_node(node)
        self.selected = None
        self._refresh()
        return self._status("已删除节点 %d 和它相连的边；改动还没保存。" % node)

    def undo_edge(self):
        if self.editor is None or not self.editor.edges:
            return self._status("还没有连线可撤销。", True)
        edge = sorted(self.editor.edges)[-1]
        self.editor.remove_edge(*edge)
        self._refresh()
        return self._status("已撤销连线 %d–%d；改动还没保存。" % edge)

    def clear_selection(self):
        if self.editor is None:
            return False
        self.selected = None
        self._refresh()
        return self._status("已取消选中。")


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    root = tk.Tk()
    root.withdraw()
    root.title("沙盘路网标注")
    window = RoadEditorWindow(root)
    if argv:
        window.load(argv[0])

    def close():
        window.destroy()
        root.destroy()

    window.protocol("WM_DELETE_WINDOW", close)
    window.mainloop()


if __name__ == "__main__":
    main()
