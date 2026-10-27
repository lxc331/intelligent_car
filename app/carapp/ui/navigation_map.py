"""Navigation map selection and overlays built on the shared occupancy viewer."""
import math

from carapp.core.navigation_map import valid_target
from carapp.ui import theme
from carapp.ui.mappanel import MapPanel


class NavigationMapPanel(MapPanel):
    """Click a free cell; dragging sets heading. Right-drag always pans.

    ``on_pose(mode, x, y, yaw)`` runs on Tk's thread after a complete selection.
    Setters store display data only; drawing is deferred to visible on_tick.
    """
    def __init__(self, parent, model, on_pose=None):
        self.on_pose = on_pose
        self.mode = "goal"
        self.goal = None
        self.initial = None
        self.plan = ()
        self._selection = None
        self._selection_error = ""
        super().__init__(parent, model)
        self.canvas.bind("<ButtonPress-1>", self._select_start)
        self.canvas.bind("<B1-Motion>", self._select_move)
        self.canvas.bind("<ButtonRelease-1>", self._select_finish)
        for button in (2, 3):
            self.canvas.bind("<ButtonPress-%d>" % button, self._drag_start)
            self.canvas.bind("<B%d-Motion>" % button, self._drag_move)
            self.canvas.bind("<ButtonRelease-%d>" % button, self._pan_finish)
        for widget in self.winfo_children():
            if widget.winfo_class() == "Label" and "浅色" in widget.cget("text"):
                widget.configure(text="浅色：空闲　深色：占用　灰色：未知　左键选点/拖出朝向　右键平移　滚轮缩放")

    def set_mode(self, mode):
        if mode not in ("goal", "initial", "pan"):
            raise ValueError("地图操作模式无效")
        self.mode = mode
        self._selection = None
        self._selection_error = ""
        self.canvas.configure(cursor="fleur" if mode == "pan" else "crosshair")

    def set_model(self, model):
        """Switch a worker-prepared map even when its revision matches the old one."""
        self.model = model
        self._grid = self._source = self._photo = None
        self._source_revision = self._render_key = self._last_state = None
        self._selection = None
        self._selection_error = ""
        self.goal = self.initial = None
        self.plan = ()
        self.reset_view()

    @staticmethod
    def _pose(value):
        if value is None:
            return None
        if len(value) != 3 or not all(math.isfinite(v) for v in value):
            raise ValueError("地图标记需要有限的 x/y/yaw")
        return tuple(float(v) for v in value)

    def set_goal(self, pose):
        self.goal = self._pose(pose)

    def set_initial(self, pose):
        self.initial = self._pose(pose)

    def set_plan(self, points):
        if len(points) > 100_000:
            raise ValueError("路径点过多")
        self.plan = tuple((float(p[0]), float(p[1])) for p in points)
        if not all(math.isfinite(v) for p in self.plan for v in p):
            self.plan = ()
            raise ValueError("路径点需要有限坐标")

    def canvas_to_world(self, x, y):
        if not self._grid:
            return None
        scale, left, top = self._geometry()
        gx = (x - left) / scale
        gy = self._grid["height"] - (y - top) / scale
        ox, oy, yaw = self._grid["origin"]
        dx, dy = gx * self._grid["resolution"], gy * self._grid["resolution"]
        c, s = math.cos(yaw), math.sin(yaw)
        return ox + c * dx - s * dy, oy + s * dx + c * dy

    def _select_start(self, event):
        if self.mode == "pan":
            self._drag_start(event)
        elif self._grid:
            self._selection_error = ""
            self._selection = (self._grid["revision"], event.x, event.y, event.x, event.y)
        return "break"

    def _select_move(self, event):
        if self.mode == "pan":
            self._drag_move(event)
        elif self._selection:
            revision, x, y, _, _ = self._selection
            self._selection = revision, x, y, event.x, event.y
        return "break"

    def _pan_finish(self, _event):
        self._drag = None

    def _select_finish(self, event):
        if self.mode == "pan":
            self._pan_finish(event)
            return "break"
        selection, self._selection = self._selection, None
        if not selection or not self._grid or selection[0] != self._grid["revision"]:
            return "break"
        point = self.canvas_to_world(selection[1], selection[2])
        if point is None:
            return "break"
        if not valid_target(self._grid, *point):
            self._selection_error = "请选择地图内的已知空闲区域；障碍物和未知区域不能作为目标或初始位置。"
            return "break"
        finish = self.canvas_to_world(event.x, event.y)
        moved = math.hypot(event.x - selection[1], event.y - selection[2]) >= 4
        yaw = math.atan2(finish[1] - point[1], finish[0] - point[0]) if moved else 0.
        if self.on_pose:
            self.on_pose(self.mode, *point, yaw)
        return "break"

    def _marker(self, pose, color, label, tag, geometry):
        if pose is None:
            return
        x, y = self._point(pose[0], pose[1], geometry)
        angle = pose[2] - self._grid["origin"][2]
        self.canvas.create_oval(x - 6, y - 6, x + 6, y + 6, outline=color, width=2, tags=tag)
        self.canvas.create_line(x, y, x + 22 * math.cos(angle), y - 22 * math.sin(angle),
                                arrow="last", fill=color, width=2, tags=tag)
        self.canvas.create_text(x + 8, y - 12, text=label, anchor="w", fill=color, tags=tag)

    def _draw(self, snapshot):
        super()._draw(snapshot)
        self.canvas.delete("nav_path", "nav_goal", "nav_initial", "nav_select")
        if not self._grid:
            return
        geometry = self._geometry()
        if len(self.plan) > 1:
            points = [v for point in self.plan for v in self._point(*point, geometry)]
            self.canvas.create_line(*points, fill="#f0a52b", width=3, tags="nav_path")
            if self.canvas.find_withtag("vehicle"):
                self.canvas.tag_lower("nav_path", "vehicle")
        self._marker(self.goal, "#ed6670", "目标", "nav_goal", geometry)
        self._marker(self.initial, "#a98bff", "初始位姿", "nav_initial", geometry)
        if self._selection and self._selection[0] == self._grid["revision"]:
            _, x, y, end_x, end_y = self._selection
            self.canvas.create_line(x, y, end_x, end_y, fill=theme.ACCENT, width=2,
                                    arrow="last", tags="nav_select")

    def on_tick(self, state=None):
        super().on_tick(state)
        if self._selection_error:
            self.info.configure(text=self._selection_error, fg=theme.WARN)
