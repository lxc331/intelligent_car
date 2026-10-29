"""Navigation map selection and overlays built on the shared occupancy viewer."""
import math
import tkinter as tk

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
        self.start = None
        self.initial = None
        self.plan = ()
        self.network = None
        self.route = ()
        self._start_id = None
        self._selection = None
        self._selection_error = ""
        self._localized = None
        self._localization_detail = ""
        super().__init__(parent, model)
        pose_bar = tk.Frame(self, bg=theme.PANEL)
        pose_bar.pack(fill="x", before=self.canvas)
        self.pose_label = tk.Label(pose_bar, text="当前位置：未定位", anchor="w", justify="left",
                                   bg=theme.PANEL, fg=theme.WARN, font=theme.FONT_BOLD)
        self.pose_label.pack(side="left", fill="x", expand=True, padx=6, pady=4)
        self.center_button = tk.Button(pose_bar, text="定位小车", command=self.center_vehicle,
            bg=theme.PANEL_2, fg=theme.ACCENT, relief="flat", font=theme.FONT_SMALL)
        self.center_button.pack(side="right", padx=4)
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
        if mode not in ("goal", "start", "initial", "pan"):
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
        self.goal = self.initial = self.start = None
        self.plan = ()
        self.route = ()
        self._start_id = None
        self.set_localization(False, "请启动导航定位，并标定实际位置与朝向")
        self.reset_view()

    def set_localization(self, localized, detail=""):
        """Data only: an initial-pose click is not a localization confirmation."""
        self._localized = bool(localized)
        self._localization_detail = str(detail)

    def center_vehicle(self):
        snapshot = self.model.snapshot()
        pose = snapshot.get("pose")
        if self._localized is False or not pose or not snapshot.get("grid"):
            return False
        self._grid = snapshot["grid"]
        x, y = self._point(pose[0], pose[1], self._geometry())
        self._pan = (self._pan[0] + self.canvas.winfo_width() / 2 - x,
                     self._pan[1] + self.canvas.winfo_height() / 2 - y)
        self._render_key = None
        return True

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

    def set_network(self, network):
        """Overlay a road network loaded beside the map; ``None`` clears it."""
        self.network = network
        self.route = ()
        self.start = None
        self._start_id = None

    def set_start(self, pose, node_id=None):
        self.start = self._pose(pose)
        self._start_id = None if node_id is None else int(node_id)

    def set_route(self, ids):
        self.route = tuple(int(node_id) for node_id in ids)

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

    def _draw_network(self, geometry):
        """Road graph under the markers; the selected route is drawn brighter."""
        network = self.network
        points = {node.id: self._point(node.x, node.y, geometry) for node in network}
        route = set(zip(self.route, self.route[1:]))
        for source, target, _metres in network.edges():
            if (source, target) in route or source not in points or target not in points:
                continue
            self.canvas.create_line(*points[source], *points[target],
                                    fill="#3d6b73", width=1, tags="nav_network")
        for source, target in route:
            if source in points and target in points:
                self.canvas.create_line(*points[source], *points[target],
                                        fill=theme.ACCENT, width=3, tags="nav_network")
        on_route = set(self.route)
        for node in network:
            x, y = points[node.id]
            color = "#5ad1a5" if node.id == self._start_id else (
                theme.ACCENT if node.id in on_route else "#8aa0a6")
            self.canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill=color, outline="",
                                    tags="nav_network")
            self.canvas.create_text(x + 5, y + 5, text=str(node.id), anchor="nw",
                                    fill="#a9c3c8", font=theme.FONT_SMALL, tags="nav_network")

    def _draw(self, snapshot):
        visible = snapshot
        if self._localized is False:
            visible = dict(snapshot, pose=None, trajectory=())
        super()._draw(visible)
        self.canvas.delete("nav_path", "nav_goal", "nav_initial", "nav_select",
                           "nav_network", "nav_start")
        if not self._grid:
            return
        geometry = self._geometry()
        if self.network is not None:
            self._draw_network(geometry)
        if len(self.plan) > 1:
            points = [v for point in self.plan for v in self._point(*point, geometry)]
            self.canvas.create_line(*points, fill="#f0a52b", width=3, tags="nav_path")
            if self.canvas.find_withtag("vehicle"):
                self.canvas.tag_lower("nav_path", "vehicle")
        self._marker(self.goal, "#ed6670", "目标", "nav_goal", geometry)
        self._marker(self.initial, "#a98bff", "初始位姿", "nav_initial", geometry)
        if self.start is not None:
            self._marker(self.start, "#5ad1a5", "起点", "nav_start", geometry)
        pose = visible.get("pose")
        if pose is not None:
            self.canvas.delete("vehicle")
            x, y = self._point(pose[0], pose[1], geometry)
            color = theme.WARN if snapshot.get("pose_stale") else "#00cfe9"
            angle = pose[2] - self._grid["origin"][2]
            self.canvas.create_oval(x-13, y-13, x+13, y+13, outline=color, width=3, tags="vehicle")
            self.canvas.create_oval(x-6, y-6, x+6, y+6, fill=color, outline="#ffffff", tags="vehicle")
            self.canvas.create_line(x, y, x+29*math.cos(angle), y-29*math.sin(angle),
                                    arrow="last", fill=color, width=4, tags="vehicle")
            label = "上次位置（已过期）" if snapshot.get("pose_stale") else "当前小车"
            text = self.canvas.create_text(x+19, y-17, anchor="sw", fill="#ffffff", font=theme.FONT_SMALL,
                text="%s\nx=%.2f  y=%.2f m  %.0f°" % (label, pose[0], pose[1], math.degrees(pose[2])),
                tags=("vehicle", "nav_vehicle_text"))
            bounds = self.canvas.bbox(text)
            if bounds:
                # Keep the coordinate callout inside the viewport at map edges.
                dx = max(0, 4-bounds[0]) - max(0, bounds[2]-self.canvas.winfo_width()+4)
                dy = max(0, 4-bounds[1]) - max(0, bounds[3]-self.canvas.winfo_height()+4)
                self.canvas.move(text, dx, dy)
                left, top, right, bottom = self.canvas.bbox(text)
                background = self.canvas.create_rectangle(left-3, top-2, right+3, bottom+2,
                    fill="#172128", outline=color, tags="vehicle")
                self.canvas.tag_lower(background, text)
        if self._selection and self._selection[0] == self._grid["revision"]:
            _, x, y, end_x, end_y = self._selection
            self.canvas.create_line(x, y, end_x, end_y, fill=theme.ACCENT, width=2,
                                    arrow="last", tags="nav_select")

    def on_tick(self, state=None):
        super().on_tick(state)
        snapshot = self._last_state
        pose = snapshot.get("pose") if self._localized is not False else None
        self.center_button.config(state="normal" if pose is not None else "disabled")
        if pose is not None:
            prefix = "上次位置（定位过期）" if snapshot.get("pose_stale") else "当前位置"
            self.pose_label.config(text="%s：x=%.2f m  y=%.2f m  朝向=%.0f°" %
                (prefix, pose[0], pose[1], math.degrees(pose[2])),
                fg=theme.WARN if snapshot.get("pose_stale") else theme.ACCENT)
        else:
            detail = self._localization_detail or "请启动导航定位，并标定实际位置与朝向"
            self.pose_label.config(text="当前位置：未定位；"+detail, fg=theme.WARN,
                                   wraplength=max(200, self.canvas.winfo_width()-100))
            if self._localized is False:
                self.info.config(text="尚无已确认的地图坐标；紫色初始位姿是手动标定点。", fg=theme.WARN)
        if self._selection_error:
            self.info.configure(text=self._selection_error, fg=theme.WARN)
