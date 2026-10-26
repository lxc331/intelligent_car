"""Tk occupancy map viewer; all image and canvas work stays on the UI thread."""
import math
import tkinter as tk

try:
    from PIL import Image, ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

from carapp.core.map_data import world_to_grid
from carapp.ui import theme


UNKNOWN = (112, 120, 126)
FREE = (232, 235, 237)
OCCUPIED = (34, 39, 42)
OUTSIDE = (21, 25, 28)
PALETTE = [*UNKNOWN, *FREE, *OCCUPIED, *OUTSIDE] + list(UNKNOWN) * 252


def cell_colour(value):
    """Three explicit classes: unknown, free (<65%), occupied (>=65%)."""
    return UNKNOWN if value < 0 else OCCUPIED if value >= 65 else FREE


class MapPanel(tk.Frame):
    def __init__(self, parent, model):
        super().__init__(parent, bg=theme.BG)
        self.model = model
        self._grid = None
        self._source = None
        self._source_revision = None
        self._photo = None
        self._render_key = None
        self._zoom = 1.0
        self._pan = (0., 0.)
        self._drag = None
        self._last_state = None
        self._build()

    def _build(self):
        bar = tk.Frame(self, bg=theme.PANEL)
        bar.pack(fill="x", pady=(0, 4))
        tk.Label(bar, text="实时二维地图", bg=theme.PANEL, fg=theme.ACCENT,
                 font=theme.FONT_BOLD).pack(side="left", padx=(8, 4))
        for label, command in (("适配", self.reset_view),
                               ("＋", lambda: self._change_zoom(1.25)),
                               ("－", lambda: self._change_zoom(.8))):
            tk.Button(bar, text=label, command=command, bg=theme.PANEL_2, fg=theme.TEXT,
                      relief="flat", font=theme.FONT_SMALL, padx=6, pady=3,
                      activebackground=theme.BORDER).pack(side="left", padx=2, pady=4)
        self.canvas = tk.Canvas(self, bg="#15191c", highlightthickness=0, width=240, height=150)
        self.canvas.pack(fill="both", expand=True)
        self.info = tk.Label(self, text="等待 /map 地图数据…", bg=theme.PANEL,
                             fg=theme.TEXT_DIM, font=theme.FONT_SMALL, anchor="w", justify="left")
        self.info.pack(fill="x", pady=(4, 0), padx=4)
        tk.Label(self, text="浅色：空闲　深色：占用　灰色：未知　滚轮缩放 / 拖动平移",
                 bg=theme.BG, fg=theme.TEXT_DIM, font=theme.FONT_SMALL,
                 anchor="w").pack(fill="x", padx=4, pady=(0, 2))
        self.canvas.bind("<Configure>", self._resize)
        self.canvas.bind("<MouseWheel>", self._wheel)
        self.canvas.bind("<Button-4>", lambda event: self._wheel(event, 1.25))
        self.canvas.bind("<Button-5>", lambda event: self._wheel(event, .8))
        self.canvas.bind("<ButtonPress-1>", self._drag_start)
        self.canvas.bind("<B1-Motion>", self._drag_move)
        self.canvas.bind("<ButtonRelease-1>", lambda event: setattr(self, "_drag", None))
        self.canvas.create_text(120, 75, text="等待 /map 地图数据…", fill=theme.TEXT_DIM,
                                font=theme.FONT, tags="waiting")

    def _resize(self, event):
        self.info.configure(wraplength=max(150, event.width - 12))
        self.canvas.coords("waiting", event.width / 2, event.height / 2)
        self._render_key = None

    def reset_view(self):
        self._zoom = 1.0
        self._pan = (0., 0.)
        self._render_key = None
        if self._last_state is not None:
            self._draw(self._last_state)

    def _geometry(self):
        if not self._grid:
            return 1., 0., 0.
        width, height = max(2, self.canvas.winfo_width()), max(2, self.canvas.winfo_height())
        scale = min(max(1, width - 24) / self._grid["width"],
                    max(1, height - 24) / self._grid["height"]) * self._zoom
        return (scale, (width - self._grid["width"] * scale) / 2 + self._pan[0],
                (height - self._grid["height"] * scale) / 2 + self._pan[1])

    def _change_zoom(self, factor, point=None):
        old = self._zoom
        new = max(.25, min(32., old * factor))
        factor = new / old
        if point:
            cx, cy = self.canvas.winfo_width() / 2, self.canvas.winfo_height() / 2
            px, py = self._pan
            self._pan = (point[0] - cx - factor * (point[0] - cx - px),
                         point[1] - cy - factor * (point[1] - cy - py))
        self._zoom = new
        self._render_key = None
        if self._last_state is not None:
            self._draw(self._last_state)

    def _wheel(self, event, factor=None):
        factor = factor or (1.25 if event.delta > 0 else .8)
        self._change_zoom(factor, (event.x, event.y))
        return "break"

    def _drag_start(self, event):
        self._drag = event.x, event.y, self._pan

    def _drag_move(self, event):
        if self._drag:
            x, y, pan = self._drag
            self._pan = pan[0] + event.x - x, pan[1] + event.y - y
            self._render_key = None
            if self._last_state is not None:
                self._draw(self._last_state)

    def _prepare_source(self, grid):
        if grid["revision"] == self._source_revision:
            return
        self._source_revision = grid["revision"]
        if HAS_PIL:
            # The ingestion worker has already classified and reversed the rows.
            indexed = Image.frombytes("P", (grid["width"], grid["height"]),
                                      grid["pixels"])
            indexed.putpalette(PALETTE)
            self._source = indexed
        else:
            self._source = None

    def _render_raster(self, scale, left, top):
        width, height = max(2, self.canvas.winfo_width()), max(2, self.canvas.winfo_height())
        if HAS_PIL:
            # Render a viewport-sized image, even at high zoom, to bound memory.
            raster = self._source.transform((width, height), Image.Transform.AFFINE,
                                           (1 / scale, 0, -left / scale, 0, 1 / scale, -top / scale),
                                           resample=Image.Resampling.NEAREST, fillcolor=3)
            self._photo = ImageTk.PhotoImage(raster.convert("RGB"), master=self.canvas)
        else:
            grid = self._grid
            # Bound Python work independently of map/window size. Native Tk zoom
            # expands this sampled viewport; marker/trajectory coordinates remain
            # at full canvas precision. Input callbacks need no 4M-cell traversal.
            step = max(1, math.ceil(math.sqrt(width * height / 100_000)))
            sample_width, sample_height = math.ceil(width / step), math.ceil(height / step)
            columns = [math.floor((x * step - left) / scale) for x in range(sample_width)]
            palette = tuple(bytes(colour) for colour in (UNKNOWN, FREE, OCCUPIED))
            outside = bytes(OUTSIDE)
            rows = []
            previous_row = None
            previous_pixels = None
            for py in range(sample_height):
                row = math.floor((py * step - top) / scale)
                if row == previous_row:
                    rows.append(previous_pixels)
                    continue
                if 0 <= row < grid["height"]:
                    offset = row * grid["width"]
                    pixels = b"".join(palette[grid["pixels"][offset + col]]
                                      if 0 <= col < grid["width"] else outside for col in columns)
                else:
                    pixels = outside * sample_width
                rows.append(pixels)
                previous_row, previous_pixels = row, pixels
            ppm = ("P6\n%d %d\n255\n" % (sample_width, sample_height)).encode("ascii") + b"".join(rows)
            self._photo = tk.PhotoImage(master=self.canvas, data=ppm, format="PPM")
            if step > 1:
                self._photo = self._photo.zoom(step, step)
        self.canvas.delete("raster")
        self.canvas.create_image(0, 0, anchor="nw", image=self._photo, tags="raster")
        self.canvas.tag_lower("raster")

    def _point(self, x, y, geometry):
        gx, gy = world_to_grid(self._grid, x, y)
        scale, left, top = geometry
        return left + gx * scale, top + (self._grid["height"] - gy) * scale

    def _draw(self, snapshot):
        grid = snapshot["grid"]
        if not grid:
            self._grid = None
            self.canvas.delete("raster", "trail", "vehicle")
            self._photo = self._source = None
            self._source_revision = None
            self._render_key = None
            self.canvas.itemconfigure("waiting", state="normal")
            return
        self.canvas.itemconfigure("waiting", state="hidden")
        self._grid = grid
        geometry = self._geometry()
        key = (grid["revision"], self.canvas.winfo_width(), self.canvas.winfo_height(), geometry)
        if key != self._render_key:
            self._prepare_source(grid)
            self._render_raster(*geometry)
            self._render_key = key
        self.canvas.delete("trail", "vehicle")
        trajectory = snapshot.get("trajectory", ())
        if len(trajectory) > 1:
            points = [coordinate for point in trajectory for coordinate in self._point(*point, geometry)]
            self.canvas.create_line(*points, fill="#0aa4ce", width=2, tags="trail")
        pose = snapshot.get("pose")
        if pose is not None:
            x, y = self._point(pose[0], pose[1], geometry)
            angle = pose[2] - grid["origin"][2]
            tip = x + 14 * math.cos(angle), y - 14 * math.sin(angle)
            colour = theme.WARN if snapshot.get("pose_stale") else "#02b6e7"
            self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill=colour,
                                    outline="#ffffff", tags="vehicle")
            self.canvas.create_line(x, y, *tip, arrow="last", fill=colour, width=3, tags="vehicle")

    def on_tick(self, state=None):
        """Call on the Tk main thread; state is accepted for shared panel parity."""
        snapshot = self.model.snapshot()
        self._last_state = snapshot
        self._draw(snapshot)
        grid = snapshot["grid"]
        if grid:
            age = snapshot.get("map_age")
            text = "%d × %d 格 | %.3f m/格" % (grid["width"], grid["height"], grid["resolution"])
            if age is not None:
                text += " | 地图 %.1f 秒前" % age
            pose = snapshot.get("pose")
            if pose:
                text += "\n车位：%.2f, %.2f m / %.0f°" % (pose[0], pose[1], math.degrees(pose[2]))
        else:
            text = "等待 /map 地图数据…"
        error = snapshot.get("error") or snapshot.get("pose_error")
        if error:
            text += "\n" + error
        self.info.configure(text=text, fg=theme.WARN if error else theme.TEXT_DIM)

    def on_close(self):
        self._photo = self._source = None
