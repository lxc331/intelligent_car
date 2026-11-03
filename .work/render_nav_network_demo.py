# -*- coding: utf-8 -*-
"""Render the navigation map with the road network overlay and grab a screenshot.

Offline only: no car, no ROS.  Used to eyeball the Phase 2 road-network layer
(nodes, edges, highlighted route, start marker, preview path).
"""
import sys
import time
import tkinter as tk
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "app"))

from carapp.core.map_data import MapModel  # noqa: E402
from carapp.core.navigation_map import load_navigation_map  # noqa: E402
from carapp.core.road_network import load_road_network  # noqa: E402
from carapp.ui.navigation_map import NavigationMapPanel  # noqa: E402

MAP = ROOT / "app" / "savedMap" / "sandbox_half" / "map.yaml"
OUT = ROOT / ".work" / "nav_network_demo.png"
START, GOAL = 0, 8


def main():
    bundle = load_navigation_map(MAP)
    network = load_road_network(MAP.with_name("road_network.yaml"))
    route = network.shortest_path(START, GOAL)
    points = network.dense_path(route)
    print("nodes=%d edges=%d route=%s %.2f m" %
          (len(network), len(network.edges()), route, network.path_length(route)))

    root = tk.Tk()
    root.title("road network preview")
    root.geometry("980x760+30+20")
    model = MapModel()
    model.accept_map(bundle.grid_message, time.monotonic())
    panel = NavigationMapPanel(root, model)
    panel.pack(fill="both", expand=True)
    panel.set_model(model)
    panel.set_network(network)
    start_node = network.node(START)
    panel.set_start((start_node.x, start_node.y, 0.), START)
    panel.set_route(route)
    panel.set_plan(points)
    goal_x, goal_y, goal_yaw = points[-1]
    panel.set_goal((goal_x, goal_y, goal_yaw))
    panel.set_localization(True)
    root.update()
    panel.on_tick()
    root.attributes("-topmost", True)
    root.lift()
    root.focus_force()
    root.update()
    time.sleep(1.0)
    root.update()

    from PIL import ImageGrab
    box = (root.winfo_rootx(), root.winfo_rooty(),
           root.winfo_rootx() + root.winfo_width(), root.winfo_rooty() + root.winfo_height())
    ImageGrab.grab(bbox=box).save(OUT)
    print("wrote", OUT)
    root.destroy()


if __name__ == "__main__":
    main()
