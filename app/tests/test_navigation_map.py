"""Saved-map classification, coordinate transforms and navigation selection."""
import math
from pathlib import Path
import tempfile
import time
import tkinter as tk
import types
import unittest
from unittest.mock import patch

from carapp.core.map_data import MapModel
from carapp.core.navigation_map import load_navigation_map, valid_target
from carapp.core.road_network import parse_road_network
from carapp.ui.navigation_map import NavigationMapPanel


class NavigationMapTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def write_map(self, pixels=bytes([0, 205, 254, 254, 0, 254]), *, width=3,
                  height=2, metadata="", image="map.pgm"):
        (self.directory / image).write_bytes(("P5\n%d %d\n255\n" % (width, height)).encode() + pixels)
        path = self.directory / "map.yaml"
        path.write_text("image: %s\nresolution: 0.1\norigin: [1, 2, 0]\n"
                        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n%s" % (image, metadata),
                        encoding="utf-8")
        return path

    def test_pgm_y_flip_unknown_and_canonical_round_trip(self):
        bundle = load_navigation_map(self.write_map())
        self.assertEqual(bundle.grid_message["data"], (0, 100, 0, 100, -1, 0))
        self.assertEqual(bundle.source_path.name, "map.yaml")
        self.assertEqual(bundle.as_upload()["image_bytes"], bundle.image_bytes)
        (self.directory / "map.pgm").write_bytes(bundle.image_bytes)
        (self.directory / "map.yaml").write_text(bundle.yaml_text, encoding="utf-8")
        round_trip = load_navigation_map(self.directory / "map.yaml")
        self.assertEqual(round_trip.grid_message, bundle.grid_message)

    def test_negate_changes_occupied_and_free(self):
        path = self.write_map(bytes([0, 255]), width=2, height=1)
        path.write_text(path.read_text().replace("negate: 0", "negate: 1"))
        self.assertEqual(load_navigation_map(path).grid_message["data"], (0, 100))

    def test_rotated_origin_and_target_validation(self):
        path = self.write_map(bytes([254, 0, 205]), width=3, height=1)
        path.write_text(path.read_text().replace("[1, 2, 0]", "[1, 2, %s]" % (math.pi / 2)))
        bundle = load_navigation_map(path)
        self.assertTrue(valid_target(bundle, .95, 2.05))
        self.assertFalse(valid_target(bundle, .95, 2.15))
        self.assertFalse(valid_target(bundle, .95, 2.25))
        self.assertFalse(valid_target(bundle, .95, 2.35))
        self.assertFalse(valid_target(bundle, float("nan"), 2.05))
        with self.assertRaisesRegex(ValueError, "轴对齐"):
            bundle.as_upload()

    def test_full_turn_origin_can_upload_as_axis_aligned(self):
        path = self.write_map()
        path.write_text(path.read_text().replace("[1, 2, 0]", "[1, 2, %s]" % (2 * math.pi)))
        self.assertIn("image_bytes", load_navigation_map(path).as_upload())

    def test_png_uses_channel_mean_and_humble_trinary_alpha(self):
        from PIL import Image
        image = Image.new("RGBA", (2, 1))
        image.putdata([(0, 0, 0, 255), (255, 255, 255, 0)])
        path = self.write_map(width=2, height=1)
        image.save(self.directory / "map.png")
        path.write_text(path.read_text().replace("map.pgm", "map.png"))
        # Humble averages alpha with RGB in trinary: opaque black is occupied;
        # transparent white is unknown, rather than assuming white means free.
        self.assertEqual(load_navigation_map(path).grid_message["data"], (100, -1))

    def test_invalid_metadata_and_unsafe_tags_rejected(self):
        path = self.write_map()
        original = path.read_text()
        for replacement in ("resolution: .nan", "resolution: 0", "resolution: true"):
            with self.subTest(value=replacement):
                path.write_text(original.replace("resolution: 0.1", replacement))
                with self.assertRaises(ValueError):
                    load_navigation_map(path)
        for suffix in ("mode: scale", "mode: raw", "origin: [0, 0, 0]", "x: !!python/object/apply:os.system ['echo bad']"):
            with self.subTest(value=suffix):
                path.write_text(original + "\n" + suffix)
                with self.assertRaises(ValueError):
                    load_navigation_map(path)

    def test_oversize_image_rejected_before_pixel_conversion(self):
        from unittest.mock import patch
        path = self.write_map()
        with patch("carapp.core.navigation_map.MapModel.MAX_CELLS", 2):
            with self.assertRaisesRegex(ValueError, "尺寸"):
                load_navigation_map(path)


class NavigationMapUITests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.model = MapModel()
        self.model.accept_map({"header": {"frame_id": "map"},
                               "info": {"width": 4, "height": 3, "resolution": .1,
                                        "origin": {"position": {"x": 2., "y": 3., "z": 0.},
                                                   "orientation": {"x": 0., "y": 0.,
                                                                   "z": math.sin(.35), "w": math.cos(.35)}}},
                               "data": (0,) * 11 + (100,)}, 1.)
        self.selections = []
        self.panel = NavigationMapPanel(self.root, self.model,
                                        lambda *args: self.selections.append(args))
        self.panel.pack(fill="both", expand=True)
        self.panel.on_tick()

    def tearDown(self):
        self.panel.on_close()
        self.root.destroy()

    def test_world_canvas_inverse_with_rotated_origin_zoom_and_pan(self):
        self.panel._zoom = 3.2
        self.panel._pan = (123., -56.)
        grid = self.model.snapshot()["grid"]
        point = (2.1, 3.15)
        xy = self.panel._point(*point, self.panel._geometry())
        actual = self.panel.canvas_to_world(*xy)
        self.assertAlmostEqual(actual[0], point[0])
        self.assertAlmostEqual(actual[1], point[1])
        self.assertTrue(valid_target(grid, *point))

    def test_current_car_has_coordinates_heading_and_can_be_centered(self):
        self.model.accept_map_pose((2.1,3.15,.7),time.monotonic())
        self.panel.set_localization(True)
        self.panel._pan=(200.,-100.)
        with patch.object(self.panel.canvas,'winfo_width',return_value=600), \
                patch.object(self.panel.canvas,'winfo_height',return_value=400):
            self.panel.on_tick()
            self.assertTrue(self.panel.canvas.find_withtag('vehicle'))
            text=self.panel.canvas.itemcget(self.panel.canvas.find_withtag('nav_vehicle_text')[0],'text')
            self.assertIn('当前小车',text)
            self.assertIn('x=2.10',text)
            self.assertIn('y=3.15',text)
            self.assertTrue(self.panel.center_vehicle())
            point=self.panel._point(2.1,3.15,self.panel._geometry())
            self.assertAlmostEqual(point[0],300.)
            self.assertAlmostEqual(point[1],200.)

    def test_initial_pose_is_never_presented_as_current_localization(self):
        self.model.accept_map_pose((2.1,3.15,.7),time.monotonic())
        self.panel.set_initial((2.1,3.15,.7))
        self.panel.set_localization(False,'等待雷达确认')
        self.panel.on_tick()
        self.assertFalse(self.panel.canvas.find_withtag('vehicle'))
        self.assertTrue(self.panel.canvas.find_withtag('nav_initial'))
        self.assertIn('未定位',self.panel.pose_label.cget('text'))
        self.assertEqual(self.panel.center_button.cget('state'),'disabled')

    def test_expired_coordinates_are_labeled_as_last_position(self):
        self.model.accept_map_pose((2.1,3.15,.7),time.monotonic()-3.)
        self.panel.set_localization(True)
        self.panel.on_tick()
        self.assertIn('上次位置',self.panel.pose_label.cget('text'))
        text=self.panel.canvas.itemcget(self.panel.canvas.find_withtag('nav_vehicle_text')[0],'text')
        self.assertIn('已过期',text)

    def test_free_selection_drag_heading_and_occupied_rejection(self):
        self.panel._zoom = 100.
        geometry = self.panel._geometry()
        origin = self.model.snapshot()["grid"]["origin"]
        def world(col, row):
            c, s = math.cos(origin[2]), math.sin(origin[2])
            return origin[0] + c * col * .1 - s * row * .1, origin[1] + s * col * .1 + c * row * .1
        start = self.panel._point(*world(.5, .5), geometry)
        end = self.panel._point(*world(2.5, .5), geometry)
        self.panel._select_start(types.SimpleNamespace(x=start[0], y=start[1]))
        self.panel._select_move(types.SimpleNamespace(x=end[0], y=end[1]))
        self.panel._select_finish(types.SimpleNamespace(x=end[0], y=end[1]))
        self.assertEqual(len(self.selections), 1)
        self.assertEqual(self.selections[0][0], "goal")
        self.assertAlmostEqual(self.selections[0][3], .7)
        occupied = self.panel._point(*world(3.5, 2.5), geometry)
        event = types.SimpleNamespace(x=occupied[0], y=occupied[1])
        self.panel._select_start(event)
        self.panel._select_finish(event)
        self.assertEqual(len(self.selections), 1)
        self.assertTrue(self.panel._selection_error)

    def test_setters_defer_overlay_render_until_tick(self):
        self.panel.set_plan([(2., 3.), (2.1, 3.1)])
        self.panel.set_goal((2.1, 3.1, 0.))
        self.panel.set_initial((2.05, 3.05, .4))
        self.assertFalse(self.panel.canvas.find_withtag("nav_path"))
        self.panel.on_tick()
        self.assertTrue(self.panel.canvas.find_withtag("nav_path"))
        self.assertTrue(self.panel.canvas.find_withtag("nav_goal"))
        self.assertTrue(self.panel.canvas.find_withtag("nav_initial"))
        self.panel.set_goal(None)
        self.panel.set_plan([])
        self.panel.on_tick()
        self.assertFalse(self.panel.canvas.find_withtag("nav_path"))
        self.assertFalse(self.panel.canvas.find_withtag("nav_goal"))

    def test_road_network_overlay_route_and_start_marker_render_on_tick(self):
        network = parse_road_network("waypoints:\n"
            "  - id: 0\n    pose: [2.1, 3.15, 0.0]\n    to: [1]\n"
            "  - id: 1\n    pose: [2.4, 3.15]\n    to: []\n")
        self.panel.set_network(network)
        self.panel.set_start((2.1, 3.15, 0.), 0)
        self.panel.set_route((0, 1))
        self.panel.set_plan(network.dense_path((0, 1)))
        self.assertFalse(self.panel.canvas.find_withtag("nav_network"))
        self.panel.on_tick()
        self.assertTrue(self.panel.canvas.find_withtag("nav_network"))
        self.assertTrue(self.panel.canvas.find_withtag("nav_start"))
        self.assertTrue(self.panel.canvas.find_withtag("nav_path"))
        self.panel.set_network(None)
        self.panel.on_tick()
        self.assertFalse(self.panel.canvas.find_withtag("nav_network"))
        self.assertFalse(self.panel.canvas.find_withtag("nav_start"))

    def test_replacing_model_clears_same_revision_raster_and_partial_selection(self):
        self.panel._select_start(types.SimpleNamespace(x=3, y=4))
        replacement = MapModel()
        message = {"header": {"frame_id": "map"},
                   "info": {"width": 4, "height": 3, "resolution": .1,
                            "origin": {"position": {"x": 0., "y": 0., "z": 0.},
                                       "orientation": {"x": 0., "y": 0., "z": 0., "w": 1.}}},
                   "data": (100,) * 12}
        replacement.accept_map(message, 2.)
        self.assertEqual(replacement.snapshot()["grid"]["revision"], self.panel._source_revision)
        self.panel.set_model(replacement)
        self.assertIsNone(self.panel._source_revision)
        self.assertIsNone(self.panel._selection)
        self.panel.on_tick()
        self.assertEqual(self.panel._grid["data"], (100,) * 12)

    def test_pan_mode_and_empty_model_never_select_a_target(self):
        self.panel.set_mode("pan")
        event = types.SimpleNamespace(x=20, y=30)
        self.panel._select_start(event)
        self.panel._select_move(types.SimpleNamespace(x=50, y=60))
        self.panel._select_finish(event)
        self.assertEqual(self.panel._pan, (30., 30.))
        self.assertFalse(self.selections)
        self.panel.set_model(MapModel())
        self.panel.set_mode("initial")
        self.panel._select_start(event)
        self.panel._select_finish(event)
        self.assertIsNone(self.panel.canvas_to_world(20, 30))
        self.assertFalse(self.selections)


if __name__ == "__main__":
    unittest.main()
