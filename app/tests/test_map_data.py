"""Map transport limits, TF composition, async reset and viewport caching."""
import copy
import math
import threading
import time
import tkinter as tk
import unittest
from unittest.mock import patch

from carapp.core.map_data import MapModel, world_to_grid
from carapp.ui.mappanel import MapPanel, cell_colour, FREE, OCCUPIED, UNKNOWN


def quaternion(yaw=0.):
    return dict(x=0., y=0., z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def grid_message(width=2, height=2, origin=(0., 0., 0.)):
    return dict(header=dict(frame_id="map"), info=dict(width=width, height=height,
                resolution=.05, origin=dict(position=dict(x=origin[0], y=origin[1], z=0.),
                orientation=quaternion(origin[2]))), data=[-1, 0, 65, 100] * (width * height // 4))


def transform(parent="map", child="odom", x=0., y=0., yaw=0.):
    return dict(header=dict(frame_id=parent), child_frame_id=child,
                transform=dict(translation=dict(x=x, y=y, z=0.), rotation=quaternion(yaw)))


def odometry(frame="odom", x=0., y=0., yaw=0.):
    return dict(header=dict(frame_id=frame), child_frame_id="base_link",
                pose=dict(pose=dict(position=dict(x=x, y=y, z=0.), orientation=quaternion(yaw))))


class MapModelTests(unittest.TestCase):
    def setUp(self):
        self.model = MapModel()

    def test_confirmed_map_pose_works_without_tf_and_expires_or_resets(self):
        self.model.accept_map(grid_message(),10.)
        self.assertTrue(self.model.accept_map_pose((.03,.04,.5),11.))
        snapshot=self.model.snapshot(11.2)
        self.assertEqual(snapshot['pose'],(.03,.04,.5))
        self.assertFalse(snapshot['pose_stale'])
        self.assertEqual(snapshot['pose_error'],'')
        self.assertTrue(self.model.snapshot(14.)['pose_stale'])
        self.model.reset(keep_map=True)
        self.assertIsNone(self.model.snapshot(14.)['pose'])

    def test_map_pose_rejects_raw_odom_frame_invalid_values_and_old_epochs(self):
        self.model.accept_map(grid_message(),10.)
        for pose in ((float('nan'),0.,0.),(1.,2.),(True,0.,0.)):
            self.assertFalse(self.model.accept_map_pose(pose,11.))
        self.assertFalse(self.model.accept_map_pose((1.,2.,0.),11.,frame_id='odom'))
        epoch=self.model.generation
        self.model.reset(keep_map=True)
        self.assertFalse(self.model.accept_map_pose((1.,2.,0.),11.,generation=epoch))
        self.assertTrue(self.model.accept_map_pose((1.,2.,0.),12.))
        self.assertFalse(self.model.accept_map_pose((9.,9.,0.),11.))
        self.assertEqual(self.model.snapshot(12.)['pose'],(1.,2.,0.))

    def test_reset_localization_keeps_static_map_without_redecoding(self):
        self.model.accept_map(grid_message(), 10.)
        self.model.accept_odom(odometry(frame="map"), 10.)
        before = self.model.snapshot(10.)
        generation = self.model.generation
        self.assertIsNotNone(before["pose"])
        self.model.reset(keep_map=True)
        after = self.model.snapshot(10.)
        self.assertIs(before["grid"]["data"], after["grid"]["data"])
        self.assertIs(before["grid"]["pixels"], after["grid"]["pixels"])
        self.assertIsNone(after["pose"])
        self.assertFalse(after["trajectory"])
        self.assertGreater(self.model.generation, generation)
        self.assertFalse(self.model.accept_map(grid_message(), 10., generation=generation))

    def test_grid_snapshot_immutable_payload_is_shared_not_copied(self):
        message = grid_message()
        self.assertTrue(self.model.accept_map(message, 10.))
        first = self.model.snapshot(11.)
        second = self.model.snapshot(12.)
        self.assertEqual(first["grid"]["data"], (-1, 0, 65, 100))
        self.assertIs(first["grid"]["data"], second["grid"]["data"])
        self.assertIs(first["grid"]["pixels"], second["grid"]["pixels"])
        self.assertIsNot(first["grid"], second["grid"])
        message["data"][0] = 100
        first["grid"]["width"] = 1000
        self.assertEqual(self.model.snapshot(12.)["grid"]["width"], 2)
        self.assertEqual(self.model.snapshot(12.)["grid"]["data"][0], -1)

    def test_worker_pixels_classify_and_reverse_ros_bottom_up_rows(self):
        message = grid_message(width=4, height=2)
        message["data"] = [-1, 0, 64, 65, 100, 30, -1, 50]
        self.assertTrue(self.model.accept_map(message, 10.))
        pixels = self.model.snapshot(10.)["grid"]["pixels"]
        self.assertIsInstance(pixels, bytes)
        self.assertEqual(pixels, bytes((2, 1, 0, 1, 0, 1, 1, 2)))

    def test_bad_frames_preserve_last_good_map_and_report_error(self):
        self.model.accept_map(grid_message(), 10.)
        previous = self.model.snapshot(10.)["grid"]
        bad = []
        for field, value in (("width", 8193), ("height", -1), ("width", True),
                             ("width", 2.0), ("resolution", math.nan), ("resolution", 0.)):
            message = grid_message()
            message["info"][field] = value
            bad.append(message)
        for value in ((-1, 0), (-2, 0, 0, 0), (101, 0, 0, 0),
                      (False, 0, 0, 0), (.5, 0, 0, 0), (math.inf, 0, 0, 0)):
            message = grid_message()
            message["data"] = value
            bad.append(message)
        message = grid_message()
        message["info"].update(width=2001, height=2000)
        bad.append(message)
        message = grid_message()
        message["info"]["origin"]["orientation"] = dict(x=0, y=0, z=0, w=0)
        bad.append(message)
        message = grid_message()
        message["info"]["origin"]["position"]["x"] = math.inf
        bad.append(message)
        bad.extend((None, {}, {"header": None}))
        for message in bad:
            with self.subTest(message=message):
                self.assertFalse(self.model.accept_map(message, 11.))
                current = self.model.snapshot(11.)
                self.assertEqual(current["grid"], previous)
                self.assertTrue(current["error"])
        self.assertTrue(self.model.accept_map(grid_message(), 12.))
        self.assertFalse(self.model.snapshot(12.)["error"])

    def test_composes_rotated_map_to_odom_and_odom_pose(self):
        self.model.accept_map(grid_message(), 10.)
        self.model.accept_tf(dict(transforms=[transform(x=10, y=20, yaw=math.pi / 2)]), 10.)
        self.model.accept_odom(odometry(x=2, y=3, yaw=math.pi / 4), 10.1)
        snapshot = self.model.snapshot(10.2)
        x, y, yaw = snapshot["pose"]
        self.assertAlmostEqual(x, 7.)
        self.assertAlmostEqual(y, 22.)
        self.assertAlmostEqual(yaw, 3 * math.pi / 4)
        self.assertFalse(snapshot["pose_error"])

    def test_reverse_tf_and_multi_hop_path(self):
        self.model.accept_map(grid_message(), 10.)
        self.model.accept_tf(dict(transforms=[transform("odom", "middle", x=2),
                                              transform("middle", "map", y=-4)]), 10.)
        self.model.accept_odom(odometry(x=3, y=1), 10.)
        self.assertEqual(self.model.snapshot(10.)["pose"], (1., 5., 0.))

    def test_missing_tf_does_not_mislabel_odom_as_map_pose(self):
        self.model.accept_map(grid_message(), 10.)
        self.model.accept_odom(odometry(x=3, y=1), 10.)
        snapshot = self.model.snapshot(10.)
        self.assertIsNone(snapshot["pose"])
        self.assertIn("坐标变换", snapshot["pose_error"])

    def test_direct_map_odom_and_tf_base_fallback(self):
        self.model.accept_map(grid_message(), 10.)
        self.model.accept_odom(odometry(frame="/map", x=3, y=1), 10.)
        self.assertEqual(self.model.snapshot(10.)["pose"], (3., 1., 0.))
        self.model.reset()
        self.model.accept_map(grid_message(), 20.)
        self.model.accept_tf(dict(transforms=[transform("map", "base_footprint", x=2)]), 20.)
        self.assertEqual(self.model.snapshot(20.)["pose"], (2., 0., 0.))

    def test_stale_pose_is_flagged_without_adding_stale_trajectory(self):
        self.model.accept_map(grid_message(), 10.)
        self.model.accept_tf(dict(transforms=[transform()]), 10.)
        self.model.accept_odom(odometry(x=1), 10.)
        self.assertEqual(self.model.snapshot(10.)["trajectory"], ((1., 0.),))
        self.model.accept_odom(odometry(x=2), 15.)
        stale = self.model.snapshot(15.)
        self.assertTrue(stale["pose_stale"])
        self.assertIn("过期", stale["pose_error"])
        self.assertEqual(stale["trajectory"], ((1., 0.),))
        self.model.accept_tf(dict(transforms=[transform()]), 15.)
        self.assertEqual(self.model.snapshot(15.)["trajectory"], ((1., 0.), (2., 0.)))

    def test_static_tf_does_not_expire_with_fresh_odometry(self):
        self.model.accept_map(grid_message(), 10.)
        self.model.accept_tf(dict(transforms=[transform()]), 10., static=True)
        self.model.accept_odom(odometry(x=2), 100.)
        self.assertFalse(self.model.snapshot(100.)["pose_stale"])

    def test_bad_tf_and_odometry_preserve_last_pose(self):
        self.model.accept_map(grid_message(), 10.)
        self.model.accept_tf(dict(transforms=[transform()]), 10.)
        self.model.accept_odom(odometry(x=1), 10.)
        invalid = transform()
        invalid["transform"]["rotation"]["w"] = math.nan
        self.assertFalse(self.model.accept_tf(dict(transforms=[invalid]), 11.))
        invalid = odometry()
        invalid["pose"]["pose"]["position"]["y"] = math.inf
        self.assertFalse(self.model.accept_odom(invalid, 11.))
        self.assertEqual(self.model.snapshot(11.)["pose"], (1., 0., 0.))

    def test_rotated_grid_origin_world_conversion(self):
        self.model.accept_map(grid_message(origin=(10., 20., math.pi / 2)), 10.)
        grid = self.model.snapshot(10.)["grid"]
        gx, gy = world_to_grid(grid, 9.9, 20.05)
        self.assertAlmostEqual(gx, 1.)
        self.assertAlmostEqual(gy, 2.)

    def test_reset_rejects_queued_frame_and_clears_all_data(self):
        epoch = self.model.generation
        self.model.accept_map(grid_message(), 10.)
        self.model.accept_odom(odometry(frame="map", x=1), 10.)
        self.model.snapshot(10.)
        self.model.reset()
        self.assertGreater(self.model.generation, epoch)
        self.assertFalse(self.model.accept_map(grid_message(), 11., generation=epoch))
        snapshot = self.model.snapshot(11.)
        self.assertIsNone(snapshot["grid"])
        self.assertIsNone(snapshot["pose"])
        self.assertEqual(snapshot["trajectory"], ())

    def test_reset_during_decode_cannot_commit_previous_vehicle_frame(self):
        entered, resume = threading.Event(), threading.Event()

        class PausingList(list):
            def __iter__(self):
                entered.set()
                if not resume.wait(2.):
                    raise RuntimeError("decode synchronization failed")
                return super().__iter__()

        message = grid_message()
        message["data"] = PausingList(message["data"])
        result = []
        worker = threading.Thread(target=lambda: result.append(self.model.accept_map(message, 10.)))
        worker.start()
        try:
            self.assertTrue(entered.wait(2.))
            self.model.reset()
        finally:
            resume.set()
            worker.join(3.)
        self.assertFalse(worker.is_alive())
        self.assertEqual(result, [False])
        self.assertIsNone(self.model.snapshot(11.)["grid"])


class MapPanelTests(unittest.TestCase):
    def setUp(self):
        try:
            self.root = tk.Tk()
            self.root.withdraw()
        except tk.TclError as exc:
            self.skipTest(str(exc))
        self.model = MapModel()
        self.panel = MapPanel(self.root, self.model)
        self.panel.pack(fill="both", expand=True)
        self.root.update_idletasks()

    def tearDown(self):
        if hasattr(self, "root"):
            self.root.destroy()

    def test_unchanged_revision_keeps_raster_and_reset_clears_it(self):
        self.model.accept_map(grid_message(), time.monotonic())
        self.panel.on_tick()
        photo = self.panel._photo
        self.panel.on_tick()
        self.assertIs(self.panel._photo, photo)
        self.model.reset()
        self.panel.on_tick()
        self.assertIsNone(self.panel._photo)
        self.assertIsNone(self.panel._grid)

    def test_pillow_optional_fallback_displays_three_cell_classes(self):
        self.assertEqual([cell_colour(v) for v in (-1, 0, 64, 65, 100)],
                         [UNKNOWN, FREE, FREE, OCCUPIED, OCCUPIED])
        self.model.accept_map(grid_message(), time.monotonic())
        with patch("carapp.ui.mappanel.HAS_PIL", False):
            self.panel.on_tick()
            self.assertIsNotNone(self.panel._photo)
            self.panel._change_zoom(2)
            self.assertIsNotNone(self.panel._photo)

    def test_main_thread_render_never_reads_occupancy_tuple(self):
        self.model.accept_map(grid_message(), time.monotonic())
        snapshot = self.model.snapshot()
        snapshot["grid"]["data"] = object()  # It cannot be indexed or iterated.
        for pillow in (True, False):
            with self.subTest(pillow=pillow):
                try:
                    import PIL
                except ImportError:
                    if pillow:
                        continue
                self.panel._source_revision = None
                self.panel._render_key = None
                with patch("carapp.ui.mappanel.HAS_PIL", pillow), patch.object(
                        self.model, "snapshot", return_value=snapshot):
                    self.panel.on_tick()
                self.assertIsNotNone(self.panel._photo)


if __name__ == "__main__":
    unittest.main()
