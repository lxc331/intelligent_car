"""Hand-built road graph: edits are checked against the project keep-out mask."""
import shutil
import tempfile
import unittest
from pathlib import Path

from carapp.core.network_editor import NetworkEditor
from carapp.core.sandbox_project import load_project

APP = Path(__file__).resolve().parent.parent
FIXTURE = APP / "savedMap" / "sandbox_nav_ready_20261101"


def corridor_grid(width=60, height=60, resolution=.05, walls=(19, 33)):
    """Two vertical walls with a free corridor between them."""
    data = [0] * (width * height)
    for row in range(height):
        for column in walls:
            data[row * width + column] = 100
    return dict(width=width, height=height, resolution=resolution, origin=(0., 0., 0.), data=data)


def project_with(grid):
    """Minimal stand-in: the editor only reaches .keepout.grid and .network."""
    from carapp.core.road_network import RoadNetwork, RoadNode

    class Bundle:
        def __init__(self, grid):
            self.grid = grid

    class FakeProject:
        def __init__(self):
            self.path = Path("fake/sandbox.json")
            self.keepout = Bundle(grid)
            self.network = RoadNetwork([])
            self.metadata = {}

    return FakeProject()


class NetworkEditorTests(unittest.TestCase):
    def setUp(self):
        self.grid = corridor_grid()
        self.editor = NetworkEditor(project_with(self.grid))

    def test_clearing_and_rebuilding_a_network_works(self):
        editor = NetworkEditor(project_with(self.grid))
        self.assertIn("已清空", editor.clear())
        self.assertEqual(len(editor.nodes), 0)
        self.assertFalse(editor.edges)
        with self.assertRaises(ValueError):
            editor.save()  # a blank graph has nothing to save yet
        first, error = editor.add_node(1.3, .4)
        self.assertIsNotNone(first, error)
        second, error = editor.add_node(1.3, 1.2)
        self.assertIsNotNone(second, error)
        ok, message = editor.connect(first, second)
        self.assertTrue(ok, message)

    def test_node_outside_corridor_is_rejected(self):
        node_id, error = self.editor.add_node(.99, .5)   # on the wall column
        self.assertIsNone(node_id)
        self.assertIn("余量", error)

    def test_second_node_within_8cm_is_refused(self):
        self.assertIsNotNone(self.editor.add_node(1.3, .5)[0])
        node_id, error = self.editor.add_node(1.32, .5)
        self.assertIsNone(node_id)
        self.assertIn("8 cm", error)

    def test_edge_along_the_corridor_reports_its_width(self):
        left, _ = self.editor.add_node(1.3, .5)
        right, _ = self.editor.add_node(1.3, 1.5)
        ok, message = self.editor.connect(left, right)
        self.assertTrue(ok, message)
        self.assertIn("最窄", message)
        self.assertGreaterEqual(self.editor.edge_widths()["%d:%d" % (left, right)], self.editor.min_width)

    def test_edge_across_the_wall_is_rejected(self):
        left, _ = self.editor.add_node(.5, .5)
        right, _ = self.editor.add_node(2.5, .5)
        ok, message = self.editor.connect(left, right)
        self.assertFalse(ok)
        self.assertIn("m", message)

    def test_removing_a_node_also_drops_its_edges(self):
        left, _ = self.editor.add_node(1.3, .5)
        right, _ = self.editor.add_node(1.3, 1.5)
        self.assertTrue(self.editor.connect(left, right)[0])
        self.assertEqual(self.editor.remove_node(left), "")
        self.assertEqual(self.editor.edges, set())
        self.assertNotIn(left, self.editor.nodes)

    def test_saving_without_edges_is_refused(self):
        self.editor.add_node(1.3, .5)
        with self.assertRaises(ValueError):
            self.editor.save()

    def test_double_connection_is_refused(self):
        left, _ = self.editor.add_node(1.3, .5)
        right, _ = self.editor.add_node(1.3, 1.5)
        self.assertTrue(self.editor.connect(left, right)[0])
        ok, message = self.editor.connect(right, left)
        self.assertFalse(ok)
        self.assertIn("已经连过", message)


@unittest.skipUnless(FIXTURE.is_dir(), "缺少沙盘工程样例")
class RoadEditorWindowTests(unittest.TestCase):
    """The standalone window loads a project and selects nodes on click."""

    def setUp(self):
        import tkinter as tk
        try:
            self.root = tk.Tk()
            self.root.withdraw()
        except tk.TclError as exc:
            self.skipTest("没有图形环境：%s" % exc)

    def tearDown(self):
        self.root.destroy()

    def test_window_loads_project_and_toggles_selection(self):
        from carapp.road_editor import RoadEditorWindow

        with tempfile.TemporaryDirectory() as folder:
            copy = Path(folder) / FIXTURE.name
            shutil.copytree(FIXTURE, copy)
            window = RoadEditorWindow(self.root)
            try:
                self.assertTrue(window.load(copy / "sandbox.json"))
                self.assertIsNotNone(window.editor)
                self.assertGreater(len(window.editor.nodes), 0)
                node = next(iter(window.editor.nodes))
                x, y = window.editor.nodes[node]
                window._click("network", x, y, 0.)
                self.assertEqual(window.selected, node)
                window._click("network", x, y, 0.)
                self.assertIsNone(window.selected)
                self.assertIn(FIXTURE.name, window.path_label.cget("text"))
            finally:
                window.destroy()

    def test_open_dialog_starts_in_an_existing_folder(self):
        """APP_DIR is a plain string; joining it must not break the dialog."""
        from pathlib import Path
        from unittest import mock

        from carapp.road_editor import RoadEditorWindow

        window = RoadEditorWindow(self.root)
        try:
            with mock.patch("carapp.road_editor.filedialog.askopenfilename",
                            return_value="") as dialog:
                window.ask_open()
            self.assertEqual(dialog.call_count, 1)
            initial = Path(dialog.call_args.kwargs["initialdir"])
            self.assertTrue(initial.is_dir(), initial)
        finally:
            window.destroy()


@unittest.skipUnless(FIXTURE.is_dir(), "缺少沙盘工程样例")
class SaveNetworkTests(unittest.TestCase):
    def test_edited_graph_lands_in_the_project_and_still_loads(self):
        with tempfile.TemporaryDirectory() as folder:
            copy = Path(folder) / FIXTURE.name
            shutil.copytree(FIXTURE, copy)
            project = load_project(copy / "sandbox.json")
            before = len(project.network)
            editor = NetworkEditor(project)
            editor.remove_node(max(editor.nodes))
            saved = editor.save()
            self.assertEqual(len(saved.network), before - 1)
            again = load_project(copy / "sandbox.json")
            self.assertEqual(len(again.network), before - 1)
            self.assertTrue(again.metadata["edge_widths"])


if __name__ == "__main__":
    unittest.main()
