"""Offline tests for the road-network graph used by map navigation."""
import math
from pathlib import Path
import tempfile
import unittest

from carapp.core.road_network import (
    DEFAULT_STEP, MAX_YAML_BYTES, RoadNetwork, RoadNode, find_road_network,
    load_road_network, parse_road_network)


VENDOR_SAMPLE = Path(r"D:\项目\intelligent_car-main\.work\vendor_src\src\road_network"
                     r"\config\road_network.yaml")


def node(node_id, x, y, to=(), yaw=None, name=None):
    return RoadNode(node_id, x, y, yaw, tuple(to), name)


def grid(width=10, height=10, resolution=.1, origin=(0., 0., 0.), occupied=(), unknown=()):
    data = []
    for row in range(height):
        for column in range(width):
            if (column, row) in occupied:
                data.append(100)
            elif (column, row) in unknown:
                data.append(-1)
            else:
                data.append(0)
    return {"width": width, "height": height, "resolution": resolution,
            "origin": origin, "data": tuple(data)}


class ParseTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def write(self, text, name="road_network.yaml"):
        path = self.directory / name
        path.write_text(text, encoding="utf-8")
        return path

    def test_vendor_shape_loads_with_optional_yaw_and_name(self):
        network = load_road_network(self.write(
            "waypoints:\n"
            "- id: 0\n"
            "  pose: [0.0, 0.0, 0.0]\n"
            "  to: [1, 2]\n"
            "- id: 1\n"
            "  pose: [0.98, 0.0]\n"          # yaw omitted -> travel direction
            "  to: [0]\n"
            "  name: 仓库\n"
            "- id: 2\n"
            "  pose: [0.98, 1.1, -0.04]\n"
            "  to: []\n"))
        self.assertEqual(network.ids(), (0, 1, 2))
        self.assertEqual(network.node(1).yaw, None)
        self.assertEqual(network.node(1).name, "仓库")
        self.assertEqual(network.node(0).to, (1, 2))
        self.assertEqual(network.node(2).to, ())
        self.assertAlmostEqual(network.node(1).x, .98)
        self.assertAlmostEqual(network.node(2).yaw, -.04)

    def test_nodes_alias_and_source_path_recorded(self):
        path = self.write("nodes:\n- id: 4\n  pose: [1, 2]\n")
        network = load_road_network(path)
        self.assertEqual(network.ids(), (4,))
        self.assertEqual(network.source_path, path.resolve())

    def test_malformed_documents_are_rejected(self):
        cases = {
            "top level list": "- id: 0\n  pose: [0, 0]\n",
            "missing waypoints": "other: []\n",
            "empty list": "waypoints: []\n",
            "waypoints not list": "waypoints: {}\n",
            "unknown document field": "waypoints:\n- id: 0\n  pose: [0, 0]\nother: 1\n",
            "node not mapping": "waypoints:\n- 3\n",
            "missing id": "waypoints:\n- pose: [0, 0]\n",
            "bool id": "waypoints:\n- id: true\n  pose: [0, 0]\n",
            "negative id": "waypoints:\n- id: -1\n  pose: [0, 0]\n",
            "float id": "waypoints:\n- id: 1.5\n  pose: [0, 0]\n",
            "pose too short": "waypoints:\n- id: 0\n  pose: [0]\n",
            "pose too long": "waypoints:\n- id: 0\n  pose: [0, 0, 0, 0]\n",
            "pose not finite": "waypoints:\n- id: 0\n  pose: [0, .nan]\n",
            "pose string": "waypoints:\n- id: 0\n  pose: [a, b]\n",
            "pose out of range": "waypoints:\n- id: 0\n  pose: [1e9, 0]\n",
            "to not list": "waypoints:\n- id: 0\n  pose: [0, 0]\n  to: 1\n",
            "to bool": "waypoints:\n- id: 0\n  pose: [0, 0]\n  to: [true]\n",
            "self loop": "waypoints:\n- id: 0\n  pose: [0, 0]\n  to: [0]\n",
            "duplicate target": "waypoints:\n- id: 0\n  pose: [0, 0]\n  to: [1, 1]\n  - id: 1\n"
                                "    pose: [1, 1]\n",
            "unknown target": "waypoints:\n- id: 0\n  pose: [0, 0]\n  to: [7]\n",
            "duplicate id": "waypoints:\n- id: 0\n  pose: [0, 0]\n- id: 0\n  pose: [1, 1]\n",
            "unknown node field": "waypoints:\n- id: 0\n  pose: [0, 0]\n  colour: red\n",
            "empty name": "waypoints:\n- id: 0\n  pose: [0, 0]\n  name: '  '\n",
            "long name": "waypoints:\n- id: 0\n  pose: [0, 0]\n  name: %s\n" % ("名" * 50),
            "duplicate yaml key": "waypoints:\n- id: 0\n  pose: [0, 0]\n  pose: [1, 1]\n",
            "bad yaml": "waypoints: [\n",
        }
        for label, text in cases.items():
            with self.subTest(label):
                with self.assertRaises(ValueError):
                    load_road_network(self.write(text))

    def test_too_many_nodes_and_oversized_file(self):
        entries = "".join("  - id: %d\n    pose: [%d, 0]\n" % (index, index) for index in range(501))
        with self.assertRaises(ValueError):
            load_road_network(self.write("waypoints:\n" + entries))
        oversized = "waypoints:\n- id: 0\n  pose: [0, 0]\n#" + "x" * MAX_YAML_BYTES
        with self.assertRaises(ValueError):
            load_road_network(self.write(oversized))

    def test_non_utf8_file_is_rejected(self):
        path = self.directory / "road_network.yaml"
        path.write_bytes(b"waypoints:\n- id: 0\n  pose: [0, 0]\n \xff\n")
        with self.assertRaises(ValueError):
            load_road_network(path)

    def test_missing_file_raises(self):
        with self.assertRaises((OSError, ValueError)):
            load_road_network(self.directory / "absent.yaml")

    @unittest.skipUnless(VENDOR_SAMPLE.is_file(), "厂家样例路网不在本机")
    def test_vendor_sample_yaml_loads(self):
        network = load_road_network(VENDOR_SAMPLE)
        self.assertGreaterEqual(len(network), 9)
        self.assertIn(0, network.ids())
        self.assertTrue(network.node(0).to)


class PlanTests(unittest.TestCase):
    def network(self):
        # 0-1-3 是 10 m + 10 m，0-2-3 是 1 m + 1 m：最少跳数会选错，Dijkstra 应选 0-2-3
        return RoadNetwork([
            node(0, 0., 0., (1, 2)),
            node(1, 10., 0., (3,)),
            node(2, 1., 0., (3,)),
            node(3, 2., 0., ()),
        ])

    def test_shortest_path_uses_metres_not_hops(self):
        self.assertEqual(self.network().shortest_path(0, 3), (0, 2, 3))
        self.assertAlmostEqual(self.network().path_length((0, 2, 3)), 2.)

    def test_shortest_path_single_node_and_reverse(self):
        network = self.network()
        self.assertEqual(network.shortest_path(2, 2), (2,))
        with self.assertRaises(ValueError):
            network.shortest_path(3, 0)          # 单向：无法回退

    def test_shortest_path_tie_breaks_by_lower_id(self):
        network = RoadNetwork([
            node(0, 0., 0., (2, 1)),
            node(1, 0., 1., (3,)),
            node(2, 0., 1., (3,)),
            node(3, 0., 2., ()),
        ])
        self.assertEqual(network.shortest_path(0, 3), (0, 1, 3))

    def test_unknown_node_raises(self):
        with self.assertRaises(ValueError):
            self.network().shortest_path(0, 9)
        with self.assertRaises(ValueError):
            self.network().node(9)
        with self.assertRaises(ValueError):
            self.network().length(0, 9)

    def test_edges_and_vertices(self):
        network = self.network()
        self.assertEqual(len(network.edges()), 4)
        self.assertEqual(network.edges()[0], (0, 1, 10.))
        self.assertEqual(network.vertices()[0], (0, 0., 0., None, None))

    def test_nearest_node_and_entry(self):
        network = self.network()
        found, distance = network.nearest_node(.9, .1)
        self.assertEqual(found.id, 2)
        self.assertAlmostEqual(distance, math.hypot(.1, .1))
        self.assertEqual(network.entry_node(.9, .1).id, 2)
        self.assertIsNone(network.entry_node(5., 5., max_distance=.2))
        with self.assertRaises(ValueError):
            network.entry_node(0., 0., max_distance=0.)


class DensePathTests(unittest.TestCase):
    def network(self):
        return RoadNetwork([
            node(0, 0., 0., (1,), yaw=1.57),
            node(1, 1., 0., (2,)),
            node(2, 1., 1., (), yaw=3.14),
            node(3, 5., 5., ()),
        ])

    def test_points_follow_edges_without_duplicate_vertices(self):
        points = self.network().dense_path((0, 1))
        self.assertEqual(points[0], (0., 0., 1.57))          # 起点用自身 yaw
        self.assertEqual(len(points), 1 + 20)                # 1 m / 0.05 m
        self.assertAlmostEqual(points[1][0], DEFAULT_STEP)
        self.assertAlmostEqual(points[-1][0], 1.)
        self.assertEqual(points[1][2], 0.)                   # 行进方向
        self.assertEqual(points.count(points[-1]), 1)

    def test_intermediate_heading_and_goal_yaw(self):
        points = self.network().dense_path((0, 1, 2))
        self.assertAlmostEqual(points[-1][1], 1.)
        self.assertAlmostEqual(points[-1][2], 3.14)          # 终点 yaw 覆盖行进方向
        turning = [point for point in points if abs(point[1] - 0.) < 1e-9][-1]
        self.assertEqual(turning[2], 0.)

    def test_single_node_and_step_bounds(self):
        network = self.network()
        self.assertEqual(len(network.dense_path((3,))), 1)
        self.assertAlmostEqual(len(network.dense_path((0, 1), step=.2)), 6)
        for bad in (0., -.1, 5., float("nan")):
            with self.assertRaises(ValueError):
                network.dense_path((0, 1), step=bad)

    def test_missing_edge_and_unknown_node_raise(self):
        network = self.network()
        with self.assertRaises(ValueError):
            network.dense_path((0, 2))
        with self.assertRaises(ValueError):
            network.dense_path(())
        with self.assertRaises(ValueError):
            network.dense_path((0, 9))

    def test_coincident_nodes_do_not_divide_by_zero(self):
        network = RoadNetwork([node(0, 1., 1., (1,)), node(1, 1., 1., ())])
        points = network.dense_path((0, 1))
        self.assertEqual(len(points), 2)
        self.assertTrue(all(math.isfinite(value) for point in points for value in point))


class RoundTripTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def test_text_and_save_round_trip(self):
        network = RoadNetwork([
            node(0, 0.98, -.8, (1, 2), yaw=-1.57, name="仓库"),
            node(1, 0.98, 0., (0,), yaw=0.),
            node(2, -.5, 2.25, ()),
        ])
        reloaded = parse_road_network(network.text())
        self.assertEqual(reloaded.vertices(), network.vertices())
        path = network.save(self.directory / "saved" / "road_network.yaml")
        self.assertTrue(path.is_file())
        again = load_road_network(path)
        self.assertEqual(again.vertices(), network.vertices())
        self.assertEqual(again.edges(), network.edges())

    def test_non_ascii_names_stay_readable(self):
        network = RoadNetwork([node(0, 0., 0., name="红绿灯")])
        text = network.text()
        self.assertIn("name: 红绿灯", text)
        self.assertEqual(parse_road_network(text).node(0).name, "红绿灯")

    def test_find_road_network_beside_map(self):
        (self.directory / "map.yaml").write_text("image: map.pgm\n", encoding="utf-8")
        self.assertIsNone(find_road_network(self.directory / "map.yaml"))
        network_file = self.directory / "road_network.yaml"
        network_file.write_text("waypoints:\n- id: 0\n  pose: [0, 0]\n", encoding="utf-8")
        self.assertEqual(find_road_network(self.directory / "map.yaml"), network_file)


class MapConsistencyTests(unittest.TestCase):
    def test_free_network_reports_nothing(self):
        network = RoadNetwork([node(0, .05, .05, (1,)), node(1, .45, .05, (0,))])
        self.assertEqual(network.check_map(grid()), ())

    def test_node_on_obstacle_or_unknown_is_reported(self):
        network = RoadNetwork([node(0, .05, .05, ()), node(1, .25, .25, ())])
        problems = network.check_map(grid(occupied={(0, 0)}))
        self.assertIn("节点 0 落在障碍上", problems)
        self.assertEqual(len(problems), 1)
        problems = network.check_map(grid(unknown={(2, 2)}))
        self.assertIn("节点 1 落在未知区域", problems)

    def test_edge_crossing_wall_is_reported(self):
        network = RoadNetwork([node(0, .05, .05, (1,)), node(1, .05, .95, ())])
        problems = network.check_map(grid(occupied={(0, 5)}))
        self.assertIn("边 0→1 穿过障碍", problems)

    def test_node_outside_map_and_edge_leaving_map(self):
        network = RoadNetwork([node(0, .05, .05, (1,)), node(1, 2.0, .05, ())])
        problems = network.check_map(grid())
        self.assertIn("节点 1 在地图范围外", problems)
        self.assertIn("边 0→1 穿出地图范围", problems)

    def test_check_honours_rotated_origin(self):
        network = RoadNetwork([node(0, 1.05, 2.05, ())])
        self.assertEqual(network.check_map(grid(origin=(1., 2., 0.))), ())
        self.assertIn("节点 0 在地图范围外", network.check_map(grid()))


if __name__ == "__main__":
    unittest.main()
