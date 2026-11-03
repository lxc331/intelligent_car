# -*- coding: utf-8 -*-
"""Generate a demo road network that matches app/savedMap/sandbox_half/map.yaml.

The sandbox map is a real SLAM map; the vendor self_driving_network.yaml was
drawn for another arena (18 consistency problems on this map).  This script
walks the map itself: it finds the free-space grid path between two distant
cells, then simplifies it into straight, fully-free segments.  The result is a
traversable spine whose edges pass RoadNetwork.check_map().

The result is DEMO data, not a surveyed field network -- replace it once the
sandbox road network is measured on site.
"""
import math
from collections import deque
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "app"))

from carapp.core.navigation_map import load_navigation_map, valid_target  # noqa: E402
from carapp.core.road_network import RoadNetwork, RoadNode  # noqa: E402

MAP = Path(__file__).resolve().parents[1] / "app" / "savedMap" / "sandbox_half" / "map.yaml"
OUT = MAP.with_name("road_network.yaml")
MAX_SEGMENT = 2.0     # metres; keeps FollowPath lookahead sane
STEP = 0.02           # sampling step, matches check_map resolution


def free(bundle, x, y):
    return valid_target(bundle, x, y)


def line_free(bundle, a, b):
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    count = max(2, int(length / STEP) + 1)
    for index in range(count + 1):
        t = index / count
        if not free(bundle, a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t):
            return False
    return True


def grid_path(bundle, start, goal):
    """Breadth-first path over free cells, returning world-coordinate points."""
    width, height = bundle.width, bundle.height
    resolution = bundle.resolution
    origin = bundle.origin
    data = bundle.grid["data"]

    def cell(x, y):
        return (int((x - origin[0]) / resolution), int((y - origin[1]) / resolution))

    def world(col, row):
        return (origin[0] + (col + .5) * resolution, origin[1] + (row + .5) * resolution)

    def is_free(col, row):
        if not (0 <= col < width and 0 <= row < height):
            return False
        return data[row * width + col] == 0

    first, last = cell(*start), cell(*goal)
    if not is_free(*first) or not is_free(*last):
        raise SystemExit("start or goal cell is not free")
    previous = {first: None}
    queue = deque([first])
    while queue:
        current = queue.popleft()
        if current == last:
            break
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbour = (current[0] + dc, current[1] + dr)
            if neighbour in previous or not is_free(*neighbour):
                continue
            previous[neighbour] = current
            queue.append(neighbour)
    if last not in previous:
        raise SystemExit("no free path between the two cells")
    chain = []
    node = last
    while node is not None:
        chain.append(node)
        node = previous[node]
    chain.reverse()
    return [world(col, row) for col, row in chain]


def main():
    bundle = load_navigation_map(MAP)
    width, height, resolution = bundle.width, bundle.height, bundle.resolution
    origin = bundle.origin
    print("map %dx%d res %.3f origin %s" % (width, height, resolution, origin))

    free_points = []
    for row in range(height):
        for col in range(width):
            if bundle.grid["data"][row * width + col] == 0:
                free_points.append((origin[0] + (col + .5) * resolution,
                                    origin[1] + (row + .5) * resolution))
    print("free cells:", len(free_points))
    if len(free_points) < 20:
        raise SystemExit("not enough free space for a demo network")

    centre = (origin[0] + width * resolution / 2., origin[1] + height * resolution / 2.)
    start = min(free_points, key=lambda p: math.hypot(p[0] - centre[0], p[1] - centre[1]))
    goal = max(free_points, key=lambda p: math.hypot(p[0] - start[0], p[1] - start[1]))
    route = grid_path(bundle, start, goal)
    print("grid path points:", len(route))

    # Line-of-sight simplification: each segment is straight and fully free.
    nodes = [route[0]]
    index = 0
    while index < len(route) - 1:
        furthest = index + 1
        for candidate in range(index + 1, len(route)):
            if math.hypot(route[candidate][0] - route[index][0],
                          route[candidate][1] - route[index][1]) > MAX_SEGMENT:
                break
            if line_free(bundle, route[index], route[candidate]):
                furthest = candidate
            else:
                break
        nodes.append(route[furthest])
        index = furthest
    nodes = [(round(x, 3), round(y, 3)) for x, y in nodes]
    print("simplified nodes:", len(nodes))

    network = RoadNetwork(tuple(
        RoadNode(id=index,
                 x=point[0], y=point[1],
                 to=(index - 1, index + 1) if 0 < index < len(nodes) - 1
                 else ((1,) if index == 0 else (len(nodes) - 2,)))
        for index, point in enumerate(nodes)
    ))
    problems = network.check_map(bundle.grid)
    print("nodes=%d directed edges=%d consistency problems=%d"
          % (len(network), len(network.edges()), len(problems)))
    for problem in problems:
        print("  !", problem)
    if problems:
        raise SystemExit("demo network is inconsistent with the map")
    network.save(OUT)
    print("wrote", OUT)
    route_ids = network.shortest_path(0, len(network) - 1)
    print("route 0 -> %d: %s  (%.2f m, %d points)"
          % (len(network) - 1, route_ids, network.path_length(route_ids),
             len(network.dense_path(route_ids))))


if __name__ == "__main__":
    main()
