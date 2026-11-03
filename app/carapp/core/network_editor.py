"""Hand-built road graph on top of a sandbox project.

Clicking the map adds nodes and connecting two nodes adds an edge, so a corridor
the automatic builder refuses can be corrected by hand. Every edit is checked
against the project's own keep-out mask before it is accepted, and ``save``
rewrites ``road_network.yaml`` inside the project (refreshing the hash list), so
an edited project still passes ``load_project``.
"""
import math
from collections import deque

from carapp.core.navigation_path import swept_clear
from carapp.core.road_network import RoadNetwork, RoadNode
from carapp.core.sandbox_project import save_network

MIN_EDGE_WIDTH = .20  # metres; the floor for an edge to be worth planning
FOOTPRINT = (.20, .10, .09)  # (front, rear, half) — 0.30 x 0.18 m carriage
BLOCKED = 100  # keep-out grid: 0 free, 100 obstacle, 255 unknown
LONGEST_EDGE = 2.0


def _grid_origin(grid):
    ox, oy, yaw = grid["origin"]
    return float(ox), float(oy), float(yaw)


class _Clearance:
    """Distance in metres from every free cell to the nearest blocked cell."""

    def __init__(self, grid):
        self.grid = grid
        width, height, resolution = grid["width"], grid["height"], grid["resolution"]
        distance = [-1] * (width * height)
        queue = deque()
        for index, value in enumerate(grid["data"]):
            if value >= BLOCKED:
                distance[index] = 0
                queue.append(index)
        while queue:
            index = queue.popleft()
            step = distance[index] + 1
            column, row = index % width, index // width
            neighbours = []
            if column:
                neighbours.append(index - 1)
            if column + 1 < width:
                neighbours.append(index + 1)
            if row:
                neighbours.append(index - width)
            if row + 1 < height:
                neighbours.append(index + width)
            for neighbour in neighbours:
                if distance[neighbour] < 0:
                    distance[neighbour] = step
                    queue.append(neighbour)
        self.distance = [(-1.0 if value < 0 else value * resolution) for value in distance]

    def at(self, x, y):
        """Metres to the nearest blocked cell; -1 outside the grid or unknown."""
        ox, oy, yaw = _grid_origin(self.grid)
        dx, dy = x - ox, y - oy
        cosine, sine = math.cos(yaw), math.sin(yaw)
        local_x, local_y = cosine * dx + sine * dy, -sine * dx + cosine * dy
        column = int(math.floor(local_x / self.grid["resolution"]))
        row = int(math.floor(local_y / self.grid["resolution"]))
        if not (0 <= column < self.grid["width"] and 0 <= row < self.grid["height"]):
            return -1.0
        return self.distance[row * self.grid["width"] + column]


def _edge_key(left, right):
    return (left, right) if left < right else (right, left)


class NetworkEditor:
    """Editable copy of a project's road graph; nothing is written until save."""

    def __init__(self, project, footprint=None):
        self.project = project
        self.grid = project.keepout.grid
        self.clearance = _Clearance(self.grid)
        self.footprint = tuple(footprint or project.metadata.get("footprint") or FOOTPRINT)
        self.min_width = max(MIN_EDGE_WIDTH, 2. * self.footprint[2] + .02)
        self.nodes = {node.id: (float(node.x), float(node.y)) for node in project.network}
        self.edges = {_edge_key(source, target)
                      for source, target, _metres in project.network.edges()}
        self.next_id = max(self.nodes) + 1 if self.nodes else 0
        self.widths = {_edge_key(int(key.split(":")[0]), int(key.split(":")[1])): float(width)
                       for key, width in (project.metadata.get("edge_widths") or {}).items()}

    # -- lookup -----------------------------------------------------------------
    def node_near(self, x, y, tolerance=.12):
        best, nearest = None, tolerance
        for node_id, (node_x, node_y) in self.nodes.items():
            space = math.hypot(node_x - x, node_y - y)
            if space <= nearest:
                best, nearest = node_id, space
        return best

    def edge_near(self, x, y, tolerance=.10):
        """Closest existing edge to a point, for the delete-edge button."""
        best, nearest = None, tolerance
        for left, right in self.edges:
            ax, ay = self.nodes[left]
            bx, by = self.nodes[right]
            length = math.hypot(bx - ax, by - ay)
            if length <= 0:
                continue
            t = max(0.0, min(1.0, ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / (length * length)))
            space = math.hypot(ax + (bx - ax) * t - x, ay + (by - ay) * t - y)
            if space <= nearest:
                best, nearest = (left, right), space
        return best

    # -- edits ------------------------------------------------------------------
    def clear(self):
        """Drop every node and edge, so a network can be rebuilt from scratch.

        The caller must still press save; until then nothing on disk changes and
        the window can reload the project to get the old graph back.
        """
        self.nodes.clear()
        self.edges.clear()
        self.widths.clear()
        self.next_id = 0
        return "网络已清空；从现在开始放节点连线。"

    def add_node(self, x, y):
        clear = self.clearance.at(x, y)
        if clear < self.footprint[2] + .01:
            return None, "这里离障碍太近（余量 %.2f m，车身半宽 %.2f m），请点在道路中间。" % (
                max(0.0, clear), self.footprint[2])
        if self.node_near(x, y, .08) is not None:
            return None, "附近 8 cm 已有节点，直接点它即可。"
        node_id = self.next_id
        self.next_id += 1
        self.nodes[node_id] = (float(x), float(y))
        return node_id, ""

    def remove_node(self, node_id):
        if node_id not in self.nodes:
            return "没有这个节点"
        self.nodes.pop(node_id)
        self.edges = {edge for edge in self.edges if node_id not in edge}
        return ""

    def remove_edge(self, left, right):
        key = _edge_key(left, right)
        if key not in self.edges:
            return "这两个节点之间没有连线"
        self.edges.discard(key)
        return ""

    def connect(self, left, right):
        if left == right:
            return False, "请选择两个不同的节点"
        missing = [node_id for node_id in (left, right) if node_id not in self.nodes]
        if missing:
            return False, "节点 %d 不存在" % missing[0]
        key = _edge_key(left, right)
        if key in self.edges:
            return False, "这两个节点已经连过"
        ax, ay = self.nodes[left]
        bx, by = self.nodes[right]
        length = math.hypot(bx - ax, by - ay)
        if length > LONGEST_EDGE:
            return False, "单段 %.1f m 太长，请在中间补一个节点" % length
        if length < .02:
            return False, "两点几乎重合"
        yaw = math.atan2(by - ay, bx - ax)
        samples = max(2, int(length / max(.01, self.grid["resolution"] * .5)) + 1)
        width = 0.0
        for index in range(samples):
            t = index / (samples - 1)
            clear = self.clearance.at(ax + (bx - ax) * t, ay + (by - ay) * t)
            if clear < 0:
                return False, "这段落在未知区域或地图外"
            width = 2 * clear if width <= 0 else min(width, 2 * clear)
        if width < self.min_width:
            return False, ("这段最窄处只有 %.2f m，低于 %.2f m；请把节点往路中间挪，"
                           "或改走更宽的走廊。" % (width, self.min_width))
        if not swept_clear(self.grid, ((ax, ay, yaw), (bx, by, yaw)), self.footprint, BLOCKED):
            return False, "整条车身扫掠过不去（%.2f × %.2f m）；请换条路。" % (
                self.footprint[0] + self.footprint[1], self.footprint[2] * 2)
        self.edges.add(key)
        self.widths[key] = width
        return True, "已连边 %d–%d，最窄 %.2f m。" % (left, right, width)

    # -- output -----------------------------------------------------------------
    def network(self):
        links = {node_id: set() for node_id in self.nodes}
        for left, right in self.edges:
            links[left].add(right)
            links[right].add(left)
        return RoadNetwork([RoadNode(node_id, x, y, to=tuple(sorted(links[node_id])),
                                     name="路口%d" % node_id)
                            for node_id, (x, y) in sorted(self.nodes.items())])

    def edge_widths(self):
        widths = {}
        for left, right in self.edges:
            value = round(max(self.min_width, self.widths.get((left, right), self.min_width)), 3)
            widths["%d:%d" % (left, right)] = value
            widths["%d:%d" % (right, left)] = value
        return widths

    def save(self):
        if not self.edges:
            raise ValueError("还没有连线；先放两个节点再点它们连线。")
        self.project = save_network(self.project.path, self.network(), self.edge_widths(), self.footprint)
        return self.project
