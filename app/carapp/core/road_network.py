"""Known road network: waypoint graph in map metres, no Tk / ROS / network.

The vendor ships the same on-disk shape in ``road_network/config/*.yaml``::

    waypoints:
      - id: 0
        pose: [0.0, 0.0, 0.0]      # x, y, yaw（yaw 可省略，省略时用行进方向）
        to: [1, 2]                 # 有向邻接，双向需写两次
        name: 仓库                  # 可选地标名

So vendor files load unchanged.  Planning differs on purpose: weighted
Dijkstra over edge metres (shortest distance) instead of fewest hops, and the
dense path is produced here so the caller can preview it without a car.

Everything in this module is pure Python and covered by offline tests.
"""
from dataclasses import dataclass
import heapq
import math
from pathlib import Path

from carapp.core.map_data import world_to_grid


MAX_YAML_BYTES = 65_536
MAX_NODES = 500
MAX_EDGES = 5_000
MAX_ID = 99_999
MAX_NAME = 40
MAX_COORD = 1e5
DEFAULT_STEP = .05
MIN_STEP = .01
MAX_STEP = 1.
MAX_ENTRY_DISTANCE = 1.


@dataclass(frozen=True)
class RoadNode:
    id: int
    x: float
    y: float
    yaw: float = None
    to: tuple = ()
    name: str = None

    @property
    def pose(self):
        return (self.x, self.y, self.yaw)


class RoadNetwork:
    """Directed waypoint graph plus the dense-path helpers navigation needs."""

    def __init__(self, nodes, source_path=None):
        ordered = sorted(nodes, key=lambda node: node.id)
        self.nodes = {node.id: node for node in ordered}
        self.source_path = Path(source_path) if source_path is not None else None
        if len(self.nodes) != len(ordered):
            raise ValueError("路网节点 id 重复")

    def __len__(self):
        return len(self.nodes)

    def __iter__(self):
        return iter(self.nodes.values())

    def ids(self):
        return tuple(self.nodes)

    def node(self, node_id):
        try:
            return self.nodes[node_id]
        except KeyError:
            raise ValueError("路网没有节点 %s" % node_id) from None

    def vertices(self):
        return tuple((node.id, node.x, node.y, node.yaw, node.name)
                     for node in self.nodes.values())

    def edges(self):
        """Directed edges as ``(source, target, metres)``."""
        result = []
        for node in self.nodes.values():
            for target in node.to:
                other = self.nodes[target]
                result.append((node.id, target,
                               math.hypot(other.x - node.x, other.y - node.y)))
        return tuple(result)

    def length(self, source, target):
        first, second = self.node(source), self.node(target)
        return math.hypot(second.x - first.x, second.y - first.y)

    def shortest_path(self, start, goal):
        """Shortest node sequence by travelled metres; ties break by lower id.

        Raises ValueError when the nodes exist but no directed route links them.
        """
        self.node(start)
        self.node(goal)
        if start == goal:
            return (start,)
        best = {start: 0.}
        previous = {}
        queue = [(0., start)]
        done = set()
        while queue:
            distance, node_id = heapq.heappop(queue)
            if node_id in done:
                continue
            done.add(node_id)
            if node_id == goal:
                break
            for target in self.node(node_id).to:
                if target in done:
                    continue
                candidate = distance + self.length(node_id, target)
                known = best.get(target)
                if known is None or (candidate, node_id) < (known, previous.get(target, target)):
                    best[target] = candidate
                    previous[target] = node_id
                    heapq.heappush(queue, (candidate, target))
        if goal not in best:
            raise ValueError("路网中节点 %d 到 %d 没有可通行的方向边" % (start, goal))
        sequence = [goal]
        while sequence[-1] != start:
            sequence.append(previous[sequence[-1]])
        return tuple(reversed(sequence))

    def path_length(self, ids):
        ids = tuple(ids)
        return sum(self.length(source, target) for source, target in zip(ids, ids[1:]))

    def nearest_node(self, x, y):
        """Closest node as ``(node, metres)``; ties break by lower id."""
        _finite(x, "x")
        _finite(y, "y")
        ordered = sorted(self.nodes.values(),
                         key=lambda node: (math.hypot(node.x - x, node.y - y), node.id))
        if not ordered:  # pragma: no cover - constructor rejects empty networks
            raise ValueError("路网没有节点")
        node = ordered[0]
        return node, math.hypot(node.x - x, node.y - y)

    def entry_node(self, x, y, max_distance=MAX_ENTRY_DISTANCE):
        """Node used to join the network from a pose, or None when too far."""
        limit = _finite(max_distance, "max_distance")
        if limit <= 0.:
            raise ValueError("路网接入距离需要正数")
        node, distance = self.nearest_node(x, y)
        return node if distance <= limit else None

    def dense_path(self, ids, step=DEFAULT_STEP):
        """Interpolated ``(x, y, yaw)`` points along consecutive node ids.

        Intermediate headings follow the travel direction; a node that carries
        an explicit yaw keeps it as the last heading of its incoming segment
        (the vendor behaviour), the first point uses its own yaw when present.
        """
        step = _step(step)
        ids = tuple(ids)
        if not ids:
            raise ValueError("路网路径需要至少一个节点")
        for node_id in ids:
            self.node(node_id)
        for source, target in zip(ids, ids[1:]):
            if target not in self.node(source).to:
                raise ValueError("路网节点 %d 到 %d 之间没有边" % (source, target))
        first = self.node(ids[0])
        if len(ids) == 1:
            return ((first.x, first.y, 0. if first.yaw is None else first.yaw),)
        points = []
        for index, node_id in enumerate(ids):
            node = self.node(node_id)
            if index == 0:
                heading = node.yaw
                if heading is None:
                    following = self.node(ids[1])
                    heading = math.atan2(following.y - node.y, following.x - node.x)
                points.append((node.x, node.y, heading))
                continue
            previous = self.node(ids[index - 1])
            dx, dy = node.x - previous.x, node.y - previous.y
            distance = math.hypot(dx, dy)
            heading = math.atan2(dy, dx) if distance else (previous.yaw or 0.)
            count = max(1, int(math.ceil(distance / step)))
            for sample in range(1, count + 1):
                ratio = sample / count
                points.append((previous.x + dx * ratio, previous.y + dy * ratio, heading))
            if node.yaw is not None and index == len(ids) - 1:
                points[-1] = (points[-1][0], points[-1][1], node.yaw)
        return tuple(points)

    def check_map(self, grid):
        """Map-consistency problems: nodes outside free space, edges crossing it.

        Returns a tuple of human readable warnings; an empty tuple means the
        network only touches known-free cells.  Inflation is a costmap concern,
        so grazing a wall still warns instead of failing.
        """
        problems = []
        width, height = int(grid["width"]), int(grid["height"])
        resolution = float(grid["resolution"])
        data = grid["data"]

        def cell(x, y):
            gx, gy = world_to_grid(grid, x, y)
            column, row = math.floor(gx), math.floor(gy)
            if not (0 <= column < width and 0 <= row < height):
                return None
            return data[row * width + column]

        for node in self.nodes.values():
            value = cell(node.x, node.y)
            if value is None:
                problems.append("节点 %d 在地图范围外" % node.id)
            elif value == 100:
                problems.append("节点 %d 落在障碍上" % node.id)
            elif value != 0:
                problems.append("节点 %d 落在未知区域" % node.id)
        sample = max(resolution * .5, .02)
        checked = set()
        for node in self.nodes.values():
            for target in node.to:
                if (node.id, target) in checked:
                    continue
                checked.add((node.id, target))
                other = self.nodes[target]
                distance = math.hypot(other.x - node.x, other.y - node.y)
                count = max(1, int(math.ceil(distance / sample)))
                bad = None
                for index in range(count + 1):
                    ratio = index / count
                    value = cell(node.x + (other.x - node.x) * ratio,
                                 node.y + (other.y - node.y) * ratio)
                    if value is None:
                        bad = 0
                        break
                    if value == 100:
                        bad = 100
                        break
                    if value != 0 and bad is None:
                        bad = value
                if bad == 0:
                    problems.append("边 %d→%d 穿出地图范围" % (node.id, target))
                elif bad == 100:
                    problems.append("边 %d→%d 穿过障碍" % (node.id, target))
                elif bad is not None:
                    problems.append("边 %d→%d 穿过未知区域" % (node.id, target))
        return tuple(problems)

    def text(self):
        """Vendor-compatible YAML text; ``load_road_network(text())`` round-trips."""
        lines = ["waypoints:"]
        for node in self.nodes.values():
            pose = [_short(node.x), _short(node.y)]
            if node.yaw is not None:
                pose.append(_short(node.yaw))
            lines.append("  - id: %d" % node.id)
            lines.append("    pose: [%s]" % ", ".join(pose))
            lines.append("    to: [%s]" % ", ".join(str(target) for target in node.to))
            if node.name is not None:
                lines.append("    name: %s" % node.name)
        return "\n".join(lines) + "\n"

    def save(self, path):
        target = Path(path).expanduser()
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(self.text(), encoding="utf-8", newline="\n")
        return target


def find_road_network(map_yaml_path, name="road_network.yaml"):
    """``<map dir>/road_network.yaml`` next to the imported map, or None."""
    path = Path(map_yaml_path).expanduser().resolve(strict=False)
    candidate = path.parent / name
    return candidate if candidate.is_file() else None


def load_road_network(path):
    source = Path(path).expanduser().resolve(strict=True)
    with source.open("rb") as stream:
        raw = stream.read(MAX_YAML_BYTES + 1)
    if len(raw) > MAX_YAML_BYTES:
        raise ValueError("路网 YAML 文件过大（最大 64 KB）")
    try:
        text = raw.decode("utf-8-sig")
    except UnicodeError:
        raise ValueError("路网 YAML 需要 UTF-8 编码") from None
    return parse_road_network(text, source)


def parse_road_network(text, source_path=None):
    try:
        import yaml
    except ImportError as exc:
        raise ValueError("路网导入需要 PyYAML，请安装 app/requirements.txt 中的依赖") from exc

    class UniqueLoader(yaml.SafeLoader):
        pass

    def unique_mapping(loader, node, deep=False):
        values = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=deep)
            if not isinstance(key, str) or key in values:
                raise ValueError("路网 YAML 含重复或无效字段")
            values[key] = loader.construct_object(value_node, deep=deep)
        return values

    UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)
    try:
        document = yaml.load(text, Loader=UniqueLoader)
    except yaml.YAMLError as exc:
        raise ValueError("路网 YAML 格式无效") from exc
    if not isinstance(document, dict):
        raise ValueError("路网 YAML 需要字段映射")
    unknown = set(document) - {"waypoints", "nodes"}
    if unknown:
        raise ValueError("路网 YAML 含未知字段：" + ", ".join(sorted(unknown)))
    entries = document.get("waypoints", document.get("nodes"))
    if entries is None:
        raise ValueError("路网 YAML 缺少 waypoints 列表")
    if not isinstance(entries, list) or not entries:
        raise ValueError("路网 waypoints 需要非空列表")
    if len(entries) > MAX_NODES:
        raise ValueError("路网节点过多（最多 %d 个）" % MAX_NODES)
    nodes = [_parse_node(entry, index) for index, entry in enumerate(entries)]
    network = RoadNetwork(nodes, source_path)
    edges = 0
    for node in network:
        for target in node.to:
            if target not in network.nodes:
                raise ValueError("节点 %d 的邻接目标 %d 不存在" % (node.id, target))
            edges += 1
    if edges > MAX_EDGES:
        raise ValueError("路网边过多（最多 %d 条）" % MAX_EDGES)
    return network


def _parse_node(entry, index):
    if not isinstance(entry, dict):
        raise ValueError("路网第 %d 个节点需要字段映射" % (index + 1))
    unknown = set(entry) - {"id", "pose", "to", "name"}
    if unknown:
        raise ValueError("路网节点含未知字段：" + ", ".join(sorted(unknown)))
    node_id = entry.get("id")
    if isinstance(node_id, bool) or not isinstance(node_id, int) or not 0 <= node_id <= MAX_ID:
        raise ValueError("路网第 %d 个节点 id 需要 0–%d 的整数" % (index + 1, MAX_ID))
    pose = entry.get("pose")
    if not isinstance(pose, (list, tuple)) or not 2 <= len(pose) <= 3:
        raise ValueError("节点 %d 的 pose 需要 [x, y] 或 [x, y, yaw]" % node_id)
    x = _finite(pose[0], "pose x")
    y = _finite(pose[1], "pose y")
    if abs(x) > MAX_COORD or abs(y) > MAX_COORD:
        raise ValueError("节点 %d 的坐标超出范围" % node_id)
    yaw = None if len(pose) == 2 else _finite(pose[2], "pose yaw")
    targets = entry.get("to", [])
    if not isinstance(targets, (list, tuple)):
        raise ValueError("节点 %d 的 to 需要 id 列表" % node_id)
    seen = set()
    ordered = []
    for target in targets:
        if isinstance(target, bool) or not isinstance(target, int):
            raise ValueError("节点 %d 的邻接目标需要整数 id" % node_id)
        if target == node_id:
            raise ValueError("节点 %d 不能连接自己" % node_id)
        if target in seen:
            raise ValueError("节点 %d 的邻接目标 %d 重复" % (node_id, target))
        seen.add(target)
        ordered.append(target)
    name = entry.get("name")
    if name is not None:
        if not isinstance(name, str) or not name.strip() or len(name.strip()) > MAX_NAME:
            raise ValueError("节点 %d 的 name 需要 1–%d 个字符" % (node_id, MAX_NAME))
        name = name.strip()
    return RoadNode(node_id, x, y, yaw, tuple(ordered), name)


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("路网 %s 需要有限数值" % label)
    return float(value)


def _step(step):
    value = _finite(step, "插值步长")
    if not MIN_STEP <= value <= MAX_STEP:
        raise ValueError("路网插值步长需在 %.2f–%.1f m" % (MIN_STEP, MAX_STEP))
    return value


def _short(value):
    text = "%.4f" % value
    return text.rstrip("0").rstrip(".") if "." in text else text
