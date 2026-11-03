"""Classify drawn network edges by live-costmap clearance and report components."""
import json
import math
import time
from pathlib import Path

import rclpy
from nav2_msgs.srv import GetCostmap

NODES = json.loads(Path('/tmp/net_nodes.json').read_text())


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose_clear(grid, pose, footprint=(.25, .15, .115), threshold=253):
    x, y, yaw = pose
    ox, oy, oa = grid['origin']
    r = grid['resolution']
    c, s = math.cos(oa), math.sin(oa)
    dx, dy = x - ox, y - oy
    x, y = (c * dx + s * dy) / r, (-s * dx + c * dy) / r
    yaw -= oa
    front, rear, half = footprint
    a, b = (front + rear) / (2 * r), half / r
    c, s = math.cos(yaw), math.sin(yaw)
    x += (front - rear) / (2 * r) * c
    y += (front - rear) / (2 * r) * s
    ex, ey = abs(c) * a + abs(s) * b, abs(s) * a + abs(c) * b
    lo_x, hi_x = math.floor(x - ex), math.floor(x + ex)
    lo_y, hi_y = math.floor(y - ey), math.floor(y + ey)
    if lo_x < 0 or lo_y < 0 or hi_x >= grid['width'] or hi_y >= grid['height']:
        return False
    for row in range(lo_y, hi_y + 1):
        for col in range(lo_x, hi_x + 1):
            value = grid['data'][row * grid['width'] + col]
            if 0 <= value < threshold:
                continue
            dx, dy = col + .5 - x, row + .5 - y
            if abs(c * dx + s * dy) <= a + .5 * (abs(c) + abs(s)) and abs(-s * dx + c * dy) <= b + .5 * (abs(c) + abs(s)):
                return False
    return True


rclpy.init(args=['--ros-args', '--disable-external-lib-logs'])
node = rclpy.create_node('carapp_edgeclass')
client = node.create_client(GetCostmap, '/global_costmap/get_costmap')
client.wait_for_service(timeout_sec=8.)
future = client.call_async(GetCostmap.Request())
end = time.monotonic() + 10
while not future.done() and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
cost = future.result().map
m = cost.metadata
o = m.origin
R = m.resolution
grid = dict(width=m.size_x, height=m.size_y, resolution=R,
            origin=(o.position.x, o.position.y, yaw_of(o.orientation)), data=cost.data)
pos = {int(i): p for i, p, _ in NODES}
links = {int(i): [int(t) for t in to] for i, _, to in NODES}
edges = sorted({(min(a, b), max(a, b)) for a, ts in links.items() for b in ts})


def edge_clear(a, b):
    p, q = pos[a], pos[b]
    yaw = math.atan2(q[1] - p[1], q[0] - p[0])
    distance = math.dist(p, q)
    count = max(1, math.ceil(distance / .02))
    for i in range(count + 1):
        t = i / count
        if not pose_clear(grid, (p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t, yaw)):
            return False
    return True


ok, bad = [], []
for a, b in edges:
    (ok if edge_clear(a, b) else bad).append((a, b))
adj = {}
for a, b in ok:
    adj.setdefault(a, set()).add(b)
    adj.setdefault(b, set()).add(a)
seen = set()
comps = []
for start in adj:
    if start in seen:
        continue
    stack, comp = [start], []
    seen.add(start)
    while stack:
        cur = stack.pop()
        comp.append(cur)
        for nxt in adj[cur]:
            if nxt not in seen:
                seen.add(nxt)
                stack.append(nxt)
    comps.append(sorted(comp))
comps.sort(key=len, reverse=True)
print('edges total %d ok %d bad %d' % (len(edges), len(ok), len(bad)))
print('components', [len(c) for c in comps][:8])
print('bad edges', ['%d-%d' % e for e in bad])
print('ok edge count per comp', len(comps[0]) if comps else 0)
node.destroy_node()
rclpy.shutdown()
