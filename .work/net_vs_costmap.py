"""Check the drawn road network against the live global costmap (read-only)."""
import glob
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from nav2_msgs.srv import GetCostmap
from rclpy.qos import DurabilityPolicy, QoSProfile

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
node = rclpy.create_node('carapp_netcheck')
client = node.create_client(GetCostmap, '/global_costmap/get_costmap')
print('service', client.wait_for_service(timeout_sec=8.))
future = client.call_async(GetCostmap.Request())
end = time.monotonic() + 10
while not future.done() and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
if not future.done():
    print('costmap timeout')
    sys.exit(1)
cost = future.result().map
meta = cost.metadata
o = meta.origin
grid = dict(width=meta.size_x, height=meta.size_y, resolution=meta.resolution,
            origin=(o.position.x, o.position.y, yaw_of(o.orientation)), data=cost.data)
print('costmap %dx%d res %.3f origin %.2f %.2f frame %s cells %d' % (
    meta.size_x, meta.size_y, meta.resolution, o.position.x, o.position.y,
    cost.header.frame_id, len(cost.data)))
pos = {int(i): p for i, p, _ in NODES}
links = {int(i): [int(t) for t in to] for i, _, to in NODES}
bad = []
for src, targets in links.items():
    for dst in targets:
        if dst < src:
            continue
        a, b = pos[src], pos[dst]
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        distance = math.dist(a, b)
        count = max(1, math.ceil(distance / .02))
        for i in range(count + 1):
            t = i / count
            pose = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, yaw)
            if not pose_clear(grid, pose):
                bad.append((src, dst, round(distance, 2), (round(pose[0], 2), round(pose[1], 2))))
                break
print('edges', sum(len(v) for v in links.values()) // 2, 'blocked', len(bad))
for row in bad[:20]:
    print('blocked edge %d-%d len %.2f at %s' % row)
node.destroy_node()
rclpy.shutdown()
