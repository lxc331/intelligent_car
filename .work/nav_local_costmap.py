# -*- coding: utf-8 -*-
"""Runs inside the car container: inspect local costmap around the current pose."""
import math
import time

import rclpy
from nav2_msgs.srv import GetCostmap
from rclpy.node import Node

rclpy.init(args=None)
node = Node('costmap_peek')
client = node.create_client(GetCostmap, '/local_costmap/get_costmap')
if not client.wait_for_service(timeout_sec=8.0):
    print('no get_costmap service')
    raise SystemExit
future = client.call_async(GetCostmap.Request())
start = time.time()
while not future.done() and time.time() - start < 8.0:
    rclpy.spin_once(node, timeout_sec=0.1)
result = future.result()
if result is None:
    print('no result')
    raise SystemExit
grid = result.map
info = grid.metadata
data = list(grid.data)
res = info.resolution
ox, oy = info.origin.position.x, info.origin.position.y
print('costmap', info.size_x, 'x', info.size_y, '@', res, 'origin', round(ox, 3), round(oy, 3))


def value(x, y):
    gx = int(math.floor((x - ox) / res))
    gy = int(math.floor((y - oy) / res))
    if 0 <= gx < info.size_x and 0 <= gy < info.size_y:
        return data[gy * info.size_x + gx]
    return None


# The stuck pose reported by the session state.
pose = (-0.3557, -2.0535)
for radius in (0.05, 0.12, 0.2, 0.3):
    ring = []
    for step in range(24):
        angle = step * math.pi / 12
        value_at = value(pose[0] + radius * math.cos(angle), pose[1] + radius * math.sin(angle))
        ring.append(value_at)
    print('ring %.2f m' % radius, ring)
counts = {}
for row in data:
    counts[row] = counts.get(row, 0) + 1
print('local cost histogram top', sorted(counts.items(), key=lambda item: -item[1])[:6])
node.destroy_node()
rclpy.shutdown()
