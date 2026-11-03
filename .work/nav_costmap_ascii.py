# -*- coding: utf-8 -*-
"""Runs inside the car container: ASCII view of the local costmap around the car."""
import math
import time

import rclpy
from nav2_msgs.srv import GetCostmap
from rclpy.node import Node

rclpy.init(args=None)
node = Node('costmap_ascii')
client = node.create_client(GetCostmap, '/local_costmap/get_costmap')
if not client.wait_for_service(timeout_sec=8.0):
    print('no service')
    raise SystemExit
future = client.call_async(GetCostmap.Request())
start = time.time()
while not future.done() and time.time() - start < 8.0:
    rclpy.spin_once(node, timeout_sec=0.1)
grid = future.result().map
info = grid.metadata
data = list(grid.data)
res = info.resolution
ox, oy = info.origin.position.x, info.origin.position.y
pose = (-0.3557, -2.0535)
print('size', info.size_x, info.size_y, 'res', res, 'origin', round(ox, 3), round(oy, 3))
step = 4
for gy in range(info.size_y - 1, -1, -step):
    row = ''
    for gx in range(0, info.size_x, step):
        block = [data[yy * info.size_x + gx]
                 for yy in range(max(0, gy - step + 1), gy + 1)]
        cost = max(block)
        x = ox + (gx + 0.5) * res
        y = oy + (gy + 0.5) * res
        if math.hypot(x - pose[0], y - pose[1]) < 0.02:
            row += '@'
        elif cost >= 254:
            row += '#'
        elif cost >= 253:
            row += '+'
        elif cost >= 200:
            row += ':'
        elif cost >= 100:
            row += '.'
        else:
            row += ' '
    print(row)
node.destroy_node()
rclpy.shutdown()
