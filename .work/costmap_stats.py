import math
import time
from collections import Counter

import rclpy
from nav2_msgs.srv import GetCostmap

rclpy.init(args=['--ros-args', '--disable-external-lib-logs'])
node = rclpy.create_node('carapp_cmstats')
client = node.create_client(GetCostmap, '/global_costmap/get_costmap')
client.wait_for_service(timeout_sec=8.)
future = client.call_async(GetCostmap.Request())
end = time.monotonic() + 10
while not future.done() and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
cost = future.result().map
meta = cost.metadata
data = list(cost.data)
print('type', type(cost.data), 'len', len(data))
counter = Counter(data)
for value, count in sorted(counter.items())[:6]:
    print('value', value, count)
print('top', sorted(counter.items(), key=lambda kv: -kv[1])[:8])
r = meta.resolution
ox, oy = meta.origin.position.x, meta.origin.position.y


def value_at(x, y):
    col = int((x - ox) / r)
    row = int((y - oy) / r)
    return data[row * meta.size_x + col], (col, row)


for point in [(-0.9713, -1.2488), (-1.05, -1.46), (2.64, -0.9), (-2.0, 0.5), (0.0, 0.0)]:
    print(point, value_at(*point))
node.destroy_node()
rclpy.shutdown()
