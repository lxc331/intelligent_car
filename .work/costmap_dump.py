"""Dump the live global costmap to /tmp/carapp_costmap.pgm + meta json (read-only)."""
import json
import time
from array import array

import rclpy
from nav2_msgs.srv import GetCostmap

rclpy.init(args=['--ros-args', '--disable-external-lib-logs'])
node = rclpy.create_node('carapp_cmdump')
client = node.create_client(GetCostmap, '/global_costmap/get_costmap')
client.wait_for_service(timeout_sec=8.)
future = client.call_async(GetCostmap.Request())
end = time.monotonic() + 10
while not future.done() and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
cost = future.result().map
m = cost.metadata
o = m.origin
data = list(cost.data)
pixels = bytearray(len(data))
for i, value in enumerate(data):
    pixels[i] = 255 - value if 0 <= value <= 252 else (0 if value in (253, 254) else 128)
header = ('P5\n%d %d\n255\n' % (m.size_x, m.size_y)).encode()
with open('/tmp/carapp_costmap.pgm', 'wb') as stream:
    stream.write(header)
    stream.write(bytes(pixels))
meta = dict(width=m.size_x, height=m.size_y, resolution=m.resolution,
            origin=[o.position.x, o.position.y], frame=cost.header.frame_id)
with open('/tmp/carapp_costmap.json', 'w') as stream:
    json.dump(meta, stream)
print(json.dumps(meta))
node.destroy_node()
rclpy.shutdown()
