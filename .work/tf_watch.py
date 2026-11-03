# -*- coding: utf-8 -*-
"""Wait for map frame, print the TF chain and the latest AMCL pose."""
import json
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node("tf_watch")
buffer = Buffer()
TransformListener(buffer, node)
end = time.monotonic() + 10.
seen = {}
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=.05)
    for target, source in (("map", "odom"), ("map", "base_footprint"), ("odom", "base_footprint")):
        if (target, source) in seen:
            continue
        try:
            transform = buffer.lookup_transform(target, source, rclpy.time.Time()).transform
            seen[(target, source)] = [round(transform.translation.x, 3),
                                      round(transform.translation.y, 3)]
        except Exception:
            pass
print(json.dumps({"%s<-%s" % key: value for key, value in seen.items()}), flush=True)
node.destroy_node()
rclpy.shutdown()
