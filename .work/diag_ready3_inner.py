import time
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState

rclpy.init(args=["--ros-args", "--disable-external-lib-logs"])
node = rclpy.create_node("carapp_ready_diag3")
seen = {"scan": 0.0, "odom": 0.0}
node.create_subscription(LaserScan, "/scan_raw", lambda msg: seen.__setitem__("scan", time.monotonic()), qos_profile_sensor_data)
node.create_subscription(Odometry, "/odom", lambda msg: seen.__setitem__("odom", time.monotonic()), 10)
planner = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
navigator = ActionClient(node, NavigateToPose, "/navigate_to_pose")
names = ["map_server", "amcl", "controller_server", "planner_server",
         "smoother_server", "behavior_server", "bt_navigator", "velocity_smoother",
         "sandbox_mask_server", "sandbox_filter_info_server"]
clients = {name: node.create_client(GetState, "/" + name + "/get_state") for name in names}
futures = {}
states = {}
t0 = time.monotonic()
while time.monotonic() - t0 < 10:
    rclpy.spin_once(node, timeout_sec=0.05)
    for name, client in clients.items():
        old = futures.get(name)
        if old is not None and not old.done():
            continue
        if not client.service_is_ready():
            states[name] = (0, time.monotonic())
            continue
        futures[name] = client.call_async(GetState.Request())
        old = futures[name]
        old.add_done_callback(lambda f, name=name: states.__setitem__(name, (f.result().current_state.id, time.monotonic())))
now = time.monotonic()
print("scan_age %.2f odom_age %.2f" % (now - seen["scan"], now - seen["odom"]))
print("planner", planner.server_is_ready(), "navigator", navigator.server_is_ready())
for name in names:
    state, at = states.get(name, (0, 0.0))
    print("%-30s state=%s age=%.2f" % (name, state, now - at))
print("all ok", all(states.get(n, (0, 0))[0] == 3 and now - states.get(n, (0, 0))[1] < 3 for n in names))
node.destroy_node()
rclpy.shutdown()
