import time
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState

rclpy.init(args=["--ros-args", "--disable-external-lib-logs"])
node = rclpy.create_node("carapp_ready_diag2")
seen = {"scan": 0.0, "odom": 0.0}
node.create_subscription(LaserScan, "/scan_raw", lambda msg: seen.__setitem__("scan", time.monotonic()), qos_profile_sensor_data)
node.create_subscription(Odometry, "/odom", lambda msg: seen.__setitem__("odom", time.monotonic()), 10)
planner = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
navigator = ActionClient(node, NavigateToPose, "/navigate_to_pose")
names = ["map_server", "amcl", "controller_server", "planner_server",
         "smoother_server", "behavior_server", "bt_navigator", "velocity_smoother",
         "sandbox_mask_server", "sandbox_filter_info_server"]
clients = {name: node.create_client(GetState, "/" + name + "/get_state") for name in names}
end = time.monotonic() + 5
while time.monotonic() < end and not (seen["scan"] and seen["odom"]):
    rclpy.spin_once(node, timeout_sec=0.05)
print("scan_age %.2f odom_age %.2f" % (time.monotonic() - seen["scan"], time.monotonic() - seen["odom"]))
print("planner ready", planner.server_is_ready(), "navigator ready", navigator.server_is_ready())
planner.wait_for_server(timeout_sec=3.0)
navigator.wait_for_server(timeout_sec=3.0)
print("after wait planner", planner.server_is_ready(), "navigator", navigator.server_is_ready())
states = {}
futures = {}
for name, client in clients.items():
    if not client.wait_for_service(timeout_sec=1.0):
        states[name] = "NO_SERVICE"
        continue
    futures[name] = client.call_async(GetState.Request())
deadline = time.monotonic() + 3
while not all(f.done() for f in futures.values()) and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
for name, future in futures.items():
    try:
        states[name] = future.result().current_state.id if future.done() else "TIMEOUT"
    except Exception as exc:
        states[name] = "ERR %s" % exc
print("states", states)
node.destroy_node()
rclpy.shutdown()
