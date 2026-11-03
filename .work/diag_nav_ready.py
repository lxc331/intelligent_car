# -*- coding: utf-8 -*-
"""Ask the car why navigation is not ready: lifecycle states and action servers."""
import paramiko

CAR = "192.168.167.136"
SCRIPT = r'''
import time, rclpy
from lifecycle_msgs.srv import GetState
rclpy.init(args=["--ros-args", "--disable-external-lib-logs"])
node = rclpy.create_node("carapp_ready_diag")
names = ["map_server", "amcl", "controller_server", "planner_server",
         "smoother_server", "behavior_server", "bt_navigator", "velocity_smoother",
         "sandbox_mask_server", "sandbox_filter_info_server"]
clients = {name: node.create_client(GetState, "/" + name + "/get_state") for name in names}
end = time.monotonic() + 6
for name, client in clients.items():
    while not client.wait_for_service(timeout_sec=0.2) and time.monotonic() < end:
        pass
    if not client.service_is_ready():
        print(name, "NO_SERVICE")
        continue
    future = client.call_async(GetState.Request())
    deadline = time.monotonic() + 2
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    print(name, future.result().current_state.id if future.done() and future.result() else "TIMEOUT")
for topic in ("/scan_raw", "/odom", "/amcl_pose", "/map"):
    print("pubs", topic, node.count_publishers(topic))
for action in ("/compute_path_to_pose", "/navigate_to_pose", "/follow_path"):
    print("action", action, node.count_clients(action))
node.destroy_node()
rclpy.shutdown()
'''


def dexec(c, inner, timeout=120):
    command = "sudo -n docker exec -u ubuntu ClawPi bash -lc %s" % repr(inner)
    o = c.exec_command(command, timeout=timeout)
    return (o[1].read().decode("utf-8", "replace"), o[2].read().decode("utf-8", "replace"))


def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(CAR, username="pi", password="raspberrypi", timeout=10)
    try:
        out, err = dexec(c, "source /opt/ros/humble/setup.bash; python3 -c %s" % repr(SCRIPT), 120)
        print(out)
        if err:
            print("--stderr--")
            print(err[-2000:])
    finally:
        c.close()


if __name__ == "__main__":
    main()
