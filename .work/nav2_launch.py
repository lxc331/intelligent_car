"""Launch the vendor Nav2 bringup (map_server + amcl + planner + controller) inside the car container.

Usage: py -3 .work/nav2_launch.py [map_name] [--status]
Read-only probe by default: the launch is detached and its log is tailed.
"""
import sys
import time

import paramiko

from carapp.core.credentials import local_password

HOST = "192.168.167.118"
MAPS = "/home/ubuntu/ros2_ws/src/slam/maps"
NAV = "/home/ubuntu/ros2_ws/src/navigation"
ENV = (
    "source /opt/ros/humble/setup.bash; "
    "source /home/ubuntu/ros2_ws/install/setup.bash; "
    "export need_compile=True; "
)


def connect():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(HOST, 22, "pi", local_password(), timeout=8,
                   banner_timeout=8, auth_timeout=8)
    return client


def run(client, cmd, limit=3000, timeout=120):
    _, stdout, stderr = client.exec_command(
        "sudo -n docker exec -u ubuntu ClawPi bash -lc %r" % cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", "replace")
    err = stderr.read().decode("utf-8", "replace")
    text = out + ("\n[stderr] " + err if err.strip() else "")
    return text.strip()[:limit]


def run_detached(client, cmd):
    _, stdout, stderr = client.exec_command(
        "sudo -n docker exec -d -u ubuntu ClawPi bash -lc %r" % cmd, timeout=60)
    out = stdout.read().decode("utf-8", "replace")
    err = stderr.read().decode("utf-8", "replace")
    return (out + ("\n[stderr] " + err if err.strip() else "")).strip()[:500]


def launch(client, map_name):
    inner = ENV + (
        "ros2 launch %s/launch/include/bringup.launch.py "
        "map:=%s/%s.yaml params_file:=%s/config/nav2_params.yaml "
        "use_sim_time:=false autostart:=true use_namespace:=false namespace:=/ "
        "rtabmap:=false use_teb:=true > /tmp/nav2.log 2>&1"
        % (NAV, MAPS, map_name, NAV)
    )
    return run_detached(client, inner)


def status(client):
    print("=== nav2.log tail ===")
    print(run(client, "tail -30 /tmp/nav2.log", limit=3000))
    print("=== lifecycle nodes ===")
    print(run(client, ENV + "timeout 15 ros2 node list 2>/dev/null", limit=2500))
    print("=== /map and /amcl_pose ===")
    print(run(client, ENV + "timeout 15 ros2 topic list 2>/dev/null | grep -E 'map|amcl|plan|cmd_vel'", limit=1500))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flag = sys.argv[1] if len(sys.argv) > 1 else ""
    client = connect()
    try:
        if flag == "--status":
            status(client)
            return
        map_name = args[0] if args else "map_02"
        print("launching nav2 with %s" % map_name)
        print(launch(client, map_name) or "(detached)")
        time.sleep(20)
        status(client)
    finally:
        client.close()


if __name__ == "__main__":
    main()
