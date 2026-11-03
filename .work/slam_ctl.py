# -*- coding: utf-8 -*-
"""Control the car's vendor SLAM Toolbox headlessly (same launch the APP uses).

    python .work/slam_ctl.py start
    python .work/slam_ctl.py status
    python .work/slam_ctl.py save NAME     # map_saver + serialize, pull to .work/maps/NAME
    python .work/slam_ctl.py stop

The APP starts SLAM through carapp.core.mapping_remote_scripts with
/home/ubuntu/ros2_ws/src/slam/config/slam.yaml; this script mirrors that launch.
"""
import pathlib
import sys
import time

import paramiko

CAR = "192.168.167.136"
USER, PWD = "pi", "raspberrypi"
ROOT = pathlib.Path(__file__).resolve().parent
HOST_DIR = "/home/pi/carapp_slam"
IN_CONTAINER = "/tmp/slam_agent"
LOG = "/tmp/slam_agent.log"
BEGIN = "source /opt/ros/humble/setup.bash; source /home/ubuntu/ros2_ws/install/setup.bash;"
CONFIG = "/home/ubuntu/ros2_ws/src/slam/config/slam.yaml"
NODE = (
    "ros2 run slam_toolbox sync_slam_toolbox_node --ros-args "
    "--params-file %s -r __node:=slam_toolbox -p use_sim_time:=false "
    "-p mode:=mapping -p scan_topic:=/scan_raw -p map_frame:=map "
    "-p odom_frame:=odom -p base_frame:=base_footprint --disable-external-lib-logs"
) % CONFIG


def client():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(CAR, username=USER, password=PWD, timeout=8)
    return c


def run(c, cmd, timeout=60):
    o = c.exec_command(cmd, timeout=timeout)
    return (o[1].read().decode("utf-8", "replace") + o[2].read().decode("utf-8", "replace")).strip()


def dexec(c, inner, timeout=90):
    return run(c, "sudo -n docker exec -u ubuntu ClawPi bash -lc %s" % repr(BEGIN + " " + inner), timeout)


def pids(c):
    text = dexec(c, "pgrep -f sync_slam_toolbox_node || true")
    return [p for p in text.split() if p.isdigit()]


def action_start(c):
    for pid in pids(c):
        dexec(c, "kill -INT %s || true" % pid, 20)
    time.sleep(2)
    dexec(c, "rm -f %s" % LOG)
    run(c, "sudo -n docker exec -d -u ubuntu ClawPi bash -lc %s"
        % repr(BEGIN + " " + NODE + " > %s 2>&1" % LOG), 25)
    time.sleep(6)
    print("pids:", " ".join(pids(c)) or "none")
    print("nodes:", dexec(c, "ros2 node list 2>/dev/null | tr '\\n' ' '")[:300])
    print("map:", dexec(c, "timeout 5 ros2 topic info /map 2>/dev/null | tr '\\n' ' '"))
    print("log:", dexec(c, "tail -6 %s" % LOG)[-700:])


def action_status(c):
    print("pids:", " ".join(pids(c)) or "none")
    print("nodes:", dexec(c, "ros2 node list 2>/dev/null | grep -i slam | tr '\\n' ' '") or "none")
    print("map:", dexec(c, "timeout 6 ros2 topic echo /map --once --field info 2>/dev/null "
                          "| grep -E 'width|height|resolution' | tr '\\n' ' '"))
    print("config:", dexec(c, "ls -l %s 2>&1" % CONFIG))
    print("log:", dexec(c, "tail -4 %s" % LOG)[-500:])


def action_save(c, name):
    dexec(c, "mkdir -p %s" % IN_CONTAINER)
    prefix = IN_CONTAINER + "/map"
    print("map_saver:", dexec(c, "ros2 run nav2_map_server map_saver_cli -f %s --fmt pgm --ros-args "
                                 "-p map_subscribe_transient_local:=true -p save_map_timeout:=10.0 "
                                 "--disable-external-lib-logs 2>&1 | tail -4" % prefix, 90)[-400:])
    script = (
        "import rclpy\n"
        "from slam_toolbox.srv import SerializePoseGraph\n"
        "rclpy.init(args=['--ros-args','--disable-external-lib-logs'])\n"
        "n=rclpy.create_node('agent_serialize')\n"
        "cli=n.create_client(SerializePoseGraph,'/slam_toolbox/serialize_map')\n"
        "ok=cli.wait_for_service(timeout_sec=3)\n"
        "print('service', ok)\n"
        "if ok:\n"
        "    req=SerializePoseGraph.Request(); req.filename='%s'\n"
        "    fut=cli.call_async(req); rclpy.spin_until_future_complete(n,fut,timeout_sec=20)\n"
        "    print('serialize', fut.result())\n"
        "n.destroy_node(); rclpy.shutdown()\n" % prefix
    )
    sftp = c.open_sftp()
    with sftp.open("/home/pi/_serialize.py", "w") as fh:
        fh.write(script)
    sftp.close()
    run(c, "sudo -n docker cp /home/pi/_serialize.py ClawPi:/tmp/_serialize.py")
    print("serialize:", dexec(c, "python3 /tmp/_serialize.py 2>&1 | tail -3", 90)[-300:])
    print(dexec(c, "ls -l %s.pgm %s.yaml %s.posegraph %s.data 2>&1" % (prefix, prefix, prefix, prefix)))
    run(c, "mkdir -p %s; for f in pgm yaml posegraph data; do "
           "sudo -n docker cp ClawPi:%s.$f %s/slam.$f 2>/dev/null; done; "
           "sudo -n chown -R pi:pi %s" % (HOST_DIR, prefix, HOST_DIR, HOST_DIR), 90)
    local = ROOT / "maps" / name
    local.mkdir(parents=True, exist_ok=True)
    sftp = c.open_sftp()
    for f in sftp.listdir(HOST_DIR):
        if not f.startswith("slam."):
            continue
        suffix = f.split(".")[-1]
        sftp.get(HOST_DIR + "/" + f, str(local / ("map." + suffix)))
    sftp.close()
    print("saved:", ", ".join(p.name for p in sorted(local.iterdir())))


def action_stop(c):
    for pid in pids(c):
        dexec(c, "kill -INT %s || true" % pid, 20)
    time.sleep(3)
    for pid in pids(c):
        dexec(c, "kill -KILL %s || true" % pid, 15)
    print("remaining:", " ".join(pids(c)) or "none")


def main():
    op = sys.argv[1] if len(sys.argv) > 1 else "status"
    c = client()
    try:
        if op == "start":
            action_start(c)
        elif op == "status":
            action_status(c)
        elif op == "save":
            action_save(c, sys.argv[2] if len(sys.argv) > 2 else time.strftime("slam_%Y%m%d_%H%M%S"))
        elif op == "stop":
            action_stop(c)
        else:
            print("unknown op", op)
    finally:
        c.close()


if __name__ == "__main__":
    main()
