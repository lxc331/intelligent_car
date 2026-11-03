# -*- coding: utf-8 -*-
"""Control a Cartographer 2D mapping run on the car (start/status/save/stop/report).

    python carto_ctl.py start      # upload config, start cartographer_node + occupancy grid
    python carto_ctl.py status     # nodes, /map presence, scan rate, log tail
    python carto_ctl.py save NAME  # write pbstream, convert to pgm/yaml, download to PC
    python carto_ctl.py stop       # finish trajectory + stop nodes (keeps pbstream)
"""
import pathlib
import sys
import time

import paramiko

CAR = "192.168.167.136"
USER, PWD = "pi", "raspberrypi"
ROOT = pathlib.Path(__file__).resolve().parent
CFG = ROOT / "cartographer" / "sandbox_2d.lua"
HOST_DIR = "/home/pi/carapp_carto"
CONTAINER_DIR = "/tmp/carto_cfg"
LOG = "/tmp/carto.log"
GRID_LOG = "/tmp/carto_grid.log"

BEGIN = "source /opt/ros/humble/setup.bash;"
NODE = (
    "ros2 run cartographer_ros cartographer_node "
    "-configuration_directory %s -configuration_basename sandbox_2d.lua "
    "--ros-args -r __node:=cartographer_node" % CONTAINER_DIR
)
GRID = (
    "ros2 run cartographer_ros cartographer_occupancy_grid_node "
    "-resolution 0.03 -publish_period_sec 1.0"
)


def client():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(CAR, username=USER, password=PWD, timeout=8)
    return c


def run(c, cmd, timeout=60):
    _, out, err = c.exec_command(cmd, timeout=timeout)
    text = out.read().decode("utf-8", "replace")
    error = err.read().decode("utf-8", "replace")
    return text, error


def dexec(c, inner, timeout=60):
    return run(c, 'sudo -n docker exec -u ubuntu ClawPi bash -lc "%s"' % inner.replace('"', '\\"'), timeout)


def ddetach(c, inner):
    return run(c, 'sudo -n docker exec -d -u ubuntu ClawPi bash -lc "%s"' % inner.replace('"', '\\"'), 20)


def kill_all(c):
    """Kill cartographer processes by pid (pkill -f would kill the wrapper shell too)."""
    text, _ = dexec(c, "pgrep -f cartographer_node || true")
    pids = [p for p in text.split() if p.isdigit()]
    text2, _ = dexec(c, "pgrep -f occupancy_grid_node || true")
    pids += [p for p in text2.split() if p.isdigit()]
    for pid in set(pids):
        dexec(c, "kill -KILL %s || true" % pid, 15)
    time.sleep(1)
    nodes, _ = dexec(c, "ros2 node list | grep -i cartographer || true")
    return nodes.strip()


def action_start(c):
    sftp = c.open_sftp()
    run(c, "mkdir -p %s" % HOST_DIR)
    sftp.put(str(CFG), HOST_DIR + "/sandbox_2d.lua")
    sftp.close()
    text, err = run(c, "sudo -n docker cp %s/. ClawPi:%s; sudo -n docker exec ClawPi ls %s" % (HOST_DIR, CONTAINER_DIR, CONTAINER_DIR), 40)
    print("config deployed:", text.strip() or err.strip()[:200])

    left = kill_all(c)
    if "cartographer" in left:
        print("WARNING still running:", left)
        return

    ddetach(c, BEGIN + " ros2 run cartographer_ros cartographer_node -configuration_directory %s "
                   "-configuration_basename sandbox_2d.lua --ros-args -r scan:=/scan_raw "
                   "> %s 2>&1" % (CONTAINER_DIR, LOG))
    time.sleep(5)
    ddetach(c, BEGIN + " %s > %s 2>&1" % (GRID, GRID_LOG))
    time.sleep(3)
    out, _ = dexec(c, "pgrep -af cartographer")
    print("processes:", out.strip())
    log, _ = dexec(c, "tail -8 %s" % LOG)
    print("carto log:\n" + log)
    glog, _ = dexec(c, "tail -8 %s" % GRID_LOG)
    print("grid log:\n" + glog)


def action_status(c):
    out, _ = dexec(c, "pgrep -af cartographer")
    print("processes:", out.strip() or "none")
    nodes, _ = dexec(c, "ros2 node list | grep -i cartographer")
    print("nodes:", nodes.strip() or "none")
    hz, _ = dexec(c, "timeout 3 ros2 topic hz /map 2>/dev/null | tail -1")
    print("/map hz:", hz.strip() or "none")
    odom, _ = dexec(c, "timeout 3 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null | head -4")
    print("odom:", odom.strip().replace("\n", " "))
    log, _ = dexec(c, "tail -12 %s" % LOG)
    print("carto log tail:\n" + log)
    glog, _ = dexec(c, "tail -6 %s" % GRID_LOG)
    print("grid log tail:\n" + glog)


def action_save(c, name):
    req = '"{filename: \'%s_%s.pbstream\', include_unfinished_submaps: true}"' % ("/tmp/carto", name)
    out, err = dexec(
        c,
        BEGIN + " ros2 service call /write_state cartographer_ros_msgs/srv/WriteState %s" % req,
        90,
    )
    print("write_state:", out.strip()[:600], err.strip()[:300])
    time.sleep(2)
    ls, _ = dexec(c, "ls -l /tmp/carto_%s.pbstream" % name)
    print(ls.strip())
    conv, cerr = dexec(
        c,
        BEGIN + " ros2 run cartographer_ros cartographer_pbstream_to_ros_map "
        "-pbstream_filename /tmp/carto_%s.pbstream -map_filestem /tmp/carto_%s" % (name, name),
        120,
    )
    print("convert:", conv.strip()[:600], cerr.strip()[:400])
    ls2, _ = dexec(c, "ls -l /tmp/carto_%s.pgm /tmp/carto_%s.yaml 2>&1" % (name, name))
    print(ls2.strip())
    # The files live in the container; copy them out to the host before SFTP.
    run(c, "mkdir -p %s/maps" % HOST_DIR)
    run(c, "sudo -n docker cp ClawPi:/tmp/carto_%s.pgm %s/maps/carto_%s.pgm; "
           "sudo -n docker cp ClawPi:/tmp/carto_%s.yaml %s/maps/carto_%s.yaml; "
           "sudo -n docker cp ClawPi:/tmp/carto_%s.pbstream %s/maps/carto_%s.pbstream; "
           "sudo -n chown -R pi:pi %s/maps"
           % (name, HOST_DIR, name, name, HOST_DIR, name, name, HOST_DIR, name, HOST_DIR), 60)
    local = ROOT / "maps" / name
    local.mkdir(parents=True, exist_ok=True)
    sftp = c.open_sftp()
    for ext in (".pgm", ".yaml", ".pbstream"):
        try:
            sftp.get("%s/maps/carto_%s%s" % (HOST_DIR, name, ext), str(local / ("map" + ext)))
        except OSError as exc:
            print("skip", ext, exc)
    sftp.close()
    for f in sorted(local.iterdir()):
        print("saved", f, f.stat().st_size)


def action_stop(c):
    out, err = dexec(
        c,
        BEGIN + " ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "
               '"{trajectory_id: 0}"',
        60,
    )
    print("finish:", out.strip()[:400], err.strip()[:200])
    out, _ = dexec(c, "pkill -f cartographer_occupancy_grid_node; pkill -f cartographer_node; sleep 1; pgrep -af cartographer || true")
    print("remaining:", out.strip() or "none")


def main():
    op = sys.argv[1] if len(sys.argv) > 1 else "status"
    c = client()
    try:
        if op == "start":
            action_start(c)
        elif op == "status":
            action_status(c)
        elif op == "save":
            action_save(c, sys.argv[2] if len(sys.argv) > 2 else time.strftime("%Y%m%d_%H%M%S"))
        elif op == "stop":
            action_stop(c)
    finally:
        c.close()


if __name__ == "__main__":
    main()
