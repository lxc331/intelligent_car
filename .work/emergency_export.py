# -*- coding: utf-8 -*-
"""Emergency export: flush the Cartographer state and pull it to the PC before power dies.

Writes one final pbstream, converts it to pgm/yaml inside the container, copies all
files to the host, then SFTPs them into .work/maps/emergency_<stamp>/.
"""
import pathlib
import sys
import time

import paramiko

CAR = "192.168.167.136"
BEGIN = "source /opt/ros/humble/setup.bash;"
ROOT = pathlib.Path(__file__).resolve().parent
STAMP = time.strftime("%Y%m%d_%H%M%S")
NAME = "final_" + STAMP


def dexec(c, inner, timeout=90):
    o = c.exec_command("sudo -n docker exec -u ubuntu ClawPi bash -lc %s" % repr(BEGIN + " " + inner), timeout=timeout)
    return (o[1].read().decode("utf-8", "replace") + o[2].read().decode("utf-8", "replace")).strip()


def sh(c, cmd, timeout=90):
    o = c.exec_command(cmd, timeout=timeout)
    return (o[1].read().decode("utf-8", "replace") + o[2].read().decode("utf-8", "replace")).strip()


c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(CAR, username="pi", password="raspberrypi", timeout=8)
try:
    req = '"{filename: \'/tmp/carto_%s.pbstream\', include_unfinished_submaps: true}"' % NAME
    print("write_state:", dexec(c, "ros2 service call /write_state cartographer_ros_msgs/srv/WriteState " + req, 60)[-160:])
    print("convert:", dexec(c, "ros2 run cartographer_ros cartographer_pbstream_to_ros_map "
                               "-pbstream_filename /tmp/carto_%s.pbstream -map_filestem /tmp/carto_%s"
                               % (NAME, NAME), 120)[-260:])
    sh(c, "mkdir -p /home/pi/carapp_carto/emergency")
    sh(c, "sudo -n docker cp ClawPi:/tmp/carto_%s.pbstream /home/pi/carapp_carto/emergency/; "
          "sudo -n docker cp ClawPi:/tmp/carto_%s.pgm /home/pi/carapp_carto/emergency/; "
          "sudo -n docker cp ClawPi:/tmp/carto_%s.yaml /home/pi/carapp_carto/emergency/; "
          "sudo -n chown -R pi:pi /home/pi/carapp_carto/emergency" % (NAME, NAME, NAME), 90)
    # Also grab every autosave snapshot still on disk (newest last).
    names = sh(c, "sudo -n docker exec ClawPi sh -c 'ls /tmp/carto_auto_*.pbstream 2>/dev/null'").split()
    print("autosaves:", len(names))
    for n in names[-6:]:
        sh(c, "sudo -n docker cp ClawPi:%s /home/pi/carapp_carto/emergency/%s; sudo -n chown pi:pi /home/pi/carapp_carto/emergency/%s"
              % (n, n.split("/")[-1], n.split("/")[-1]), 60)
    local = ROOT / "maps" / ("emergency_" + STAMP)
    local.mkdir(parents=True, exist_ok=True)
    sftp = c.open_sftp()
    files = sftp.listdir("/home/pi/carapp_carto/emergency")
    for f in files:
        try:
            sftp.get("/home/pi/carapp_carto/emergency/" + f, str(local / f))
        except OSError as exc:
            print("skip", f, exc)
    sftp.close()
    print("pulled:", ", ".join(sorted(p.name for p in local.iterdir())))
finally:
    c.close()
print("DONE", local)
