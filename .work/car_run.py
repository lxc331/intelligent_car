# -*- coding: utf-8 -*-
"""Run a python snippet inside the car's ROS2 container over SSH.

  python car_run.py status
  python car_run.py fwd 0.5 0.05
  python car_run.py turn 90 0.5

The script is streamed through docker exec stdin, so nothing is left on the car
unless the snippet itself writes it. ROS environment is sourced inside bash.
"""
import pathlib
import sys

import paramiko

CAR = "192.168.167.136"
USER = "pi"
PWD = "raspberrypi"
SCRIPT = pathlib.Path(__file__).with_name("car_drive.py").read_text(encoding="utf-8")

args = " ".join(sys.argv[1:])
cmd = (
    "sudo -n docker exec -i -u ubuntu ClawPi bash -lc "
    "\"source /opt/ros/humble/setup.bash; python3 - %s\"" % args
)

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(CAR, username=USER, password=PWD, timeout=8)
stdin, stdout, stderr = client.exec_command(cmd, timeout=180)
stdin.write(SCRIPT.encode("utf-8"))
stdin.channel.shutdown_write()
out = stdout.read().decode("utf-8", "replace")
err = stderr.read().decode("utf-8", "replace")
client.close()
print(out.strip())
if err.strip():
    print("STDERR:", err.strip()[-1500:])
