# -*- coding: utf-8 -*-
"""Run an arbitrary local python file inside the car's ROS2 container over SSH.

  python car_exec.py .work/remote_check.py [args...]
"""
import os
import pathlib
import sys

import paramiko

CAR = os.environ.get("CAR_ADDR", "192.168.167.118")
USER = "pi"
PWD = "raspberrypi"

local = pathlib.Path(sys.argv[1])
script = local.read_text(encoding="utf-8")
args = " ".join(sys.argv[2:])
cmd = (
    "sudo -n docker exec -i -u ubuntu ClawPi bash -lc "
    "\"source /opt/ros/humble/setup.bash; python3 - %s\"" % args
)

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(CAR, username=USER, password=PWD, timeout=8)
stdin, stdout, stderr = client.exec_command(cmd, timeout=240)
stdin.write(script.encode("utf-8"))
stdin.channel.shutdown_write()
out = stdout.read().decode("utf-8", "replace")
err = stderr.read().decode("utf-8", "replace")
client.close()
print(out.strip())
if err.strip():
    print("STDERR:", err.strip()[-2000:])
