# -*- coding: utf-8 -*-
"""Kill a pid inside the car container (pid given on the command line)."""
import sys

import paramiko

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect("192.168.167.136", username="pi", password="raspberrypi", timeout=8)
for pid in sys.argv[1:]:
    _, out, err = client.exec_command("sudo -n docker exec -u ubuntu ClawPi kill -KILL %s" % pid, timeout=15)
    print(pid, out.read().decode().strip(), err.read().decode().strip()[-150:])
_, out, err = client.exec_command(
    'sudo -n docker exec -u ubuntu ClawPi bash -lc "pgrep -af cartographer ; true"', timeout=20
)
print("LEFT:", out.read().decode().strip() or "none")
client.close()
