# -*- coding: utf-8 -*-
"""Print the newest car-side navigation session state and log tail."""
import paramiko

CAR = "192.168.167.118"

SCRIPT = r"""
ls -dt /tmp/carapp_nav_* | head -3
echo ================
D=$(ls -dt /tmp/carapp_nav_* | head -1)
echo "$D"
echo ---- state.json
cat "$D/state.json"
echo ---- files
ls -l "$D"
echo ---- nav2.log tail
tail -n 40 "$D/nav2.log"
echo ---- supervisor.log tail
tail -n 15 "$D/supervisor.log"
"""

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(CAR, username="pi", password="raspberrypi", timeout=10)
out = client.exec_command("bash -lc %s" % repr(SCRIPT), timeout=40)[1].read().decode("utf-8", "replace")
print(out)
client.close()
