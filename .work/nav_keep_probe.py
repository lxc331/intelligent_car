# -*- coding: utf-8 -*-
"""Read the newest navigation session inside the car container (not host /tmp)."""
import paramiko

CAR = "192.168.167.118"

INNER = (
    "ls -dt /tmp/carapp_nav_* 2>/dev/null | head -3; "
    "D=$(ls -dt /tmp/carapp_nav_* 2>/dev/null | head -1); echo DIR=$D; "
    "cat $D/state.json 2>/dev/null; echo '--- files'; ls -l $D 2>/dev/null; "
    "echo '--- nav2.log tail'; tail -n 40 $D/nav2.log 2>/dev/null; "
    "echo '--- supervisor tail'; tail -n 10 $D/supervisor.log 2>/dev/null"
)

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(CAR, username="pi", password="raspberrypi", timeout=10)
cmd = "sudo -n docker exec -u ubuntu ClawPi bash -lc %s" % repr(INNER)
out = client.exec_command(cmd, timeout=60)[1].read().decode("utf-8", "replace")
print(out)
client.close()
