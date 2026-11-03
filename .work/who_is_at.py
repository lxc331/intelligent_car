# -*- coding: utf-8 -*-
"""Who is at CAR_IP? Serial number decides 1号车 vs 2号车."""
import os

import paramiko

CAR = os.environ.get("CAR_IP", "192.168.167.136")
client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(CAR, username="pi", password="raspberrypi", timeout=8)
script = ("printf 'serial='; tr -d '\\0\\n' < /proc/device-tree/serial-number; echo; "
          "printf 'host='; hostname; "
          "printf 'addr='; ip -4 -o addr show | awk '{print $2\"=\"$4}' | tr '\\n' ' '; echo")
stdout = client.exec_command(script)[1].read().decode("utf-8", "replace")
print(CAR)
print(stdout)
client.close()
