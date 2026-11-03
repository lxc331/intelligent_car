# -*- coding: utf-8 -*-
"""Kill cartographer PIDs inside the car container as root and report state."""
import paramiko

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect("192.168.167.136", username="pi", password="raspberrypi", timeout=8)

checks = [
    ("status", "sudo -n docker exec ClawPi bash -lc \"ps -eo pid,user,stat,comm,args | grep -i cartograph | grep -v grep\""),
    ("kill", "sudo -n docker exec ClawPi bash -lc \"pkill -9 -f cartographer_node ; pkill -9 -f occupancy_grid_node ; true\""),
    ("after", "sudo -n docker exec ClawPi bash -lc \"ps -eo pid,user,stat,comm | grep -i cartograph | grep -v grep\""),
]
for name, cmd in checks:
    _, out, err = client.exec_command(cmd, timeout=25)
    text = out.read().decode("utf-8", "replace").strip()
    error = err.read().decode("utf-8", "replace").strip()
    print("==", name, "==")
    print(text or "(empty)")
    if error:
        print("ERR:", error[-300:])
client.close()
