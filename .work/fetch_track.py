# -*- coding: utf-8 -*-
"""Stop the recorder and copy /tmp/drive_track.csv back to the PC."""
import os

import paramiko

CAR = os.environ.get("CAR_IP", "192.168.167.136")
LOCAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drive_track.csv")

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(CAR, username="pi", password="raspberrypi", timeout=10)
try:
    setup = "touch /tmp/stop_track; sleep 2; sudo -n docker cp ClawPi:/tmp/drive_track.csv /home/pi/drive_track.csv; " \
            "sudo -n chown pi:pi /home/pi/drive_track.csv; wc -l /home/pi/drive_track.csv; " \
            "sudo -n docker exec -u ubuntu ClawPi tail -3 /tmp/record_track.log 2>/dev/null || true"
    stdout, stderr = client.exec_command(setup)[1:3]
    print(stdout.read().decode("utf-8", "replace").strip())
    error = stderr.read().decode("utf-8", "replace").strip()
    if error:
        print("stderr:", error[-300:])
    sftp = client.open_sftp()
    sftp.get("/home/pi/drive_track.csv", LOCAL)
    sftp.close()
    print("local", LOCAL, os.path.getsize(LOCAL), "bytes")
finally:
    client.close()
