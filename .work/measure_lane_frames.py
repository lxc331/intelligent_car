# -*- coding: utf-8 -*-
"""Upload reference frames + the lane measurement script into the car container and run it."""
import paramiko

CAR = "192.168.167.136"
FRAMES = ["drive_view_02.jpg", "drive_view_05.jpg", "drive_view_08.jpg",
          "resume_view_01.jpg", "resume_view_02.jpg", "resume_view_03.jpg"]

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(CAR, username="pi", password="raspberrypi", timeout=10)
sftp = c.open_sftp()
c.exec_command("mkdir -p /home/pi/frames")
for f in FRAMES:
    sftp.put(r"D:\项目\intelligent_car-main\.work\\" + f, "/home/pi/frames/" + f)
sftp.put(r"D:\项目\intelligent_car-main\.work\lane_measure.py", "/home/pi/frames/lane_measure.py")
sftp.close()

for cmd in ("sudo -n docker cp /home/pi/frames ClawPi:/tmp/frames",
            "sudo -n docker exec -u ubuntu ClawPi python3 /tmp/frames/lane_measure.py"):
    out = c.exec_command(cmd, timeout=60)
    text = out[1].read().decode("utf-8", "replace")
    err = out[2].read().decode("utf-8", "replace")
    print(text.strip())
    if err.strip():
        print("ERR", err.strip()[:600])
c.close()
