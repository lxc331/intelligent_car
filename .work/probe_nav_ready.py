# -*- coding: utf-8 -*-
"""Read-only: are /scan_raw and /odom publishing right now?"""
import paramiko

HOST = "192.168.167.136"
BEGIN = ("source /opt/ros/humble/setup.bash; "
         "source /home/ubuntu/ros2_ws/install/setup.bash; ")
REMOTE = BEGIN + "ros2 node list; echo ===; ros2 topic info /scan_raw; echo ===; ros2 topic info /odom; echo ===; timeout 4 ros2 topic hz /scan_raw 2>&1 | head -2; timeout 4 ros2 topic hz /odom 2>&1 | head -2"


def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username="pi", password="raspberrypi", timeout=10)
    inner = "bash -lc " + repr(REMOTE)
    cmd = "sudo -n docker exec -u ubuntu ClawPi " + inner
    stdin, out, err = c.exec_command(cmd, timeout=90)
    text = out.read().decode("utf-8", "replace") + err.read().decode("utf-8", "replace")
    print(text[:4000])
    c.close()


if __name__ == "__main__":
    main()
