# -*- coding: utf-8 -*-
"""Read-only: cmd_vel endpoint counts, node list, and 6 s of cmd_vel traffic."""
import paramiko

CAR = "192.168.167.136"
BEGIN = "source /opt/ros/humble/setup.bash;"
CMDS = [
    "ros2 topic info /controller/cmd_vel",
    "ros2 topic info /app/cmd_vel",
    "ros2 node list | wc -l",
    "timeout 6 ros2 topic echo /controller/cmd_vel --once 2>&1 | head -12",
]


def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(CAR, username="pi", password="raspberrypi", timeout=10)
    try:
        for cmd in CMDS:
            outer = "sudo -n docker exec -u ubuntu ClawPi bash -lc %r" % (BEGIN + " " + cmd)
            _in, out, err = c.exec_command(outer, timeout=60)
            text = (out.read() + err.read()).decode("utf-8", "replace").strip()
            print("$ " + cmd)
            print(text[-800:])
            print()
    finally:
        c.close()


if __name__ == "__main__":
    main()
