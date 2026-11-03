# -*- coding: utf-8 -*-
"""Read-only check that mapping is live: /map size, odom pose, scan rate."""
import paramiko

CAR = "192.168.167.136"
BEGIN = "source /opt/ros/humble/setup.bash;"
CMDS = [
    "ros2 topic echo /map --once --field info",
    "timeout 3 ros2 topic hz /scan_raw | tail -2",
    "ros2 topic echo /odom --once --field pose.pose.position",
]


def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(CAR, username="pi", password="raspberrypi", timeout=10)
    try:
        for cmd in CMDS:
            outer = "sudo -n docker exec -u ubuntu ClawPi bash -lc %r" % (BEGIN + " " + cmd)
            _in, out, err = c.exec_command(outer, timeout=40)
            text = (out.read() + err.read()).decode("utf-8", "replace").strip()
            print("$ " + cmd)
            print(text[-600:])
            print()
    finally:
        c.close()


if __name__ == "__main__":
    main()
