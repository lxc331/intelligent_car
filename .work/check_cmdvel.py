# -*- coding: utf-8 -*-
"""Read-only: who publishes/subscribes cmd_vel and whether rosbridge has clients."""
import paramiko

CAR = "192.168.167.136"
BEGIN = "source /opt/ros/humble/setup.bash;"
CMDS = [
    "ros2 topic info /controller/cmd_vel -v | head -30",
    "ss -tn state established '( sport = :9090 or sport = :8080 )' | head -10",
    "ros2 node list | grep -E 'joy|controller|base' ",
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
            print(text[-1200:])
            print()
    finally:
        c.close()


if __name__ == "__main__":
    main()
