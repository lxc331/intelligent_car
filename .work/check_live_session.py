# -*- coding: utf-8 -*-
"""Inspect the live navigation session on the car (read-only)."""
import json

import paramiko

CAR = "192.168.167.136"


def dexec(c, inner, timeout=120):
    command = "sudo -n docker exec -u ubuntu ClawPi bash -lc %s" % repr(inner)
    o = c.exec_command(command, timeout=timeout)
    return (o[1].read().decode("utf-8", "replace"), o[2].read().decode("utf-8", "replace"))


def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(CAR, username="pi", password="raspberrypi", timeout=10)
    try:
        out, _ = dexec(c, "ls -1dt /tmp/carapp_nav_* 2>/dev/null | head -3")
        dirs = [line.strip() for line in out.splitlines() if line.strip()]
        print("dirs:", dirs)
        if not dirs:
            return
        latest = dirs[0]
        out, _ = dexec(c, "cat %s/state.json" % latest)
        print("state:", out[:1600])
        out, _ = dexec(c, "tail -n 30 %s/nav2.log" % latest)
        print("-- nav2.log tail --")
        print(out)
    finally:
        c.close()


if __name__ == "__main__":
    main()
