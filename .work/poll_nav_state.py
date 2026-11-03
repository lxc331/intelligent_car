# -*- coding: utf-8 -*-
"""Poll the newest on-car navigation state file for a few seconds."""
import json
import time

import paramiko

CAR = "192.168.167.136"


def dexec(c, inner, timeout=60):
    command = "sudo -n docker exec -u ubuntu ClawPi bash -lc %s" % repr(inner)
    o = c.exec_command(command, timeout=timeout)
    return o[1].read().decode("utf-8", "replace")


def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(CAR, username="pi", password="raspberrypi", timeout=10)
    try:
        out = dexec(c, "ls -1dt /tmp/carapp_nav_* | head -1")
        latest = out.strip()
        print("session:", latest)
        for _ in range(5):
            raw = dexec(c, "cat %s/state.json" % latest)
            try:
                state = json.loads(raw)
            except ValueError:
                print("unreadable", raw[:200])
                time.sleep(2)
                continue
            print(time.strftime("%H:%M:%S"),
                  {key: state.get(key) for key in ("running", "ready", "localized", "goal_state", "detail")})
            time.sleep(2)
    finally:
        c.close()


if __name__ == "__main__":
    main()
