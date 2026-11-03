# -*- coding: utf-8 -*-
"""Inspect the navigation session on whichever car answers; prints state + readiness."""
import json
import sys
import time

import paramiko

CAR = sys.argv[1] if len(sys.argv) > 1 else "192.168.167.118"


def dexec(c, inner, timeout=120):
    command = "sudo -n docker exec -u ubuntu ClawPi bash -lc %s" % repr(inner)
    o = c.exec_command(command, timeout=timeout)
    return (o[1].read().decode("utf-8", "replace"), o[2].read().decode("utf-8", "replace"))


def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(CAR, username="pi", password="raspberrypi", timeout=10)
    try:
        out, _ = dexec(c, "docker ps --format '{{.Names}} {{.Status}}'")
        print("containers:", out.strip())
        out, _ = dexec(c, "ls -1dt /tmp/carapp_nav_* 2>/dev/null | head -3")
        dirs = [line.strip() for line in out.splitlines() if line.strip()]
        print("sessions:", dirs)
        if not dirs:
            out, _ = dexec(c, "ros2 node list 2>/dev/null | sort | head -30")
            print("nodes:\n" + out)
            return
        for _ in range(3):
            raw, _ = dexec(c, "cat %s/state.json" % dirs[0])
            try:
                state = json.loads(raw)
            except ValueError:
                print("unreadable:", raw[:200])
                break
            print(time.strftime("%H:%M:%S"),
                  {key: state.get(key) for key in ("running", "ready", "localized", "pose", "goal_state", "detail")})
            time.sleep(2)
        out, _ = dexec(c, "grep -c '' %s/nav2.log; tail -n 8 %s/nav2.log" % (dirs[0], dirs[0]))
        print("-- nav2.log --")
        print(out[-2500:])
    finally:
        c.close()


if __name__ == "__main__":
    main()
