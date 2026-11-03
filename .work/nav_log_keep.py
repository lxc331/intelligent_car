# -*- coding: utf-8 -*-
"""Keep a copy of every car-side navigation session outside the container.

The app deletes /tmp/carapp_nav_<id> (which lives inside the ClawPi container)
when the session ends, taking nav2.log with it. This loop mirrors the useful
files to /home/pi/carapp_nav_keep on the car so a failed run stays inspectable.

    python nav_log_keep.py [seconds] [car-ip]
"""
import sys
import time

import paramiko

CAR = sys.argv[2] if len(sys.argv) > 2 else "192.168.167.118"
KEEP = "/home/pi/carapp_nav_keep"
LIMIT = float(sys.argv[1]) if len(sys.argv) > 1 else 900.0
FILES = ("nav2.log", "state.json", "supervisor.log", "owner.json", "nav2.yaml")

CONTAINER = "ClawPi"


def main():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(CAR, username="pi", password="raspberrypi", timeout=10)
    client.exec_command("mkdir -p %s" % KEEP, timeout=20)[1].channel.recv_exit_status()

    def run(command, timeout=40):
        out = client.exec_command("bash -lc %s" % repr(command), timeout=timeout)
        return out[1].read().decode("utf-8", "replace").strip()

    end = time.time() + LIMIT
    seen = set()
    while time.time() < end:
        listing = run("sudo -n docker exec -u ubuntu %s bash -lc 'ls -d /tmp/carapp_nav_* 2>/dev/null'"
                      % CONTAINER)
        sessions = [line.strip() for line in listing.splitlines()
                    if line.strip().startswith("/tmp/carapp_nav_")]
        for path in sessions:
            name = path.rsplit("/", 1)[-1]
            target = "%s/%s" % (KEEP, name)
            run("mkdir -p %s" % target, timeout=20)
            for item in FILES:
                run("sudo -n docker cp %s:%s/%s %s/%s >/dev/null 2>&1"
                    % (CONTAINER, path, item, target, item), timeout=30)
            if name not in seen:
                seen.add(name)
                print("%s new session %s" % (time.strftime("%H:%M:%S"), name), flush=True)
        print("%s sessions=%d mirrored=%d" % (time.strftime("%H:%M:%S"), len(sessions), len(seen)),
              flush=True)
        time.sleep(20.0)
    client.close()


if __name__ == "__main__":
    main()
