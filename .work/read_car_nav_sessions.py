# -*- coding: utf-8 -*-
"""Read car-side navigation session dirs and supervisor logs (read-only)."""
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
        out, err = dexec(c, "ls -1dt /tmp/carapp_nav_* 2>/dev/null | head -6")
        dirs = [line.strip() for line in out.splitlines() if line.strip()]
        print("dirs:", dirs)
        for d in dirs[:2]:
            print("==", d)
            out, err = dexec(c, "ls -la %s; echo ---state---; cat %s/state.json 2>/dev/null | head -c 1200; echo; echo ---supervisor---; tail -n 25 %s/supervisor.log 2>/dev/null" % (d, d, d))
            print(out)
    finally:
        c.close()


if __name__ == "__main__":
    main()
