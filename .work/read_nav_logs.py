# -*- coding: utf-8 -*-
"""Read the car-side navigation session logs for one run."""
import sys

import paramiko

CAR = "192.168.167.136"
SESSION = sys.argv[1] if len(sys.argv) > 1 else "carapp_nav_c73d3669b26143bcafd5fc61d8866dc9"


def main():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(CAR, username="pi", password="raspberrypi", timeout=10)
    base = "/tmp/" + SESSION
    cmd = (
        "echo '=== files ==='; ls -la {base} 2>&1; "
        "echo '=== state ==='; cat {base}/state.json 2>&1; "
        "echo '=== supervisor ==='; tail -40 {base}/supervisor.log 2>&1; "
        "echo '=== nav2 ==='; tail -60 {base}/nav2.log 2>&1"
    ).format(base=base)
    _, out, err = client.exec_command(cmd, timeout=60)
    print(out.read().decode("utf-8", "replace"))
    tail = err.read().decode("utf-8", "replace").strip()
    if tail:
        print("[stderr]", tail)
    client.close()


if __name__ == "__main__":
    main()
