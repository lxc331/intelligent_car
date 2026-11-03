# -*- coding: utf-8 -*-
"""Read serial + hostname + addresses from any car that answers on 22."""
import sys

import paramiko

CREDENTIALS = [("pi", "raspberrypi"), ("ubuntu", "ubuntu")]
READ = ("tr -d '\\0' < /proc/device-tree/serial-number; echo; hostname; "
        "hostname -I; ip -4 -o addr show scope global | awk '{print $2, $4}'")


def read(host):
    for user, password in CREDENTIALS:
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            client.connect(host, username=user, password=password, timeout=6)
        except Exception as exc:
            client.close()
            print(host, user, "FAIL", type(exc).__name__)
            continue
        try:
            out, err = client.exec_command(READ, timeout=15)[1:3]
            print(host, "as", user, "=>")
            print(out.read().decode("utf-8", "replace").strip())
            print("stderr:", err.read().decode("utf-8", "replace").strip()[:200])
        finally:
            client.close()
        return True
    return False


if __name__ == "__main__":
    for address in sys.argv[1:]:
        read(address)
