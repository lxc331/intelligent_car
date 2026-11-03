# -*- coding: utf-8 -*-
"""Run a short Python snippet inside the car's ClawPi container (read-only helper).

    python car_py.py some_script.py

The file content is base64 encoded so quotes never reach the shell parser.
"""
import base64
import os
import sys

import paramiko

CAR = os.environ.get("CAR_IP", "192.168.167.136")
BEGIN = "source /opt/ros/humble/setup.bash;"


def run_python(source, timeout=180):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(CAR, username="pi", password="raspberrypi", timeout=10)
    try:
        payload = base64.b64encode(source.encode("utf-8")).decode("ascii")
        inner = BEGIN + " echo %s | base64 -d | python3 -" % payload
        command = "sudo -n docker exec -u ubuntu ClawPi bash -lc %s" % repr(inner)
        stdout, stderr = client.exec_command(command, timeout=timeout)[1:3]
        return (stdout.read().decode("utf-8", "replace"), stderr.read().decode("utf-8", "replace"))
    finally:
        client.close()


def _connect():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(CAR, username="pi", password="raspberrypi", timeout=10)
    return client


def shell(command, container=True, timeout=60):
    """Run a shell command on the car host (or inside the ClawPi container)."""
    client = _connect()
    try:
        if container:
            command = "sudo -n docker exec -u ubuntu ClawPi bash -lc %s" % repr(BEGIN + " " + command)
        stdout, stderr = client.exec_command(command, timeout=timeout)[1:3]
        return (stdout.read().decode("utf-8", "replace") + stderr.read().decode("utf-8", "replace")).strip()
    finally:
        client.close()


def dexec(inner, timeout=120):
    return shell(inner, container=True, timeout=timeout)


def main():
    if len(sys.argv) < 2:
        print("usage: car_py.py script.py")
        return 1
    source = open(sys.argv[1], encoding="utf-8").read()
    out, err = run_python(source)
    print(out)
    if err:
        print("--stderr--")
        print(err[-3000:])
    return 0


if __name__ == "__main__":
    sys.exit(main())
