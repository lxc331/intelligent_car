"""Read a vehicle's CPU serial number over SSH so the app can verify *which car* it drives.

The console can only tell cars apart by profile; nothing on the ROS side carries the
serial.  This module adds one read-only check: connect with the recorded host key,
read /proc/device-tree/serial-number, and turn it into the same HW-XXXXXXXX profile
id the Wi-Fi wizard uses.  Callers run it in a worker thread, never on Tk.
"""
import base64
import re
import socket

import paramiko

# /proc/device-tree/serial-number is raw bytes with a trailing NUL.
SERIAL_COMMAND = 'tr -d "\\0\\n" < /proc/device-tree/serial-number; echo'
PROFILE_ID = re.compile(r"HW-[0-9A-F]{8}\Z")


def identity_from_serial(serial):
    """Map a Pi serial such as e92c53fa0857935d to HW-A0857935 (same rule as the Wi-Fi wizard)."""
    text = (serial or "").strip()
    if len(text) < 9:
        return ""
    candidate = ("HW-" + text[-9:-1]).upper()
    return candidate if PROFILE_ID.match(candidate) else ""


def _known_keys(host, path):
    """(key type, key blob) pairs recorded for this address under any vehicle alias."""
    try:
        with open(path, "r", encoding="utf-8") as stream:
            lines = stream.readlines()
    except OSError:
        return []
    keys = []
    for line in lines:
        parts = line.split()
        if len(parts) < 3 or parts[0].startswith("#"):
            continue
        if not any(alias.rsplit("@", 1)[-1] == host for alias in parts[0].split(",")):
            continue
        try:
            keys.append((parts[1], base64.b64decode(parts[2], validate=True)))
        except ValueError:
            continue
    return keys


class _OnlyKnownKeys(paramiko.MissingHostKeyPolicy):
    """Reject a host key that was never recorded for this address."""

    def __init__(self, allowed):
        self.allowed = allowed

    def missing_host_key(self, client, hostname, key):
        if not any(key.get_name() == kind and key.asbytes() == blob for kind, blob in self.allowed):
            raise paramiko.SSHException("车辆主机密钥与已记录的不一致")


def verify_vehicle_identity(host, username, password, *, port=22, known_hosts="", timeout=6):
    """Return {"ok": bool, "identity": str, "detail": str}; never raises."""
    if not password:
        return {"ok": False, "identity": "", "detail": "没有可用的登录密码"}
    allowed = _known_keys(host, known_hosts)
    if not allowed:
        return {"ok": False, "identity": "", "detail": "没有该地址的主机密钥记录"}
    client = paramiko.SSHClient()
    try:
        client.set_missing_host_key_policy(_OnlyKnownKeys(allowed))
        client.connect(host, port=int(port), username=username, password=password,
                       look_for_keys=False, allow_agent=False,
                       timeout=timeout, auth_timeout=timeout, banner_timeout=timeout)
        _, stdout, _ = client.exec_command(SERIAL_COMMAND, timeout=timeout)
        identity = identity_from_serial(stdout.read().decode("utf-8", "replace"))
        if not identity:
            return {"ok": False, "identity": "", "detail": "读不到车辆序列号"}
        return {"ok": True, "identity": identity, "detail": "ok"}
    except (paramiko.SSHException, OSError, socket.error) as exc:
        return {"ok": False, "identity": "", "detail": str(exc)[:160] or exc.__class__.__name__}
    finally:
        client.close()
