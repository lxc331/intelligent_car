"""Read the physical car's CPU serial over the wired link (read-only)."""
import sys

sys.path.insert(0, '.work')

import paramiko  # noqa: E402

HOST = '192.168.150.2'
USER = 'pi'
PASSWORD = 'raspberrypi'

with paramiko.SSHClient() as ssh:
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(HOST, port=22, username=USER, password=PASSWORD, timeout=8,
                allow_agent=False, look_for_keys=False)
    for label, cmd in (
            ('serial', 'tr -d "\\0\\n" < /proc/device-tree/serial-number; echo'),
            ('model', 'tr -d "\\0" < /proc/device-tree/model; echo'),
            ('hostname', 'hostname'),
            ('cpu', "grep -m1 Serial /proc/cpuinfo || true"),
    ):
        _, out, err = ssh.exec_command(cmd, timeout=10)
        print('%-8s %s' % (label + ':', out.read().decode('utf-8', 'replace').strip()))
