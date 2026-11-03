# -*- coding: utf-8 -*-
"""Why did the car reboot? Look at boot history, voltage topics and shutdown logs."""
import paramiko

CAR = "192.168.167.136"
BEGIN = "source /opt/ros/humble/setup.bash;"


def sh(c, cmd, timeout=40):
    o = c.exec_command(cmd, timeout=timeout)
    return (o[1].read().decode("utf-8", "replace") + o[2].read().decode("utf-8", "replace")).strip()


c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(CAR, username="pi", password="raspberrypi", timeout=10)
try:
    print("boot:", sh(c, "uptime -s; who -b"))
    print("last reboots:", sh(c, "last -x reboot shutdown 2>/dev/null | head -6"))
    print("prev boot tail:", sh(c, "sudo -n journalctl -b -1 -n 8 --no-pager 2>/dev/null | tail -8")[:1500])
    print("volt/batt topics:", sh(c, "sudo -n docker exec -u ubuntu ClawPi bash -lc "
                                     + repr(BEGIN + " ros2 topic list 2>/dev/null | grep -iE 'volt|batt|power' | tr '\\n' ' '"))
          )
    print("power now:", sh(c, "vcgencmd get_throttled 2>/dev/null; vcgencmd measure_volts core 2>/dev/null"))
finally:
    c.close()
