# -*- coding: utf-8 -*-
"""Read-only Wi-Fi diagnosis over the Ethernet link."""
import paramiko

CAR = "192.168.150.2"
CMD = r"""
echo '--- ip ---'; ip -4 -o addr show | sed 's/  */ /g'
echo '--- wlan0 ---'; ip link show wlan0 2>&1 | head -3
echo '--- iw ---'; iw dev 2>&1 | head -20
echo '--- nmcli dev ---'; nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device 2>&1
echo '--- nmcli wlan ---'; nmcli -f GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS device show wlan0 2>&1
echo '--- active conns ---'; nmcli -t -f NAME,UUID,TYPE,DEVICE connection show --active 2>&1
echo '--- scans ---'; nmcli -t -f SSID,SIGNAL,SECURITY device wifi list 2>&1 | head -12
echo '--- wifi service ---'; systemctl list-units --type=service --all 2>/dev/null | grep -i -E 'wifi|hiwonder' | head -10
echo '--- wifi procs ---'; ps -eo pid,etime,cmd | grep -i -E 'wifi' | grep -v grep | head -10
echo '--- toolbox ---'; ls /home/pi/hiwonder-toolbox/ 2>&1 | head -20
echo '--- conf ---'; cat /home/pi/hiwonder-toolbox/wifi_conf.py 2>/dev/null | head -40
echo '--- dmesg wifi ---'; dmesg 2>/dev/null | grep -i -E 'brcmf|wlan' | tail -15
echo '--- uptime ---'; uptime
"""


def main():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(CAR, username="pi", password="raspberrypi", timeout=10)
    _, out, err = client.exec_command(CMD, timeout=60)
    print(out.read().decode("utf-8", "replace"))
    tail = err.read().decode("utf-8", "replace").strip()
    if tail:
        print("[stderr]", tail)
    client.close()


if __name__ == "__main__":
    main()
