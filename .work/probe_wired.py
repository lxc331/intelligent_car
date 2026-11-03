"""只读探测：网线直连下的小车状态。用法：python .work/probe_wired.py [ip]"""
import sys
import paramiko

ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.150.2"
c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(ip, username="pi", password="raspberrypi", timeout=8)

cmds = [
    "uptime",
    "docker ps --format '{{.Names}}|{{.Status}}'",
    "ss -ltn 2>/dev/null | grep -E ':(22|8080|9090|80|9091)\\b'",
    "docker exec ros2-humble bash -lc \"timeout 8 ros2 node list 2>/dev/null | head -30\"",
    "docker exec ros2-humble bash -lc \"timeout 8 ros2 topic list 2>/dev/null | grep -E 'scan_raw|image_raw|odom|cmd_vel'\"",
    "cat /home/pi/ros2_ws/README.md 2>/dev/null | head -5",
    "ls /home/pi/ros2_ws/src 2>/dev/null",
    "tail -n 20 /home/pi/ros2_ws/log/*.log 2>/dev/null | tail -20",
]
for cmd in cmds:
    stdin, stdout, stderr = c.exec_command(cmd, timeout=25)
    out = stdout.read().decode(errors="replace").strip()
    err = stderr.read().decode(errors="replace").strip()
    print("==", cmd)
    print(out[:1500] or "(empty)")
    if err:
        print("ERR:", err[:400])
c.close()
