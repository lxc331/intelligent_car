import paramiko

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect("192.168.150.2", username="pi", password="raspberrypi", timeout=8)

cmds = [
    "docker ps --format '{{.Names}} | {{.Image}} | {{.Status}}'",
    "docker exec ClawPi bash -lc \"source /opt/ros/humble/setup.bash; timeout 10 ros2 node list 2>/dev/null | head -30\"",
    "docker exec ClawPi bash -lc \"source /opt/ros/humble/setup.bash; timeout 10 ros2 topic list 2>/dev/null | grep -E 'scan_raw|image_raw|odom|cmd_vel'\"",
    "docker exec ClawPi bash -lc \"ls /home/ubuntu/ros2_ws/src 2>/dev/null\"",
]
for cmd in cmds:
    i, o, e = c.exec_command(cmd, timeout=30)
    print("==", cmd)
    print(o.read().decode(errors="replace").strip()[:1500])
    err = e.read().decode(errors="replace").strip()
    if err:
        print("ERR:", err[:300])
c.close()
