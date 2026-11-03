import paramiko

HOSTS = ["192.168.167.136", "192.168.167.118"]
for host in HOSTS:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        c.connect(host, username="pi", password="raspberrypi", timeout=8)
    except Exception as ex:
        print(f"== {host} SSH失败: {type(ex).__name__}: {ex}")
        continue
    print(f"== {host} 已连接")
    cmds = [
        "hostname",
        "ls ~",
        "ls /home/pi/ros2_ws/src 2>/dev/null",
        "ls /home/pi/hiwonder-toolbox 2>/dev/null",
        "ps aux | grep -E 'ros|pick|llm|slam|nav' | grep -v grep | awk '{print $11, $12, $13}' | head -25",
    ]
    for cmd in cmds:
        _, out, err = c.exec_command(cmd, timeout=20)
        text = out.read().decode("utf-8", "replace").strip()
        print(f"$ {cmd}\n{text[:2000]}")
    c.close()
