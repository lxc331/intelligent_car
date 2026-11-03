import subprocess

out = subprocess.run(["ps", "-eo", "pid,etimes,cmd"], capture_output=True, text=True).stdout
for line in out.splitlines():
    if any(key in line for key in ("navigation.launch", "runtime.py", "nav2_", "amcl", "map_server", "bt_navigator")):
        print(line[:220])
