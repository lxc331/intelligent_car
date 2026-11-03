import sys

sys.path.insert(0, r"D:\项目\intelligent_car-main\.work")
from car_py import dexec

cmds = [
    "ros2 node list 2>/dev/null | sort",
    "ps -eo pid,etimes,args | grep navigation.launch | grep -v grep",
    "ros2 service list 2>/dev/null | grep -E 'amcl/get_state|map_server/get_state'",
    "ros2 node list 2>/dev/null | grep -E 'amcl|map_server'",
]
for cmd in cmds:
    print("=== %s" % cmd)
    print(dexec(cmd, 60))
