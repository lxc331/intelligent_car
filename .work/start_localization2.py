# -*- coding: utf-8 -*-
"""Start map_server + AMCL on the current car using a leftover session's map and params."""
import json
import os
import subprocess
import time

SESSION = "/tmp/carapp_nav_d512c9dfe68a4b4caec7a3063d0ae4d8"
MAP = SESSION + "/map.yaml"
PARAMS = SESSION + "/nav2.yaml"

log = open("/tmp/localization.log", "w")
cmd = ["ros2", "launch", "nav2_bringup", "localization_launch.py",
       "map:=" + MAP, "params_file:=" + PARAMS, "use_sim_time:=false"]
process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
print(json.dumps(dict(started=process.pid, map=MAP)), flush=True)
time.sleep(14.0)
print(json.dumps(dict(alive=process.poll() is None)), flush=True)
if process.poll() is not None:
    print(open("/tmp/localization.log", errors="replace").read()[-800:], flush=True)
