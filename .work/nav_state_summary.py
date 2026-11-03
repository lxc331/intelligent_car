# -*- coding: utf-8 -*-
"""Runs inside the car container: compact navigation session summary."""
import glob
import json
import os
import time

dirs = sorted(glob.glob('/tmp/carapp_nav_*'), key=os.path.getmtime, reverse=True)
root = dirs[0]
state = json.load(open(os.path.join(root, 'state.json')))
path = state.pop('path', [])
print('session', root, 'age_s=%.0f' % (time.time() - os.path.getmtime(root)))
print(json.dumps(state, ensure_ascii=False, indent=1)[:2500])
print('path points', len(path))
if path:
    print('path[0]', [round(v, 3) for v in path[0]])
    print('path[-1]', [round(v, 3) for v in path[-1]])

log = os.path.join(root, 'nav2.log')
text = open(log, encoding='utf-8', errors='replace').read()
lines = text.splitlines()
print('nav2.log lines', len(lines))
keep = []
for line in lines:
    if any(word in line for word in ('Failed to make progress', 'Aborting', 'abort',
                                     'Controller', 'progress', 'collision', 'Rotate',
                                     'Goal', 'goal')):
        keep.append(line)
print('--- relevant lines')
print('\n'.join(keep[-40:])[-5000:])

yaml_path = os.path.join(root, 'nav2.yaml')
if os.path.exists(yaml_path):
    print('--- progress checker settings')
    for line in open(yaml_path, encoding='utf-8'):
        if any(word in line for word in ('movement_timeout', 'required_movement',
                                         'progress_checker', 'max_vel_theta',
                                         'max_vel_x', 'min_vel_x', 'max_angular')):
            print(line.rstrip())
