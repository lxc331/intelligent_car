"""Give the 2nd-car profile yesterday's proven pick calibration and prep pose.

The 1st-car profile keeps its own copies; this only adds the HW-A0857935 keys so
that selecting 2 号车 in the UI finds the same measured values.
"""
import copy
import io
import json
import shutil

PATH = 'app/config/settings.json'
BACKUP = '.work/backup_param_shift/settings.json'

shutil.copyfile(PATH, BACKUP)
with io.open(PATH, encoding='utf-8') as f:
    cfg = json.load(f)

tp = cfg['tracking_pick']
tp.setdefault('preparation_poses', {})['HW-A0857935'] = {'arm': 1796}

cals = tp['calibrations']
yesterday = cals['HW-10CCA58E:192.168.167.136:red']
moved = copy.deepcopy(yesterday)
moved['host'] = 'HW-A0857935'
cals['HW-A0857935:red'] = moved

cfg['car']['profile_id'] = 'HW-A0857935'
cfg['car']['host'] = '192.168.150.2'
cfg['car']['transport'] = 'wired'
cfg['car']['sta_ssid'] = ''

with io.open(PATH, 'w', encoding='utf-8', newline='\n') as f:
    json.dump(cfg, f, ensure_ascii=False, indent=2)

print('car:', cfg['car'])
print('prep:', tp['preparation_poses'])
for key, value in cals.items():
    print(key, {k: (round(value[k], 4) if isinstance(value[k], float) else value[k])
                for k in ('x', 'y', 'width', 'host', 'created')})
