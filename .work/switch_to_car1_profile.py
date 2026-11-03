"""Point the app at the 2nd car over the wired link but run the 1st car's profile.

The physical car on the bench is HW-A0857935 (2 号车) but yesterday's working
program is the 1 号车 profile (HW-10CCA58E): its servo directions/limits and the
successful pick calibration were all measured with that profile.  Keep the
profile, point it at the wired address.
"""
import io
import json
import shutil

PATH = 'app/config/settings.json'
BACKUP = '.work/backup_car1_profile/settings.json'

shutil.copyfile(PATH, BACKUP)
with io.open(PATH, encoding='utf-8') as f:
    cfg = json.load(f)

car = cfg['car']
car['profile_id'] = 'HW-10CCA58E'
car['host'] = '192.168.150.2'
car['transport'] = 'wired'
car['sta_ssid'] = ''

with io.open(PATH, 'w', encoding='utf-8', newline='\n') as f:
    json.dump(cfg, f, ensure_ascii=False, indent=2)

print('car:', car)
print('calibrations:', list(cfg['tracking_pick']['calibrations']))
