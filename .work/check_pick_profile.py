"""Offline check: does the running profile find yesterday's pick calibration?"""
import io
import json
import sys
from types import SimpleNamespace

sys.path.insert(0, 'app')
sys.path.insert(0, '.work')

from carapp.config import load_config  # noqa: E402
from carapp.features.tracking_pick import TrackingPickControls  # noqa: E402
from carapp.core.visual_pick import calibration_problem, validate_calibration  # noqa: E402

cfg = load_config()
car = cfg['car']
tp = TrackingPickControls.__new__(TrackingPickControls)
tp.ctx = SimpleNamespace(cfg=cfg)
tp.cfg = cfg['tracking_pick']

identity = tp.car_identity()
key = tp.key()
cal = tp.calibration()
pose = tp.pose()
print('profile :', car['profile_id'], car['host'], car['transport'])
print('identity:', identity)
print('key     :', key)
print('servo   : look_down=%s arm_down=%s limits pitch=%s' % (
    cfg['servo']['actions']['look_down'], cfg['servo']['actions']['arm_down'],
    cfg['servo']['limits']['pitch']))
print('pose    :', pose)
print('cal     :', {k: (round(cal[k], 4) if isinstance(cal.get(k), (int, float)) else cal.get(k))
                   for k in ('x', 'y', 'width', 'host', 'created')} if cal else None)
print('problem :', calibration_problem(cal, identity, tp.cfg['color'], pose) or '(可用)')
if cal:
    validate_calibration(cal, identity, tp.cfg['color'], pose)
