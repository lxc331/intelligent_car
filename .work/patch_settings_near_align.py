"""One-off: align app/config/settings.json with the near-alignment defaults."""
import io
import json

PATH = 'app/config/settings.json'

with io.open(PATH, encoding='utf-8') as f:
    cfg = json.load(f)

tp = cfg['tracking_pick']
tp.pop('near_x_tolerance', None)
tp.setdefault('near_align_enter', .04)
tp.setdefault('near_align_exit', .015)
tp.setdefault('close_x_tolerance', .02)
order = ['color', 'max_speed', 'min_speed', 'max_turn', 'turn_gain', 'control_hz',
         'x_tolerance', 'arrive_margin', 'overshoot_margin', 'straight_margin',
         'near_align_enter', 'near_align_exit', 'close_x_tolerance',
         'stable_frames', 'frame_timeout', 'lost_timeout', 'odom_timeout',
         'max_seconds', 'max_distance', 'calibrations']
cfg['tracking_pick'] = {k: tp[k] for k in order if k in tp}

with io.open(PATH, 'w', encoding='utf-8', newline='\n') as f:
    json.dump(cfg, f, ensure_ascii=False, indent=2)
print({k: tp[k] for k in order if k in tp and k != 'calibrations'})
