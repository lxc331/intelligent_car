# -*- coding: utf-8 -*-
"""Runs inside the car container: dump the newest nav session files."""
import glob
import json
import os

dirs = sorted(glob.glob('/tmp/carapp_nav_*'), key=os.path.getmtime, reverse=True)
print('sessions:', len(dirs))
for path in dirs[:3]:
    print(path, 'age_s=%.0f' % (__import__('time').time() - os.path.getmtime(path)))
if not dirs:
    raise SystemExit
root = dirs[0]
print('newest:', root)
for name in sorted(os.listdir(root)):
    print('  %-18s %8d' % (name, os.path.getsize(os.path.join(root, name))))


def tail(name, lines):
    path = os.path.join(root, name)
    if not os.path.exists(path):
        print('--- %s missing' % name)
        return
    with open(path, encoding='utf-8', errors='replace') as stream:
        rows = stream.readlines()
    print('--- %s tail' % name)
    print(''.join(rows[-lines:]).strip()[-6000:])


print('--- state.json')
try:
    with open(os.path.join(root, 'state.json'), encoding='utf-8') as stream:
        print(json.dumps(json.load(stream), ensure_ascii=False, indent=1)[:2000])
except Exception as exc:  # noqa: BLE001
    print('state.json error', exc)
tail('nav2.log', 60)
tail('supervisor.log', 30)
