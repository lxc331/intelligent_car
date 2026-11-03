# -*- coding: utf-8 -*-
import pathlib

for name in ('.work/build_from_track6.py', '.work/repair_track_network6.py'):
    path = pathlib.Path(name)
    text = path.read_text(encoding='utf-8')
    text = text.replace('FOOTPRINT = (.25, .15, .115)', 'FOOTPRINT = (.20, .10, .09)')
    text = text.replace('WIDTH_MIN = .23', 'WIDTH_MIN = .20')
    text = text.replace('THRESHOLD = 65', 'THRESHOLD = 65')
    path.write_text(text, encoding='utf-8')
    print(name, 'ok')
