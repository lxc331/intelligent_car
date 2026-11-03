# -*- coding: utf-8 -*-
import pathlib

src = pathlib.Path('.work/build_from_track3.py').read_text(encoding='utf-8')
old = """    for left, right in zip(keys, keys[1:]):
        builder.fill(left, right)"""
new = """    index = 0
    while index < len(keys) - 1:
        length = None
        for step in range(1, 5):
            target = index + step
            if target >= len(keys):
                break
            length, width = builder.chord(keys[index], keys[target])
            if length:
                builder.links.append((keys[index], keys[target], width))
                index = target
                break
        if length is None:
            builder.failed.append((keys[index], keys[index + 1], 0., 1))
            index += 1"""
assert old in src
src = src.replace(old, new)
src = src.replace('track_network3.png', 'track_network5.png')
src = src.replace('BACKUP = ROOT / ".work" / "backup_track_build3"',
                  'BACKUP = ROOT / ".work" / "backup_track_build5"')
pathlib.Path('.work/build_from_track5.py').write_text(src, encoding='utf-8')
print('patched ok')
