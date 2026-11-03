# -*- coding: utf-8 -*-
import pathlib

builder = pathlib.Path('.work/build_from_track5.py').read_text(encoding='utf-8')
builder = builder.replace(
    '        if not swept_clear(self.clearance.grid, ((ax, ay, yaw), (bx, by, yaw)), FOOTPRINT, BLOCKED):\n            return None, width',
    '        segment = ((ax, ay, yaw), (bx, by, yaw))\n'
    '        if not swept_clear(self.clearance.grid, segment, FOOTPRINT, BLOCKED):\n'
    '            return None, width\n'
    '        if not swept_clear(self.map_grid, segment, FOOTPRINT, BLOCKED):\n'
    '            return None, width')
builder = builder.replace('builder = Builder(track, _Clearance(project.keepout.grid))',
                          'builder = Builder(track, _Clearance(project.keepout.grid))\n    builder.map_grid = project.map.grid')
builder = builder.replace('class Builder:\n    def __init__(self, track, clearance):\n        self.track, self.clearance = track, clearance',
                          'class Builder:\n    def __init__(self, track, clearance):\n        self.track, self.clearance = track, clearance\n        self.map_grid = None')
builder = builder.replace('BACKUP = ROOT / ".work" / "backup_track_build5"',
                          'BACKUP = ROOT / ".work" / "backup_track_build6"')
builder = builder.replace('track_network5.png', 'track_network6.png')
builder = builder.replace("BACKUP = ROOT / \".work\" / \"backup_track_build3\"",
                          "BACKUP = ROOT / \".work\" / \"backup_track_build6\"")
builder = builder.replace('    print("plan_route sample ok", ok, "fail", fail)',
                          '    print("plan_route sample ok", ok, "fail", fail)')
pathlib.Path('.work/build_from_track6.py').write_text(builder, encoding='utf-8')

repair = pathlib.Path('.work/repair_track_network.py').read_text(encoding='utf-8')
repair = repair.replace('        if not swept_clear(clearance.grid, ((ax, ay, yaw), (bx, by, yaw)), FOOTPRINT, THRESHOLD):\n            return None',
                        '        segment = ((ax, ay, yaw), (bx, by, yaw))\n'
                        '        if not swept_clear(clearance.grid, segment, FOOTPRINT, THRESHOLD):\n'
                        '            return None\n'
                        '        if not swept_clear(project.map.grid, segment, FOOTPRINT, THRESHOLD):\n'
                        '            return None')
repair = repair.replace('BACKUP = ROOT / ".work" / "backup_track_repair"',
                        'BACKUP = ROOT / ".work" / "backup_track_repair6"')
repair = repair.replace("""    ids = sorted(project.network.nodes)
    ok = fail = 0
    for position, start in enumerate(ids):
        for goal in ids[position + 1:]:
            try:
                plan_route(project, project.network.node(start).pose, goal)
                ok += 1
            except Exception:
                fail += 1
    print("plan pairs ok", ok, "fail", fail, "total", ok + fail)""",
"""    import random
    ids = sorted(project.network.nodes)
    random.seed(7)
    pairs = [(a, b) for a in ids for b in ids if a < b]
    sample = random.sample(pairs, min(120, len(pairs)))
    ok = fail = 0
    for start, goal in sample:
        try:
            plan_route(project, project.network.node(start).pose, goal)
            ok += 1
        except Exception:
            fail += 1
    print("plan sample ok", ok, "fail", fail)""")
pathlib.Path('.work/repair_track_network6.py').write_text(repair, encoding='utf-8')
print('patched both')
