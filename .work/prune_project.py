# -*- coding: utf-8 -*-
"""Keep only the biggest drivable component of a sandbox project and re-emit it."""
import hashlib
import json
import random
import shutil
import sys
import uuid
from pathlib import Path

sys.path.insert(0, str(Path("app").resolve()))
from carapp.core.sandbox_project import load_project  # noqa: E402
from carapp.core.sandbox_route import plan_route  # noqa: E402

SRC = Path(sys.argv[1] if len(sys.argv) > 1 else "app/savedMap/sandbox_drawn_20261031")
OUT = Path(sys.argv[2] if len(sys.argv) > 2 else "app/savedMap/sandbox_nav_ready_20261101")

project = load_project(SRC / "sandbox.json")
nodes = project.network.nodes
widths = project.metadata["edge_widths"]
graph = {i: [] for i in nodes}
for i in nodes:
    for j in nodes[i].to:
        if i < j and widths.get("%d:%d" % (i, j), 0.0) >= 0.23:
            graph[i].append(j)
            graph[j].append(i)
seen, comps = set(), []
for s in graph:
    if s in seen:
        continue
    stack, comp = [s], []
    seen.add(s)
    while stack:
        u = stack.pop()
        comp.append(u)
        for v in graph[u]:
            if v not in seen:
                seen.add(v)
                stack.append(v)
    comps.append(comp)
comps.sort(key=len, reverse=True)
main = sorted(comps[0])
remap = {old: new for new, old in enumerate(main)}
print("kept %d of %d nodes" % (len(main), len(nodes)))

if OUT.exists():
    shutil.rmtree(OUT)
OUT.mkdir(parents=True)
for name in ("map.pgm", "map.yaml", "roads.pgm", "roads.yaml", "keepout.pgm", "keepout.yaml"):
    shutil.copy(SRC / name, OUT / name)

adj = {i: [] for i in range(len(main))}
new_widths = {}
for i in main:
    for j in nodes[i].to:
        if i < j and j in remap:
            w = widths.get("%d:%d" % (i, j), 0.0)
            if w < 0.23:
                continue
            a, b = remap[i], remap[j]
            adj[a].append(b)
            adj[b].append(a)
            new_widths["%d:%d" % (a, b)] = round(w, 3)
            new_widths["%d:%d" % (b, a)] = round(w, 3)

lines = ["waypoints:"]
for i in main:
    n = remap[i]
    lines += ["  - id: %d" % n,
              "    pose: [%.4f, %.4f]" % (nodes[i].x, nodes[i].y),
              "    to: [%s]" % ", ".join(str(x) for x in sorted(adj[n])),
              "    name: 路口%d" % n]
(OUT / "road_network.yaml").write_text("\n".join(lines) + "\n", encoding="utf-8")

meta = dict(project.metadata)
files = ["map.pgm", "map.yaml", "roads.pgm", "roads.yaml", "keepout.pgm", "keepout.yaml",
         "road_network.yaml"]
meta["edge_widths"] = new_widths
meta["id"] = uuid.uuid4().hex[:12]
meta["sha256"] = {f: hashlib.sha256((OUT / f).read_bytes()).hexdigest() for f in files}
meta["sources"] = dict(meta.get("sources", {}), note="main drivable component only")
(OUT / "sandbox.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2),
                                  encoding="utf-8")
print("wrote", OUT)

checked = load_project(OUT / "sandbox.json")
ok = fail = 0
rng = random.Random(11)
ids = sorted(checked.network.nodes)
for _ in range(300):
    a, b = rng.choice(ids), rng.choice(ids)
    if a == b:
        continue
    try:
        plan_route(checked, (checked.network.nodes[a].x, checked.network.nodes[a].y, 0.0), b)
        ok += 1
    except ValueError:
        fail += 1
print("routes ok %d fail %d" % (ok, fail))
