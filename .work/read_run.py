import json, io
p = r"app\捕获\追踪夹取\20261030_195602\运行记录.json"
d = json.load(io.open(p, encoding="utf-8"))
print("top keys:", list(d))
for k in ("host","color","profile_id","result","approach_limits","vehicle_files_modified","created","started","finished"):
    if k in d: print(k, "=", json.dumps(d[k], ensure_ascii=False)[:300])
print("calibration:", json.dumps(d.get("calibration"), ensure_ascii=False))
ev = d.get("events") or []
print("events:", len(ev))
for e in ev[:60]: print("  ", json.dumps(e, ensure_ascii=False)[:220])
tr = d.get("trace") or []
print("trace:", len(tr))
if tr:
    print("first:", json.dumps(tr[0], ensure_ascii=False)[:400])
    print("last:", json.dumps(tr[-1], ensure_ascii=False)[:400])
