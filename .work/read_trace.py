import json, io
d = json.load(io.open(r"app\捕获\追踪夹取\20261030_195602\运行记录.json", encoding="utf-8"))
tr = d["trace"]
for e in tr[-14:]:
    t = e.get("target") or {}
    print("%7.3f %-6s y=%.4f w=%.4f vx=%.3f wz=%.3f dist=%.3f frame=%s" % (
        e["elapsed"], e["phase"], t.get("y", -1), t.get("width", -1), e["vx"], e["wz"], e["distance"], e["frame_id"]))
