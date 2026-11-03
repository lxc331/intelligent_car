import json, io
d = json.load(io.open(r"app\捕获\追踪夹取\20261030_200820\运行记录.json", encoding="utf-8"))
print("result:", d["result"])
print("calibration:", json.dumps(d["calibration"], ensure_ascii=False))
print("limits:", d["approach_limits"])
tr = d["trace"]
print("trace:", len(tr))
for e in tr[-16:]:
    t = e.get("target") or {}
    print("%7.3f %-8s x=%.4f y=%.4f w=%.4f vx=%.3f wz=%.3f dist=%.3f" % (
        e["elapsed"], e["phase"], t.get("x", -1), t.get("y", -1), t.get("width", -1), e["vx"], e["wz"], e["distance"]))
