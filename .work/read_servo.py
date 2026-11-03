import sys, json
sys.path.insert(0, "app")
from carapp import config as c
cfg = c.load_config()
print("keys:", list(cfg))
print("servo keys:", list(cfg["servo"]))
print(json.dumps(cfg["servo"], ensure_ascii=False)[:1200])
print("DEFAULTS servo:", json.dumps(c.DEFAULTS.get("servo"), ensure_ascii=False)[:800])
