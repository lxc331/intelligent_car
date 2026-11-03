import json, sys
sys.path.insert(0, "app")
from carapp import config as c
cfg = c.load_config()
t = cfg["tracking_pick"]
print("arrive_margin=%r straight_margin=%r near_x_tolerance=%r" % (
    t.get("arrive_margin"), t.get("straight_margin"), t.get("near_x_tolerance")))
print("json ok:", isinstance(json.load(open("app/config/settings.json", encoding="utf-8")), dict))
