import io
import json

path = r"app/config/settings.json"
with io.open(path, encoding="utf-8") as stream:
    text = stream.read()
cfg = json.loads(text)
nav = cfg.setdefault("navigation", {})
nav["sandbox_pass_cost"] = 254
with io.open(path, "w", encoding="utf-8", newline="\n") as stream:
    json.dump(cfg, stream, ensure_ascii=False, indent=2)
    stream.write("\n")
print("navigation =", cfg["navigation"])
