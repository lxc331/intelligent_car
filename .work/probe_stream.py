"""Headless probe: live MJPEG from car 1 + colour detection (no Tk)."""
import io
import sys
import time
import urllib.request

sys.path.insert(0, r"D:\项目\intelligent_car-main\app")
from carapp.core.color_tracker import detect_color  # noqa: E402

HOST = "192.168.167.136"
PORT = 8080
URL = "http://%s:%d/stream?topic=/image_raw&type=mjpeg" % (HOST, PORT)

opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
try:
    resp = opener.open(urllib.request.Request(URL, headers={"User-Agent": "carapp"}), timeout=8)
except Exception as exc:
    print("流连接失败:", type(exc).__name__, exc)
    raise SystemExit(1)

buf = bytearray()
frames = []
deadline = time.time() + 6
while time.time() < deadline and len(frames) < 40:
    chunk = resp.read1(8192)
    if not chunk:
        break
    buf.extend(chunk)
    while True:
        a = buf.find(b"\xff\xd8")
        if a < 0:
            break
        b = buf.find(b"\xff\xd9", a + 2)
        if b < 0:
            del buf[:max(a, 0)]
            break
        data = bytes(buf[a:b + 2])
        del buf[:b + 2]
        frames.append(data)
resp.close()

print("frames:", len(frames))
if not frames:
    raise SystemExit(2)
from PIL import Image  # noqa: E402
img = Image.open(io.BytesIO(frames[-1]))
print("last frame size:", img.size)
open(r"D:\项目\intelligent_car-main\.work\probe_frame.jpg", "wb").write(frames[-1])
for color in ("red", "green", "blue", "yellow"):
    hits = sum(1 for f in frames if detect_color(f, color))
    target = detect_color(frames[-1], color)
    print(color, "frames_with_target=%d/%d" % (hits, len(frames)), "last=", target)
