import sys, time, io
sys.path.insert(0, r"D:\项目\intelligent_car-main\app")
from carapp.core.color_tracker import detect_color

payload = open(r"D:\项目\intelligent_car-main\.work\probe_frame.jpg", "rb").read()
for _ in range(2):
    detect_color(payload, "red")
for color in ("red", "green"):
    t0 = time.perf_counter()
    n = 30
    for _ in range(n):
        detect_color(payload, color)
    dt = (time.perf_counter() - t0) / n
    print(f"{color}: {dt*1000:.1f} ms/frame")
