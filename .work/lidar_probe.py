# -*- coding: utf-8 -*-
"""Read one /scan_raw message and report forward-sector distances (calibration aid)."""
import json
import math
import sys
import time

sys.path.insert(0, r"D:\项目\intelligent_car-main\app")

from carapp.core.ws import WebSocket  # noqa: E402

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.167.136"
HALF_DEG = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0


def main():
    ws = WebSocket(HOST, 9090, timeout=3.0)
    ws.send_text(json.dumps({"op": "subscribe", "topic": "/scan_raw", "id": "scan"}))
    scan = None
    end = time.monotonic() + 8.0
    while time.monotonic() < end and scan is None:
        try:
            raw = ws.recv_text()
        except Exception:
            continue
        try:
            data = json.loads(raw)
        except ValueError:
            continue
        if data.get("topic") == "/scan_raw" and data.get("op") == "publish":
            scan = data.get("msg") or {}
    ws.close()
    if not scan:
        print("no /scan_raw message within 8 s")
        return 1
    angle_min = float(scan.get("angle_min", -math.pi))
    angle_increment = float(scan.get("angle_increment", 0.0))
    ranges = scan.get("ranges") or []
    rmin = float(scan.get("range_min", 0.0))
    rmax = float(scan.get("range_max", 0.0))
    print("beams=%d angle_min=%.3f inc=%.5f range=[%.2f, %.2f]"
          % (len(ranges), angle_min, angle_increment, rmin, rmax))

    def normalize(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    half = math.radians(HALF_DEG)
    forward = []
    for index, value in enumerate(ranges):
        if not isinstance(value, (int, float)):
            continue
        distance = float(value)
        if not math.isfinite(distance) or distance < rmin or distance > rmax:
            continue
        angle = normalize(angle_min + index * angle_increment)
        if abs(angle) <= half:
            forward.append((angle, distance))
    if not forward:
        print("no valid beam inside +-%.0f deg" % HALF_DEG)
        return 1
    forward.sort(key=lambda item: item[1])
    nearest = forward[0]
    print("forward sector +-%.0f deg: %d beams, nearest %.3f m at %.1f deg"
          % (HALF_DEG, len(forward), nearest[1], math.degrees(nearest[0])))
    print("nearest five:", [(round(math.degrees(a), 1), round(d, 3)) for a, d in forward[:5]])
    return 0


if __name__ == "__main__":
    sys.exit(main())
