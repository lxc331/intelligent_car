"""Live end-to-end check of the zoom detector plus scene policy against 1号车.

用法：
    py -3 .work\live_traffic_zoom_check.py [帧数] [pitch]

默认 20 帧、间隔 0.3 s；先把云台抬到 pitch（默认 1600，None 表示不动），
再用 App 配置里的 zoom 区域跑 TrafficLightDetector，最后喂 TrafficScenePolicy。
"""
import io
import sys
import time
import urllib.request

sys.path.insert(0, r"D:\项目\intelligent_car-main\app")

from carapp.config import DEFAULTS
from carapp.core.traffic_light_detector import TrafficLightDetector
from carapp.core.traffic_scene import TrafficScenePolicy

CAR = "192.168.167.136"
COUNT = int(sys.argv[1]) if len(sys.argv) > 1 else 20
PITCH = None if len(sys.argv) > 2 and sys.argv[2].lower() == "none" else int(sys.argv[2]) if len(sys.argv) > 2 else 1600


def snapshot():
    request = urllib.request.Request("http://%s:8080/snapshot?topic=/image_raw" % CAR,
                                     headers={"Cache-Control": "no-store"})
    with urllib.request.urlopen(request, timeout=5.) as response:
        return response.read()


def tilt():
    import json
    from carapp.core.ws import WebSocket
    sock = WebSocket(CAR, 9090)
    try:
        sock.send_text(json.dumps({"op": "publish", "topic": "/ros_robot_controller/pwm_servo/set_state",
                                   "msg": {"duration": .8,
                                           "state": [{"id": [1], "position": [PITCH], "offset": [0]}]}}))
    finally:
        time.sleep(1.6)
        sock.close()


def main():
    settings = DEFAULTS["traffic_light"]
    detector = TrafficLightDetector(confidence=settings["confidence"], zoom=settings["zoom"],
                                    zoom_confidence=settings.get("zoom_confidence"))
    scene = TrafficScenePolicy(red_area_min=settings["red_area"],
                               red_frames=settings.get("red_frames", 1),
                               red_hold=settings.get("red_hold", 0.))
    if PITCH is not None:
        print("云台 pitch ->", PITCH)
        tilt()
    print("zoom:", detector.zoom, "| red_area_min:", scene.red_area_min)
    reds = 0
    for index in range(COUNT):
        started = time.monotonic()
        frame = snapshot()
        result = detector.detect(frame, "jpeg")
        required = scene.observe(result.objects, result.image_size, time.monotonic())
        state = scene.snapshot()
        reds += 1 if result.red else 0
        print("%2d  red=%-5s boxes=%-22s scene=%-22s active=%-5s required=%-5s %.0f ms" % (
            index, result.red, result.boxes[:2], state["scene"], state["crosswalk_active"],
            required, (time.monotonic() - started) * 1000))
        time.sleep(.3)
    print("检出红灯帧：%d/%d" % (reds, COUNT))
    return 0 if reds else 2


if __name__ == "__main__":
    sys.exit(main())
