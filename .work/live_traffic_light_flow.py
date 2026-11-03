"""Live end-to-end rehearsal of the traffic-light flow (no Tk, no car motion).

Mirrors carapp.features.traffic_light: the same detector, the same scene policy
and the same look-up sequencer run against the car camera, and the pitch is
published through rosbridge exactly like the App does. Nothing is sent to the
wheels: this proves the camera/light hand-off and the red decision only.
"""
import json
import sys
import time
import urllib.request

sys.path.insert(0, r"D:\项目\intelligent_car-main\app")

from carapp.config import DEFAULTS
from carapp.core.traffic_light import JunctionViews
from carapp.core.ws import WebSocket
from carapp.features.traffic_light import _build, _build_look

FRAMES = int(sys.argv[1]) if len(sys.argv) > 1 else 40
HOST = sys.argv[2] if len(sys.argv) > 2 else "192.168.167.136"
URL = "http://%s:8080/snapshot?topic=/image_raw" % HOST


def snapshot():
    request = urllib.request.Request(URL, headers={"Cache-Control": "no-store"})
    return urllib.request.urlopen(request, timeout=5.).read()


def main():
    # Same construction as the App feature, so the rehearsal cannot drift.
    detector, scene = _build(DEFAULTS)
    look = _build_look(DEFAULTS)
    ws = WebSocket(HOST, 9090)

    def pitch(value):
        ws.send_text(json.dumps({"op": "publish",
                                 "topic": "/ros_robot_controller/pwm_servo/set_state",
                                 "msg": {"duration": .8,
                                         "state": [{"id": [1], "position": [value], "offset": [0]}]}}))

    pitch(look.restore)
    time.sleep(1.2)
    reds = 0
    lifted = 0
    for index in range(FRAMES):
        payload = snapshot()
        started = time.monotonic()
        result = detector.detect(payload, "jpeg")
        received = time.monotonic()
        red = scene.observe(result.objects, result.image_size, received)
        state = scene.snapshot()
        action, notice = look.step(bool(state["red_pending"]),
                                   state["crosswalk_active"], received)
        if action == JunctionViews.DIP:
            pitch(look.dip)
        elif action == JunctionViews.LIFT:
            pitch(look.pitch)
            lifted += 1
        elif action == JunctionViews.RESTORE:
            pitch(look.restore)
        if notice:
            print("%2d [%.2f] %s" % (index, received, notice))
        reds += 1 if red else 0
        print("%2d red=%-5s active=%-5s area=%-5.0f %-18s %.0fms %s" % (
            index, red, state["crosswalk_active"], state["scene_red_area"],
            state["scene"], (time.monotonic() - started) * 1000,
            action or ""))
        time.sleep(.08)
    print("frames=%d red=%d lifts=%d" % (FRAMES, reds, lifted))
    pitch(look.restore)


if __name__ == "__main__":
    main()
