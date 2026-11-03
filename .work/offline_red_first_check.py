"""Offline replay of the red-first sequence on saved live frames (no car).

Feeds the same detector / scene policy / view sequencer as the App:
  forward view with a red light (no crosswalk in view)
  -> dip view with the crosswalk on the stop line
  -> lift view with the red light and the crosswalk
and checks that the stop is armed only once the dip confirmed the crosswalk.
"""
import sys
from pathlib import Path

ROOT = Path(r"D:\项目\intelligent_car-main")
sys.path.insert(0, str(ROOT / "app"))

from carapp.config import DEFAULTS
from carapp.core.traffic_light import JunctionViews
from carapp.core.traffic_light_detector import TrafficLightDetector
from carapp.core.traffic_scene import TrafficScenePolicy

LIVE = ROOT / ".work" / "traffic_light_live"
FORWARD = LIVE / "now4.jpg"        # forward view, red light high in frame
DIP = LIVE / "stop_pitch_1450.jpg"  # pitched down, crosswalk on the stop line
LIFT = LIVE / "stop_lookup_1720.jpg"  # lifted, red light in the bottom band

SEQUENCE = [(FORWARD, "前视"), (FORWARD, "前视"), (FORWARD, "前视"),
            (DIP, "俯视"), (DIP, "俯视"), (DIP, "俯视"),
            (LIFT, "仰视"), (LIFT, "仰视"), (LIFT, "仰视")]


def main():
    settings = DEFAULTS["traffic_light"]
    detector = TrafficLightDetector(confidence=settings["confidence"],
                                    zoom=settings["zoom"],
                                    zoom_confidence=settings["zoom_confidence"])
    scene = TrafficScenePolicy(red_area_min=settings["red_area"],
                               red_frames=settings["red_frames"],
                               red_hold=settings["red_hold"])
    views = JunctionViews(settings["confirm_pitch"], settings["dip_pitch"],
                          settings["restore_pitch"], settings["confirm_delay"],
                          settings["confirm_hold"], settings["confirm_retry"],
                          settings["confirm_dip"])
    pitch = settings["restore_pitch"]
    stopped = False
    now = 100.
    for index, (path, name) in enumerate(SEQUENCE):
        payload = path.read_bytes()
        result = detector.detect(payload, "jpeg")
        blocked = scene.observe(result.objects, result.image_size, now)
        state = scene.snapshot()
        action, notice = views.step(state["red_pending"], state["crosswalk_active"], now)
        if action == JunctionViews.DIP:
            pitch = views.dip
        elif action == JunctionViews.LIFT:
            pitch = views.pitch
        elif action == JunctionViews.RESTORE:
            pitch = views.restore
        stopped = stopped or blocked
        print("%d %-4s pitch=%d red=%-5s area=%-5.0f pending=%-5s active=%-5s "
              "blocked=%-5s %-16s %s %s" % (
                  index, name, pitch, state["scene_red_visible"],
                  state["scene_red_area"], state["red_pending"],
                  state["crosswalk_active"], blocked, state["scene"],
                  action or "-", notice))
        now += .3
    print("\nRESULT stopped=%s final_view=%s final_pitch=%d" % (stopped, views.view, pitch))


if __name__ == "__main__":
    main()
