"""Global traffic-light control; no page or Tk work in the detector thread."""
import threading
import time

from carapp.core.traffic_light import JunctionViews, TrafficLightController
from carapp.core.traffic_light_detector import TrafficLightDetector
from carapp.core.traffic_scene import TrafficScenePolicy

LOOK_UP_DEFAULTS = dict(confirm_pitch=1720, dip_pitch=1450, restore_pitch=1500,
                        confirm_delay=1.4, confirm_hold=8., confirm_retry=.8,
                        confirm_dip=1.2, confirm_settle=.3)


def _build(cfg):
    """Detector and scene policy from settings; values are validated here."""
    settings = (cfg or {}).get("traffic_light") or {}
    detector = TrafficLightDetector(confidence=settings.get("confidence", .5),
                                    zoom=settings.get("zoom") or (),
                                    zoom_confidence=settings.get("zoom_confidence"),
                                    red_aspect=settings.get("red_aspect"),
                                    candidate_confidence=settings.get("candidate_confidence"))
    scene = TrafficScenePolicy(red_area_min=settings.get("red_area", 700.),
                               red_frames=settings.get("red_frames", 1),
                               red_hold=settings.get("red_hold", 0.),
                               green_area_min=settings.get("green_area", 300.),
                               release_green_frames=settings.get("release_green_frames", 0),
                               release_timeout=settings.get("release_timeout", 12.))
    return detector, scene


def _build_look(cfg):
    """Look-up sequencer from settings; a broken value falls back to defaults."""
    settings = (cfg or {}).get("traffic_light") or {}
    values = {key: settings.get(key, default) for key, default in LOOK_UP_DEFAULTS.items()}
    return JunctionViews(values["confirm_pitch"], values["dip_pitch"],
                         values["restore_pitch"], values["confirm_delay"],
                         values["confirm_hold"], values["confirm_retry"],
                         values["confirm_dip"], values["confirm_settle"])


class TrafficLightControl:
    """Owns the interlock, the look-up sequence and the camera/light hand-off.

    The sandbox light sits almost overhead once the car stops at the crosswalk,
    so the forward view no longer contains the lamp. A red light seen ahead
    first dips the camera to confirm the crosswalk, then lifts it to watch the
    lamp while the car holds; a confirmed crosswalk without a red light still
    lifts the camera to read the lamp. Only the view moves, never the stop
    decision.
    """
    POLL = .15

    def __init__(self, ctx):
        self.ctx = ctx
        try:
            detector, scene = _build(ctx.cfg)
        except (TypeError, ValueError) as exc:
            self.ctx.log("红绿灯配置无效，改用默认值：%s" % exc)
            detector, scene = TrafficLightDetector(), TrafficScenePolicy()
        try:
            self.look = _build_look(ctx.cfg)
        except (TypeError, ValueError) as exc:
            self.ctx.log("红绿灯抬头参数无效，改用默认值：%s" % exc)
            self.look = JunctionViews()
        self.controller = TrafficLightController(ctx.stop_base, self._changed,
                                                 detector=detector, scene=scene)
        self.halt = threading.Event()
        self.thread = threading.Thread(target=self._run_look, name="traffic-light-look",
                                       daemon=True)
        self.thread.start()

    def _changed(self, state):
        self.ctx.state.update(traffic_light=state)
        if state["enabled"]:
            self.ctx.log("红绿灯功能：" + state["reason"] + ("；" + state["error"] if state.get("error") else ""))

    def _run_look(self):
        # Independent of decoding: the camera is moved by scene state alone, so a
        # slow frame can neither postpone nor trigger a tilt.
        while not self.halt.wait(self.POLL):
            try:
                self._apply_look()
            except Exception as exc:
                self.ctx.log("红绿灯抬头失败：%s" % exc)

    def _apply_look(self):
        state = self.controller.snapshot()
        if not state.get("enabled"):
            if self.look.lifted:
                self.look.reset()
                self.ctx.publish_servo({"pitch": self.look.restore})
            return
        action, notice = self.look.step(bool(state.get("red_pending")),
                                        bool(state.get("crosswalk_active")),
                                        time.monotonic())
        if notice:
            self.ctx.log(notice)
        if action == JunctionViews.LIFT:
            self.ctx.publish_servo({"pitch": self.look.pitch})
        elif action == JunctionViews.DIP:
            self.ctx.publish_servo({"pitch": self.look.dip})
        elif action == JunctionViews.RESTORE:
            self.ctx.publish_servo({"pitch": self.look.restore})

    def _lower_camera(self):
        if self.look.lifted:
            self.look.reset()
            self.ctx.publish_servo({"pitch": self.look.restore})

    def toggle(self):
        enabled = not self.snapshot()["enabled"]
        try:
            self.controller.set_enabled(enabled)
        except (ImportError, FileNotFoundError, ValueError) as exc:
            self.ctx.log("红绿灯功能未开启：" + str(exc))
            return
        if not enabled:
            self._lower_camera()
        self.ctx.log("红绿灯功能：" + ("已开启；斑马线路口减速、路口内红灯暂停，连续未见红灯后恢复原任务" if enabled else "已关闭，交通场景判断和减速限制已清空"))

    def snapshot(self):
        return self.controller.snapshot()

    def guard_motion(self, publish):
        return self.controller.guard_motion(publish)

    def guard_twist(self, vx, vy, wz, publish):
        return self.controller.guard_twist(vx, vy, wz, publish)

    def reset(self):
        self._lower_camera()
        self.controller.reset()

    def frame_handler(self):
        return self.controller.frame_handler()

    def close(self):
        self.halt.set()
        self._lower_camera()
        self.controller.close()
        self.thread.join(timeout=1.)
