"""Global traffic-light control; no page or Tk work in the detector thread."""
from carapp.core.traffic_light import TrafficLightController


class TrafficLightControl:
    def __init__(self, ctx):
        self.ctx = ctx
        self.controller = TrafficLightController(ctx.stop_base, self._changed)

    def _changed(self, state):
        self.ctx.state.update(traffic_light=state)
        if state["enabled"]:
            self.ctx.log("红绿灯功能：" + state["reason"] + ("；" + state["error"] if state.get("error") else ""))

    def toggle(self):
        enabled = not self.snapshot()["enabled"]
        try:
            self.controller.set_enabled(enabled)
        except (ImportError, FileNotFoundError, ValueError) as exc:
            self.ctx.log("红绿灯功能未开启：" + str(exc))
            return
        self.ctx.log("红绿灯功能：" + ("已开启；斑马线路口减速、路口内红灯暂停，连续未见红灯后恢复原任务" if enabled else "已关闭，交通场景判断和减速限制已清空"))

    def snapshot(self):
        return self.controller.snapshot()

    def guard_motion(self, publish):
        return self.controller.guard_motion(publish)

    def guard_twist(self, vx, vy, wz, publish):
        return self.controller.guard_twist(vx, vy, wz, publish)

    def reset(self):
        self.controller.reset()

    def frame_handler(self):
        return self.controller.frame_handler()

    def close(self):
        self.controller.close()
