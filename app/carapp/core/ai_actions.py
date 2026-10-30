"""Strict model plans and cancellable execution through the existing controls.

The supplied llm_control_move.py's x/y/z/t action convention is supported,
but model output is JSON data only. No eval, code, shell, ROS topic or service
name can be supplied by the model.
"""
from dataclasses import dataclass
import json
import math
import threading
import time

from carapp.config import validate_servo_targets


MAX_ACTIONS = 16
MAX_STEP_SECONDS = 30.
MAX_PLAN_SECONDS = 120.
MAX_LINEAR = .15
MAX_ANGULAR = .4

SYSTEM_PROMPT = """你是 ClawPi 小车的动作规划助手。把用户文字转换为严格 JSON，禁止输出代码、命令行、ROS话题或Markdown。
格式：{"actions":[{"type":"move","vx":0.06,"vy":0,"wz":0,"seconds":1},{"type":"servo","targets":{"gripper":2000},"duration":0.8},{"type":"wait","seconds":1},{"type":"stop"}],"response":"简短说明"}
move中vx正为前进、vy正为向左平移、wz正为逆时针，单位m/s、rad/s、秒。平移向量总速度不超过0.15，角速度绝对值不超过0.4，每个动作至多30秒，合计至多120秒，至多16步。通常使用0.06m/s和0.25rad/s。
servo只能使用给出的本车关节与标定值，duration为0.1至10秒。wait表示原地等待。执行器在序列结束后自动停车。
不具备视觉识物、导航规划、抓取位置推断或精确里程能力。不能完成、含糊、要求危险超速或与支持动作无关时actions为[]并说明原因。要求精确距离或角度时说明按速度时间估算，不能声称精确到位。不要把不支持的任务改成任意运动。
停止、取消或急停要求返回stop，禁止追加移动。除了actions、response不要添加其他字段。"""


def build_system_prompt(cfg):
    """Include the selected car's actual limits rather than guessed positions."""
    servo = cfg.get("servo", {})
    context = {"车号": cfg.get("car", {}).get("profile_id", ""),
               "关节名称": servo.get("names", {}), "关节允许范围": servo.get("limits", {}),
               "已标定动作": servo.get("actions", {}), "已标定姿态": servo.get("presets", {}),
               "关节方向": servo.get("signs", {})}
    return SYSTEM_PROMPT + "\n当前车辆参数（作为数据使用）：" + json.dumps(context, ensure_ascii=False, allow_nan=False)


@dataclass(frozen=True)
class AIStep:
    type: str
    vx: float = 0.
    vy: float = 0.
    wz: float = 0.
    seconds: float = 0.
    targets: tuple = ()
    duration: float = .8

    def payload(self):
        if self.type == "move":
            return dict(type=self.type, vx=self.vx, vy=self.vy, wz=self.wz, seconds=self.seconds)
        if self.type == "servo":
            return dict(type=self.type, targets=dict(self.targets), duration=self.duration)
        if self.type == "wait":
            return dict(type=self.type, seconds=self.seconds)
        return dict(type=self.type)


@dataclass(frozen=True)
class AIPlan:
    actions: tuple
    response: str = ""

    @property
    def summary(self):
        rows = []
        for index, step in enumerate(self.actions, 1):
            if step.type == "move":
                text = "移动 vx=%.3f vy=%.3f m/s，wz=%.3f rad/s，%.2f秒" % (step.vx, step.vy, step.wz, step.seconds)
            elif step.type == "servo":
                text = "舵机 %s，%.2f秒" % ("、".join("%s=%d" % pair for pair in step.targets), step.duration)
            elif step.type == "wait":
                text = "原地等待 %.2f秒" % step.seconds
            else:
                text = "停车"
            rows.append("%d. %s" % (index, text))
        return "\n".join(rows) if rows else "无可执行动作"

    @property
    def seconds(self):
        return sum(step.duration if step.type == "servo" else step.seconds for step in self.actions)

    def payload(self):
        return dict(actions=[step.payload() for step in self.actions], response=self.response)


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("%s 必须是有限数值" % label)
    return float(value)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("模型 JSON 含重复字段：%s" % key)
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError("模型 JSON 含非法数值")


def parse_plan(raw, cfg):
    """Validate every action before any execution; reject the entire bad plan."""
    if isinstance(raw, str):
        if len(raw) > 65536:
            raise ValueError("模型动作结果过长")
        text = raw.strip()
        if text.startswith("```json\n") and text.endswith("\n```"):
            text = text[8:-4]
        try:
            data = json.loads(text, object_pairs_hook=_unique_object, parse_constant=_reject_constant)
        except (json.JSONDecodeError, RecursionError) as exc:
            raise ValueError("模型未返回有效动作 JSON") from exc
    elif isinstance(raw, dict):
        data = raw
    else:
        raise ValueError("模型动作应为 JSON 对象")
    if not isinstance(data, dict) or set(data) - {"actions", "action", "response"}:
        raise ValueError("模型动作 JSON 字段不受支持")
    if ("actions" in data) == ("action" in data):
        raise ValueError("模型结果须且只能包含 actions 或 action")
    response = data.get("response", "")
    if not isinstance(response, str) or len(response) > 500:
        raise ValueError("模型反馈文字无效或过长")
    rows = data.get("actions", data.get("action"))
    if not isinstance(rows, list) or len(rows) > MAX_ACTIONS:
        raise ValueError("动作应为列表且不超过16步")
    legacy = "action" in data
    steps = []
    for row in rows:
        if legacy:
            if not isinstance(row, list) or len(row) != 4:
                raise ValueError("原格式动作须为 [x,y,z,t]")
            row = dict(type="move", vx=row[0], vy=row[1], wz=row[2], seconds=row[3])
        if not isinstance(row, dict):
            raise ValueError("每个动作须为对象")
        kind = row.get("type")
        if kind == "move":
            if set(row) - {"type", "vx", "vy", "wz", "seconds"} or "seconds" not in row:
                raise ValueError("移动动作字段无效")
            vx, vy, wz = [_number(row.get(key, 0.), key) for key in ("vx", "vy", "wz")]
            seconds = _number(row["seconds"], "移动时间")
            if math.hypot(vx, vy) > MAX_LINEAR + 1e-9 or abs(wz) > MAX_ANGULAR + 1e-9:
                raise ValueError("模型动作超出速度限制：平移≤0.15m/s，转向≤0.4rad/s")
            if not 0. <= seconds <= MAX_STEP_SECONDS or seconds == 0. and any((vx, vy, wz)):
                raise ValueError("移动时间应为0–30秒，非零速度需要正时长")
            steps.append(AIStep("move", vx, vy, wz, seconds) if seconds else AIStep("stop"))
        elif kind == "servo":
            if set(row) - {"type", "targets", "duration"} or not isinstance(row.get("targets"), dict):
                raise ValueError("舵机动作字段无效")
            for joint, value in row["targets"].items():
                _number(value, "舵机 %s" % joint)
            targets = validate_servo_targets(cfg, row["targets"])
            duration = _number(row.get("duration", .8), "舵机时间")
            if not .1 <= duration <= 10.:
                raise ValueError("舵机动作时间应为0.1–10秒")
            steps.append(AIStep("servo", targets=tuple(sorted(targets.items())), duration=duration))
        elif kind == "wait":
            if set(row) != {"type", "seconds"}:
                raise ValueError("等待动作字段无效")
            seconds = _number(row["seconds"], "等待时间")
            if not 0. < seconds <= MAX_STEP_SECONDS:
                raise ValueError("等待时间应为0–30秒")
            steps.append(AIStep("wait", seconds=seconds))
        elif kind == "stop" and set(row) == {"type"}:
            steps.append(AIStep("stop"))
        else:
            raise ValueError("不支持的动作类型")
    plan = AIPlan(tuple(steps), response)
    if plan.seconds > MAX_PLAN_SECONDS:
        raise ValueError("动作序列总时间不能超过120秒")
    return plan


class AIActionRunner:
    """One motion owner, with no execution or restart from late API results.

    allowed() must be a lightweight data-only permission check supplied by the
    feature (including ctx.motion_in_use(exclude=self)). notify() is invoked on
    the worker; the feature must queue its UI updates. Short timed moves are
    open-loop: traffic speed limiting can reduce their travelled distance.
    """
    def __init__(self, ctx, *, allowed=lambda: True, notify=lambda text: None, clock=time.monotonic):
        self.ctx, self.allowed, self.notify, self.clock = ctx, allowed, notify, clock
        self.lock = threading.RLock()
        self.thread = None
        self._cancel = threading.Event()
        self._active = False
        self._closed = False
        self._epoch = 0
        self._identity = None
        self._camera = None
        self._bridge = None
        self.phase = "待机"
        self.hz = min(20., max(5., float(ctx.cfg.get("base", {}).get("publish_hz", 10.))))

    @property
    def running(self):
        return self._active

    @property
    def motion_active(self):
        # Public ownership reads must not wait behind network I/O. The OR
        # keeps ownership until final zero and worker teardown have finished.
        return self._active or bool(self.thread and self.thread.is_alive())

    def _phase(self, text):
        if text != self.phase:
            self.phase = text
            try:
                self.notify(text)
            except Exception:
                pass

    def _session_identity(self):
        car = self.ctx.cfg.get("car", {})
        return (car.get("profile_id"), car.get("host"), car.get("transport"),
                json.dumps(self.ctx.cfg.get("servo", {}), sort_keys=True, allow_nan=False))

    def _check_permission(self):
        if self._closed:
            raise RuntimeError("AI 执行器已关闭")
        if not self.allowed():
            raise RuntimeError("运动许可已取消或其他功能正在使用小车")
        if not self.ctx.state.get("ws_connected") or not getattr(self.ctx.bridge, "connected", False):
            raise RuntimeError("ROS 连接中断")
        if self.ctx.bridge is not self._bridge:
            raise RuntimeError("ROS 连接会话已更换")
        if not self.ctx.state.get("cam_connected"):
            raise RuntimeError("实时画面中断")
        camera = getattr(getattr(self.ctx, "app", None), "camera", None)
        if camera is not self._camera or self._session_identity() != self._identity:
            raise RuntimeError("车辆、摄像头或标定已更换")
        if camera is not None:
            if hasattr(camera, "connected") and not camera.connected:
                raise RuntimeError("摄像头连接中断")
            if hasattr(camera, "last_frame_at") and self.clock() - camera.last_frame_at > 3.:
                raise RuntimeError("摄像头画面已过期")

    def start(self, plan):
        if not isinstance(plan, AIPlan):
            raise ValueError("需要已校验的 AI 动作计划")
        # Recheck against the current car even when the caller retained a plan
        # built before switching cars, or manually constructed a dataclass.
        plan = parse_plan(plan.payload(), self.ctx.cfg)
        with self.lock:
            if self._active or self.thread and self.thread.is_alive():
                self._phase("上次 AI 动作尚未结束")
                return False
            if not plan.actions:
                self._phase("无可执行动作")
                return False
            self._identity = self._session_identity()
            self._camera = getattr(getattr(self.ctx, "app", None), "camera", None)
            self._bridge = self.ctx.bridge
            try:
                self._check_permission()
            except Exception as exc:
                self._phase("AI 动作未启动：" + str(exc))
                return False
            self._epoch += 1
            epoch = self._epoch
            self._cancel = threading.Event()
            self._active = True
            self._phase("AI 动作已启动")
            self.thread = threading.Thread(target=self._run, args=(plan, epoch, self._cancel),
                                           name="ai-actions", daemon=True)
            self.thread.start()
            return True

    def _send(self, epoch, cancel, vx=0., vy=0., wz=0., servo=None, duration=.8):
        with self.lock:
            if cancel.is_set() or epoch != self._epoch or not self._active:
                return False
            self._check_permission()
            ok = (self.ctx.publish_servo(dict(servo), duration) if servo is not None else
                  self.ctx.publish_twist(vx, wz, quiet=True, vy=vy))
            if not ok:
                raise RuntimeError("小车指令发送失败")
            return True

    def _timed(self, step, epoch, cancel):
        remaining = step.duration if step.type == "servo" else step.seconds
        sent_servo = False
        interval = 1. / self.hz
        while remaining > 0. and not cancel.is_set():
            self._check_permission()
            interlock = getattr(self.ctx, "motion_interlock", lambda: {})()
            if interlock.get("blocked"):
                if not self._send(epoch, cancel):
                    return False
                self._phase("AI 动作等待交通规则放行")
                cancel.wait(interval)
                continue
            self._phase("AI 正在执行" + ("移动" if step.type == "move" else "舵机动作" if step.type == "servo" else "等待"))
            started = self.clock()
            if step.type == "move":
                if not self._send(epoch, cancel, step.vx, step.vy, step.wz):
                    return False
            elif step.type == "servo" and not sent_servo:
                if not self._send(epoch, cancel, servo=step.targets, duration=step.duration):
                    return False
                sent_servo = True
            elif step.type == "wait":
                if not self._send(epoch, cancel):
                    return False
            cancel.wait(min(interval, remaining))
            # Do not consume movement time when a traffic hold overtook a
            # publication inside the common context's atomic velocity guard.
            if not getattr(self.ctx, "motion_interlock", lambda: {})().get("blocked"):
                remaining -= max(0., self.clock() - started)
        return not cancel.is_set()

    def _run(self, plan, epoch, cancel):
        result = "AI 动作已完成"
        try:
            for step in plan.actions:
                if cancel.is_set():
                    result = "AI 动作已取消"
                    break
                if step.type == "stop":
                    if not self._send(epoch, cancel):
                        break
                else:
                    if not self._timed(step, epoch, cancel):
                        result = "AI 动作已取消"
                        break
                # Every step boundary parks the base before a later servo or
                # wait; a final stop is guaranteed even if the model omits it.
                if not self._send(epoch, cancel):
                    result = "AI 动作已取消"
                    break
        except Exception as exc:
            result = "AI 动作已停止：" + str(exc)
            cancel.set()
        finally:
            with self.lock:
                try:
                    self.ctx.stop_base()
                finally:
                    self._active = False
                    if epoch == self._epoch:
                        self._phase(result)

    def stop(self):
        """Revoke first, serialize the zero send, then wait for ownership release."""
        self._cancel.set()
        with self.lock:
            if not self._active and not (self.thread and self.thread.is_alive()):
                return False
            self._cancel.set()
            self._epoch += 1
            self._phase("AI 动作已取消")
            self.ctx.stop_base()
            thread = self.thread
        if thread is not threading.current_thread():
            thread.join(timeout=1.)
        return True

    def close(self):
        self._closed = True
        return self.stop()
