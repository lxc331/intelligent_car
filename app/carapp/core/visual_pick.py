"""Bounded visual servo control. No Tk, no fixed-time blind approach."""
import math
import threading
import time

MIN_PICK_Y = .50


def validate_pick_reference(target):
    """For the fixed downward preparation pose, pickup must be in the near half."""
    if target["y"] < MIN_PICK_Y:
        raise ValueError("目标还在画面远端，不能标成可夹位置；请放到张开的两爪之间再标定")
    if target["width"] < .06:
        raise ValueError("目标太小，尚未到夹爪可夹位置")
    return target


def validate_calibration(calibration, host, color, pose):
    if not calibration or calibration.get("host") != host or calibration.get("color") != color:
        raise ValueError("本车/本颜色还没有夹取位置标定")
    if calibration.get("pose") != pose:
        raise ValueError("视觉准备姿态已变化，请重新标定")
    for key in ("x", "y", "width"):
        number = float(calibration.get(key, float("nan")))
        if not math.isfinite(number) or not 0 < number < 1:
            raise ValueError("标定数据无效，请重新标定")
    validate_pick_reference(calibration)
    if calibration.get("version") != 2 or not calibration.get("contact_confirmed"):
        raise ValueError("旧标定未确认目标在两爪之间，请重新标定夹取位置")
    return calibration


class VisualServoPolicy:
    def __init__(self, calibration, cfg):
        self.calibration, self.cfg = calibration, cfg
        self.ready_frames = 0
        self.frame_id = -1

    def decide(self, obs, now):
        cfg, cal = self.cfg, self.calibration
        if now-obs.get("seen_at", 0) > cfg["frame_timeout"]:
            self.ready_frames = 0
            return "画面超时", 0., 0.
        target = obs.get("target")
        if not target or obs.get("stable", 0) < cfg["stable_frames"]:
            self.ready_frames = 0
            return "等待稳定目标", 0., 0.
        if target["y"] > cal["y"] + cfg["overshoot_margin"]:
            raise RuntimeError("目标已越过夹取线，请人工调整位置")
        dx = target["x"]-cal["x"]
        if abs(dx) > cfg["x_tolerance"]:
            self.ready_frames = 0
            wz = max(-cfg["max_turn"], min(cfg["max_turn"], -dx*cfg["turn_gain"]))
            return "左右对准", 0., wz
        if target["y"] >= cal["y"]-cfg["y_tolerance"]:
            if not .80 <= target["width"]/cal["width"] <= 1.8:
                raise RuntimeError("目标尺寸与标定不符，请重新确认目标和夹取位置")
            if obs["frame_id"] != self.frame_id:
                self.ready_frames += 1
                self.frame_id = obs["frame_id"]
            return ("到达夹取位置" if self.ready_frames >= cfg["stable_frames"] else "确认夹取位置"), 0., 0.
        self.ready_frames = 0
        speed = min(cfg["max_speed"], max(cfg["min_speed"], (cal["y"]-target["y"])*.20))
        return "接近目标", speed, 0.


class VisualPickRunner:
    def __init__(self, cfg, calibration, actions, observe, odometry, drive, servo,
                 status=lambda text: None, connected=lambda: True, trace=lambda entry: None):
        self.cfg, self.actions = cfg, actions
        self.policy = VisualServoPolicy(calibration, cfg)
        self.observe, self.odometry, self.drive, self.servo = observe, odometry, drive, servo
        self.status, self.connected = status, connected
        self.trace = trace
        self.cancelled = threading.Event()
        self.command_lock = threading.Lock()
        self.result = None

    def cancel(self):
        self.cancelled.set()
        with self.command_lock:
            self.drive(0., 0.)

    def _drive(self, vx, wz):
        with self.command_lock:
            if self.cancelled.is_set():
                return False
            if self.drive(vx, wz) is False:
                raise RuntimeError("运动指令发送失败")
            return True

    def _servo(self, targets, duration):
        with self.command_lock:
            if self.cancelled.is_set():
                return False
            if self.servo(targets, duration) is False:
                raise RuntimeError("舵机指令发送失败")
        return not self.cancelled.wait(duration+.15)

    def run(self):
        started, missing_since, last_pose = time.monotonic(), None, None
        distance, phase = 0., None
        try:
            while not self.cancelled.is_set():
                now = time.monotonic()
                if now-started > self.cfg["max_seconds"]:
                    raise RuntimeError("超过追踪时限，已停车")
                if not self.connected():
                    raise RuntimeError("连接断开，已停止追踪")
                odom = self.odometry()
                if not odom or now-odom.get("seen_at", 0) > self.cfg["odom_timeout"]:
                    raise RuntimeError("里程计无数据或过期，禁止自动接近")
                pose = odom["pose"]
                if last_pose:
                    increment = math.hypot(pose[0]-last_pose[0], pose[1]-last_pose[1])
                    if increment > .3:
                        raise RuntimeError("里程计发生突变，已停车")
                    distance += increment
                last_pose = pose
                if distance > self.cfg["max_distance"]:
                    raise RuntimeError("超过本次最大行驶距离，已停车")
                obs = self.observe()
                new_phase, vx, wz = self.policy.decide(obs, now)
                self.trace({"time": time.time(), "elapsed": now-started, "phase": new_phase,
                            "pose": list(pose), "distance": distance, "vx": vx, "wz": wz,
                            "target": obs.get("target"), "image_age": now-obs.get("seen_at",0),
                            "frame_id": obs.get("frame_id"), "stable": obs.get("stable",0)})
                if new_phase in ("画面超时", "等待稳定目标"):
                    missing_since = now if missing_since is None else missing_since
                    if now-missing_since > self.cfg["lost_timeout"]:
                        raise RuntimeError("目标丢失或图像过期，已停车；重新确认后再启动")
                else:
                    missing_since = None
                if new_phase != phase:
                    phase = new_phase
                    self.status(phase)
                self._drive(vx, wz)
                if phase == "到达夹取位置":
                    self.status("停车，闭合夹爪")
                    if not self._servo({"gripper": self.actions["close"]}, .8):
                        break
                    self.status("抬升夹爪")
                    if not self._servo({"arm": self.actions["arm_up"]}, 1.0):
                        break
                    self.result = "动作完成（请确认是否夹住）"
                    return
                self.cancelled.wait(1./self.cfg["control_hz"])
            self.result = "已中止"
        except Exception as exc:
            self.result = "停止："+str(exc)
        finally:
            with self.command_lock:
                self.drive(0., 0.)
            self.status(self.result or "已停止")
