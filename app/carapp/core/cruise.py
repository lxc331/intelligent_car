"""Vendor lidar modes with PC-side cancellation and connection/scan watchdogs."""
import math
import threading
import time
from dataclasses import dataclass
from carapp.core.laser_scan import LaserMonitor
from carapp.core.vendor_lidar import MODE_NAMES, VendorLidarAlgorithm


@dataclass(frozen=True)
class CruiseSettings:
    detection_distance: float = .60
    speed: float = .06
    mode: str = "avoid"
    scan_angle: float = 90.
    lidar_type: str = "standard"

    def __post_init__(self):
        for value in (self.detection_distance,self.speed,self.scan_angle):
            if not isinstance(value,(float,int)) or isinstance(value,bool) or not math.isfinite(value):
                raise ValueError("巡航参数须为有限数值")
        if not .30<=self.detection_distance<=1.50:
            raise ValueError("检测距离须为 0.30–1.50 米（原范围）")
        if not .02<=self.speed<=.15:
            raise ValueError("移动速度须为 0.02–0.15 米/秒")
        if not 0<=self.scan_angle<=90:
            raise ValueError("检测角度须为 0–90°")
        if self.mode not in MODE_NAMES or not isinstance(self.lidar_type,str):
            raise ValueError("雷达模式或型号无效")


class CruisePolicy:
    """Update PID once per fresh scan, retaining the vendor's command between scans."""
    def __init__(self, settings=None, clock=None):
        self.settings = self.algorithm = self.last_sequence = self.last_scan = None
        self.phase = "待机"
        self.waiting_for_scan = False
        self.clock = clock or time.monotonic
        if settings:
            self._configure(settings,self.clock())

    def _configure(self, settings, now):
        self.settings = settings
        self.algorithm = VendorLidarAlgorithm(**settings.__dict__,now=now)
        self.last_sequence = None

    @property
    def diagnostics(self):
        return dict(self.algorithm.diagnostics,command=self.algorithm.command,
                    scan_sequence=self.last_sequence) if self.algorithm else {}

    def wait_for_scan(self):
        self.waiting_for_scan = True
        if self.algorithm:
            self.algorithm.command = (0.,0.)
        self.phase = "雷达数据暂时中断，已暂停行驶；新数据恢复后自动继续"
        return 0.,0.,self.phase,False

    def decide(self, scan, settings, now):
        self.last_scan = scan
        age = now-scan.get("received_at",0.)
        if age<-.01:
            return 0.,0.,"雷达接收时间无效，已停止，请重新启动",True
        if age>LaserMonitor.TIMEOUT:
            return self.wait_for_scan()
        if scan.get("error"):
            return 0.,0.,scan["error"],True
        if settings!=self.settings or self.waiting_for_scan:
            self._configure(settings,now)
            self.waiting_for_scan = False
        if scan["sequence"]!=self.last_sequence:
            try:
                self.algorithm.update(scan,now)
            except (ValueError,KeyError,TypeError,ZeroDivisionError,OverflowError) as exc:
                self.algorithm.command = (0.,0.)
                return 0.,0.,str(exc)+"，已停止",True
            self.last_sequence = scan["sequence"]
        vx,wz = self.algorithm.command
        self.phase = self.algorithm.phase
        return vx,wz,self.phase,False


class CruiseRunner:
    def __init__(self, ctx, monitor, notify=lambda text:None, clock=time.monotonic):
        self.ctx,self.monitor,self.notify,self.clock = ctx,monitor,notify,clock
        self.lock = threading.RLock()
        self.cancel = threading.Event()
        self.thread = None
        self.running = False
        self.settings = CruiseSettings()
        self.phase = "待机"
        self.policy = CruisePolicy()

    def _phase(self, text):
        if text!=self.phase:
            self.phase = text
            self.notify(text)

    def configure(self, settings):
        with self.lock:
            self.settings = settings
            self.policy = CruisePolicy(settings,clock=self.clock)
            if self.running:
                self.ctx.stop_base()
                self._phase("参数已更新，重新评估雷达")

    def start(self, settings):
        with self.lock:
            if self.running or (self.thread and self.thread.is_alive()):
                raise ValueError("巡航已启动或上次动作正在停止")
            if not self.ctx.state.get("ws_connected"):
                raise ValueError("ROS 尚未连接")
            if not self.ctx.state.get("cam_connected"):
                raise ValueError("请等待实时画面就绪后再启动巡航")
            probe = CruisePolicy()
            check = probe.decide(self.monitor.snapshot(),settings,self.clock())
            if probe.waiting_for_scan:
                raise ValueError("请等待有效的新雷达数据后再启动巡航")
            if check[3]:
                raise ValueError(check[2])
            self.settings,self.policy = settings,CruisePolicy(settings,clock=self.clock)
            self.cancel = threading.Event()
            self.running = True
            self.ctx.stop_base()
            self._phase(MODE_NAMES[settings.mode]+"已启动")
            self.thread = threading.Thread(target=self._run,name="cruise-lidar",daemon=True)
            self.thread.start()

    def step(self):
        with self.lock:
            if not self.running or self.cancel.is_set():
                return
            if not self.ctx.state.get("ws_connected") or not self.ctx.bridge.connected:
                decision = (0.,0.,"ROS 连接中断，已取消巡航",True)
            elif not self.ctx.state.get("cam_connected"):
                decision = (0.,0.,"实时画面中断，已取消巡航",True)
            elif getattr(self.ctx, "motion_interlock", lambda: {})().get("blocked"):
                # Restart PID/turn timing from live lidar after the light clears;
                # time spent parked must not advance an old avoidance maneuver.
                self.policy.wait_for_scan()
                decision = (0.,0.,"红绿灯暂停，连续未见红灯后自动继续",False)
            else:
                scan = self.monitor.snapshot()
                decision = self.policy.decide(scan,self.settings,self.clock())
                if not decision[3] and self.clock()-scan.get("received_at",0.)>LaserMonitor.TIMEOUT:
                    decision = self.policy.wait_for_scan()
            vx,wz,text,fatal = decision
            if fatal:
                self.running = False
                self.cancel.set()
                self.ctx.stop_base()
            elif not self.ctx.publish_twist(vx,wz,quiet=True):
                self.running = False
                self.cancel.set()
                self.ctx.stop_base()
                text = "底盘指令发送失败，已取消巡航"
            self._phase(text)

    def stop(self, reason="已停止巡航"):
        self.cancel.set()
        with self.lock:
            self.running = False
            self.ctx.stop_base()
            self._phase(reason)
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=.6)

    def _run(self):
        try:
            while not self.cancel.is_set():
                started = time.monotonic()
                self.step()
                self.cancel.wait(max(0.,.10-(time.monotonic()-started)))
        except Exception as exc:
            with self.lock:
                self.running = False
                self.cancel.set()
                self._phase("巡航异常，已停止："+str(exc))
        finally:
            self.ctx.stop_base()
