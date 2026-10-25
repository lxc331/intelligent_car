"""Raw LaserScan transport checks, without custom geometry or interpolated ranges."""
import math
import threading


class LaserMonitor:
    TIMEOUT = .55

    def __init__(self):
        self.lock = threading.Lock()
        self.reset()

    def reset(self):
        with self.lock:
            self.stamp = None
            self.sequence = 0
            self.view = dict(received_at=0.,sequence=0,ranges=(),left=None,right=None,nearest=None,
                             error="等待 /scan_raw 雷达数据")

    def update(self, msg, received_at):
        stamp = None
        try:
            header = msg["header"]
            if header.get("frame_id","").lstrip("/")!="lidar_frame":
                raise ValueError("雷达坐标系不是 lidar_frame")
            stamp = (int(header["stamp"]["sec"]),int(header["stamp"]["nanosec"]))
            ranges = msg["ranges"]
            start,increment = float(msg["angle_min"]),float(msg["angle_increment"])
            minimum,maximum = float(msg["range_min"]),float(msg["range_max"])
            if not isinstance(ranges,(list,tuple)) or not 100<=len(ranges)<=10000:
                raise ValueError("雷达点数不完整")
            if not all(math.isfinite(v) for v in (start,increment,minimum,maximum,received_at)):
                raise ValueError("雷达参数不是有限数值")
            if increment<=0 or not 5.5<=increment*(len(ranges)-1)<=6.5 or not 0<=minimum<maximum<=10000:
                raise ValueError("需要完整的正角度增量扫描及有效量程")
            clean = tuple(float(v) if isinstance(v,(int,float)) and not isinstance(v,bool)
                          and math.isfinite(v) and 0<v<=maximum else None for v in ranges)
            count = int(math.radians(45.)/increment+.5)
            left = [v for v in clean[:count] if v is not None]
            right = [v for v in clean[::-1][:count] if v is not None]
            good = [v for v in clean if v is not None]
            view = dict(received_at=received_at,error="",ranges=clean,angle_min=start,angle_increment=increment,
                        range_min=minimum,range_max=maximum,left=min(left) if left else None,
                        right=min(right) if right else None,nearest=min(good) if good else None)
        except (KeyError,ValueError,TypeError,OverflowError) as exc:
            view = dict(received_at=received_at,ranges=(),left=None,right=None,nearest=None,
                        error="无效雷达数据："+str(exc))
        with self.lock:
            if stamp is not None and stamp==self.stamp:
                return False
            self.stamp = stamp
            self.sequence += 1
            self.view = dict(view,sequence=self.sequence)
        return True

    def snapshot(self):
        with self.lock:
            return dict(self.view)
