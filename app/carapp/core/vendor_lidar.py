"""Port of ClawPi app/app/lidar_controller.py numerical logic, without ROS or Tk.

The vendor slicing, gains, deadbands, last_act and timed arcs are preserved.
No servo enter/exit service is called. Transport failures are handled by cruise.py.
"""
import math

MODE_NAMES = {"avoid":"雷达避障", "follow":"雷达跟随", "guard":"雷达警卫"}


class VendorPID:
    """sdk.pid recurrence with an explicit monotonic timestamp."""
    def __init__(self, p, i, d, now):
        self.Kp,self.Ki,self.Kd = p,i,d
        self.last_time = now
        self.clear()

    def clear(self):
        # sdk.pid.clear() also retains last_time.
        self.SetPoint = self.PTerm = self.ITerm = self.DTerm = self.last_error = self.output = 0.

    def update(self, feedback, now):
        error = self.SetPoint-feedback
        dt = now-self.last_time
        if dt>=0:
            self.PTerm = self.Kp*error
            self.ITerm = max(-20.,min(20.,self.ITerm+error*dt))
            self.DTerm = (error-self.last_error)/dt if dt>0 else 0.
            self.last_time,self.last_error = now,error
            self.output = self.PTerm+self.Ki*self.ITerm+self.Kd*self.DTerm


def _valid(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value) and value>0


def split_scan(scan, scan_angle=90., lidar_type="standard"):
    """Preserve the standard/G4 array indexing in the vendor callback."""
    ranges,increment = scan["ranges"],scan["angle_increment"]
    if lidar_type!="G4":
        maximum = int(math.radians(120.)/increment)
        left,right = ranges[:maximum],ranges[::-1][:maximum]
    else:
        minimum = int(math.radians(60.)/increment)
        maximum = minimum+int(math.radians(120.)/increment)
        left = ranges[::-1][minimum:maximum][::-1]
        right = ranges[minimum:maximum][::-1]
    count = int(math.radians(scan_angle/2)/increment+.50)
    return tuple(left[:count]),tuple(right[:count])


class VendorLidarAlgorithm:
    def __init__(self, mode="avoid", detection_distance=.6, speed=.06, scan_angle=90.,
                 lidar_type="standard", now=0.):
        if mode not in MODE_NAMES:
            raise ValueError("未知雷达模式")
        self.mode,self.threshold,self.speed = mode,detection_distance,speed
        self.scan_angle,self.lidar_type = scan_angle,lidar_type
        self.pid_yaw = VendorPID(6.6,0.,.01,now)
        self.pid_dist = VendorPID(1.7,0.,.16,now)
        self.last_act,self.timestamp = 0,0.
        self.command = (0.,0.)
        self.phase = MODE_NAMES[mode]+"：待机"
        self.diagnostics = {}

    def update(self, scan, now):
        left,right = split_scan(scan,self.scan_angle,self.lidar_type)
        lv,rv = [v for v in left if _valid(v)],[v for v in right if _valid(v)]
        self.diagnostics = dict(mode=self.mode,left=min(lv) if lv else None,
            right=min(rv) if rv else None,left_count=len(lv),right_count=len(rv),
            target_distance=None,target_angle=None,hold_until=self.timestamp,last_act=self.last_act)
        if self.mode=="avoid":
            if len(lv)<=1 or len(rv)<=1:
                raise ValueError("避障检测角内有效雷达点不足")
            if now<self.timestamp:
                self.phase = "雷达避障：保持厂家转弯动作"
                return self.command
            lclose,rclose = min(lv)<=self.threshold,min(rv)<=self.threshold
            if not lclose and not rclose:
                self.last_act = 0
                self.command,self.phase = (self.speed,0.),"雷达避障：自动前进"
            else:
                yaw = self.speed*6.
                if lclose and not rclose:
                    yaw = yaw if self.last_act not in (0,1) else -yaw
                    self.last_act,angle = 1,math.pi/2
                elif lclose and rclose:
                    self.last_act,angle = 3,math.pi
                else:
                    yaw = -yaw if self.last_act not in (0,2) else yaw
                    self.last_act,angle = 2,math.pi/2
                self.timestamp = now+angle/(self.speed*6.)/2
                self.command = (self.speed/6.,yaw)
                self.phase = "雷达避障：前进向%s转弯" % ("左" if yaw>0 else "右")
            self.diagnostics.update(hold_until=self.timestamp,last_act=self.last_act)
            return self.command

        merged = right[::-1]+left
        valid = [v for v in merged if _valid(v)]
        if len(valid)<(1 if self.mode=="follow" else 2):
            raise ValueError(MODE_NAMES[self.mode]+"检测角内有效雷达点不足")
        distance = min(valid)
        angle = -math.radians(self.scan_angle)/2+scan["angle_increment"]*merged.index(distance)
        self.diagnostics.update(target_distance=distance,target_angle=math.degrees(angle))
        yaw = speed = 0.
        if distance<self.threshold and abs(math.degrees(angle))>5:
            g4 = self.lidar_type=="G4"
            self.pid_yaw.update(angle if g4 else -angle,now)
            limit = self.speed*(4. if g4 else 8.)
            yaw = max(-limit,min(limit,self.pid_yaw.output))*(-1 if g4 else 1)
        else:
            self.pid_yaw.clear()
        if self.mode=="follow":
            # Both the threshold/2 setpoint and fixed .20 m deadband are original.
            if distance<self.threshold and abs(.2-distance)>.02:
                self.pid_dist.update(self.threshold/2-distance,now)
                speed = max(-self.speed,min(self.speed,self.pid_dist.output))
            else:
                self.pid_dist.clear()
            if abs(speed)<.01:
                speed = 0.
        if abs(yaw)<.08:
            yaw = 0.
        self.command = speed,yaw
        self.phase = MODE_NAMES[self.mode]+("：目标在范围外，停车" if distance>=self.threshold else
                     "：跟踪范围内最近目标" if speed or yaw else "：目标居中或处于死区，停车")
        return self.command
