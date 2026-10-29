"""Metric SE(2) registration and crop geometry, without GUI, ROS or numpy."""
from dataclasses import dataclass
import math


def points(values):
    if not isinstance(values, (list, tuple)) or not 3 <= len(values) <= 50:
        raise ValueError('请至少选 3 对对应点，最多 50 对')
    result = tuple(tuple(float(v) for v in p) for p in values)
    if any(len(p) != 2 or not all(math.isfinite(v) and abs(v) < 1e5 for v in p) for p in result):
        raise ValueError('对应点需要有限的 x/y 米制坐标')
    spread = max(math.dist(a, b) for a in result for b in result)
    area = max(abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))
               for a in result for b in result for c in result)
    if spread < .2 or area < max(1e-5, .005*spread*spread):
        raise ValueError('对应点过近或近似共线，请选择分散的建筑转角')
    return result


@dataclass(frozen=True)
class Registration:
    x: float
    y: float
    yaw: float
    rms: float
    maximum: float
    scale_ratio: float

    def forward(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x+c*x-s*y, self.y+s*x+c*y

    def inverse(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x, y = x-self.x, y-self.y
        return c*x+s*y, -s*x+c*y


def fit_registration(source, target, *, max_rms=.075, max_scale_error=.05):
    source, target = points(source), points(target)
    if len(source) != len(target):
        raise ValueError('两张图必须选择相同数量的对应点')
    def centroid(p):
        return tuple(sum(v[i] for v in p)/len(p) for i in (0, 1))
    a, b = centroid(source), centroid(target)
    centered = [((p[0]-a[0],p[1]-a[1]),(q[0]-b[0],q[1]-b[1])) for p,q in zip(source,target)]
    dot = sum(p[0]*q[0]+p[1]*q[1] for p,q in centered)
    cross = sum(p[0]*q[1]-p[1]*q[0] for p,q in centered)
    energy = sum(p[0]**2+p[1]**2 for p,q in centered)
    scale = math.sqrt(sum(q[0]**2+q[1]**2 for p,q in centered)/energy)
    yaw = math.atan2(cross,dot)
    c,s = math.cos(yaw),math.sin(yaw)
    x,y = b[0]-c*a[0]+s*a[1], b[1]-s*a[0]-c*a[1]
    transform = Registration(x,y,yaw,0.,0.,scale)
    errors = [math.dist(transform.forward(*p),q) for p,q in zip(source,target)]
    result = Registration(x,y,yaw,math.sqrt(sum(e*e for e in errors)/len(errors)),max(errors),scale)
    if abs(scale-1.) > max_scale_error:
        raise ValueError('两图尺度相差 %.1f%%；请核对分辨率和对应点，不自动缩放实测地图' % (abs(scale-1.)*100))
    if result.rms > max_rms or result.maximum > max_rms*2:
        raise ValueError('配准误差过大：均方根 %.1f cm / 最大 %.1f cm；请重新选点' % (result.rms*100,result.maximum*100))
    return result


def cropped_origin(origin, resolution, original_height, left, top, width, height):
    """Top-left image crop -> map origin; retain metric world coordinates."""
    if min(left,top) < 0 or min(width,height) <= 0 or top+height > original_height:
        raise ValueError('裁剪范围无效')
    dx,dy = left*resolution,(original_height-top-height)*resolution
    c,s = math.cos(origin[2]),math.sin(origin[2])
    return origin[0]+c*dx-s*dy,origin[1]+s*dx+c*dy,origin[2]
