"""Shared, ROS-free path/footprint validation (also uploaded as path_rules.py)."""
import math


def checked_path(points, limit=6000):
    if not isinstance(points,(list,tuple)) or not 2<=len(points)<=limit:
        raise ValueError('路径需要 2–%d 个点'%limit)
    result=[]
    for point in points:
        if not isinstance(point,(list,tuple)) or len(point)!=3:
            raise ValueError('路径点需要 x/y/yaw')
        if any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) for v in point):
            raise ValueError('路径点需要有限数值')
        result.append(tuple(float(v) for v in point))
    if any(math.dist(a[:2],b[:2])>.15 for a,b in zip(result,result[1:])):
        raise ValueError('路径不连续，请按不超过 0.15m 间隔插值')
    return tuple(result)


def pose_clear(grid,pose,footprint=(.25,.15,.115),threshold=65):
    """Rectangle vs raster cell SAT, including claw; unknown/outside is blocked."""
    x,y,yaw=pose; ox,oy,oa=grid['origin'];r=grid['resolution']
    c,s=math.cos(oa),math.sin(oa);dx,dy=x-ox,y-oy
    x,y=(c*dx+s*dy)/r,(-s*dx+c*dy)/r;yaw-=oa
    front,rear,half=footprint; a,b=(front+rear)/(2*r),half/r
    c,s=math.cos(yaw),math.sin(yaw)
    x+=(front-rear)/(2*r)*c;y+=(front-rear)/(2*r)*s
    ex,ey=abs(c)*a+abs(s)*b,abs(s)*a+abs(c)*b
    lo_x,hi_x=math.floor(x-ex),math.floor(x+ex)
    lo_y,hi_y=math.floor(y-ey),math.floor(y+ey)
    if lo_x<0 or lo_y<0 or hi_x>=grid['width'] or hi_y>=grid['height']:
        return False
    for row in range(lo_y,hi_y+1):
        for col in range(lo_x,hi_x+1):
            value=grid['data'][row*grid['width']+col]
            if 0<=value<threshold:continue
            dx,dy=col+.5-x,row+.5-y
            if abs(c*dx+s*dy)<=a+.5*(abs(c)+abs(s)) and abs(-s*dx+c*dy)<=b+.5*(abs(c)+abs(s)):
                return False
    return True


def swept_clear(grid,points,footprint=(.25,.15,.115),threshold=65):
    for index,point in enumerate(points):
        if not pose_clear(grid,point,footprint,threshold):return False
        if index:
            previous=points[index-1];angle=math.atan2(math.sin(point[2]-previous[2]),math.cos(point[2]-previous[2]))
            count=max(1,math.ceil(math.dist(point[:2],previous[:2])/.02),math.ceil(abs(angle)/.1))
            for sample in range(1,count):
                t=sample/count
                pose=(previous[0]+(point[0]-previous[0])*t,previous[1]+(point[1]-previous[1])*t,previous[2]+angle*t)
                if not pose_clear(grid,pose,footprint,threshold):return False
    return True


def interpolate(vertices,step=.025):
    result=[]
    for a,b in zip(vertices,vertices[1:]):
        distance=math.dist(a[:2],b[:2]);yaw=math.atan2(b[1]-a[1],b[0]-a[0]) if distance else a[2]
        if not result:result.append((a[0],a[1],a[2]))
        if distance<1e-6:continue
        count=max(1,math.ceil(distance/step))
        result.extend((a[0]+(b[0]-a[0])*i/count,a[1]+(b[1]-a[1])*i/count,yaw) for i in range(1,count+1))
    return tuple(result)
