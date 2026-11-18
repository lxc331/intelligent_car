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


def project_polyline(path, x, y):
    """Nearest point on a path: returns (arc length from the start, lateral gap)."""
    if not path:
        return 0., 0.
    if len(path) < 2:
        return 0., math.hypot(x-path[0][0], y-path[0][1])
    best_s, best_gap, s = 0., float('inf'), 0.
    for a, b in zip(path, path[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            continue
        t = max(0., min(1., ((x-a[0])*dx + (y-a[1])*dy)/(length*length)))
        gap = math.hypot(x-(a[0]+t*dx), y-(a[1]+t*dy))
        if gap < best_gap:
            best_s, best_gap = s+t*length, gap
        s += length
    return best_s, best_gap


def corridor_obstacle(points, path, start_s=0., *, half_width=.06, lookahead=.55, behind=.05):
    """Nearest point inside the driven corridor, measured *along* the path.

    ``points`` are map-frame returns from the laser, ``path`` the route from the
    car onwards and ``start_s`` where the body currently sits on it. A narrow
    ``half_width`` keeps the corridor walls out of the verdict: the sandbox
    road leaves only 2–3 cm beside the body, so a body-wide band would stop the
    car in every straight. Returns ``(distance, (x, y))`` or ``(None, None)``.
    """
    if len(path) < 2 or not points:
        return None, None
    segments, s = [], 0.
    for a, b in zip(path, path[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        if length > 1e-9 and s+length >= start_s-behind and s <= start_s+lookahead:
            segments.append((a[0], a[1], dx, dy, length, s))
        s += length
    best = None
    for x, y in points:
        for ax, ay, dx, dy, length, segment_s in segments:
            raw = ((x-ax)*dx + (y-ay)*dy)/(length*length)
            if raw < 0. or raw > 1.:
                # Outside this segment: the neighbouring one owns the point, and
                # clamping here would credit a point 6 cm past the end with a
                # diagonal distance that looks closer than it is.
                continue
            if math.hypot(x-(ax+raw*dx), y-(ay+raw*dy)) > half_width:
                continue
            ahead = segment_s + raw*length - start_s
            if ahead < -behind or ahead > lookahead:
                continue
            ahead = max(0., ahead)
            if best is None or ahead < best[0]:
                best = (ahead, (x, y))
    return best if best else (None, None)


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


def rounded_path(vertices,radius=.10,step=.025):
    """Interpolate a waypoint polyline with quadratic arcs at the corners.

    Following a raw polyline makes the controller stop at every node: the path
    heading changes abruptly, so the curvature seen by the pure-pursuit lookahead
    blows up and the car crawls or oscillates. Cutting each corner with a small
    arc keeps the path inside the same corridor (the callers still validate the
    result with ``swept_clear``) while giving the controller a continuous heading.
    """
    points=[(float(a[0]),float(a[1])) for a in vertices]
    if len(points)<2:raise ValueError('路径至少需要两个点')
    if len(points)==2:
        return interpolate([(points[0][0],points[0][1],0.),(points[1][0],points[1][1],.0)],step)

    def line(a,b):
        length=math.dist(a,b)
        if length<1e-9:return []
        count=max(1,math.ceil(length/step))
        return [(a[0]+(b[0]-a[0])*i/count,a[1]+(b[1]-a[1])*i/count) for i in range(1,count+1)]

    sampled=[points[0]];cursor=points[0]
    for index in range(1,len(points)-1):
        previous=points[index-1];corner=points[index];following=points[index+1]
        before=math.dist(previous,corner);after=math.dist(corner,following)
        if before<1e-6 or after<1e-6:continue
        cut=min(radius,.45*before,.45*after)
        start=(corner[0]+(previous[0]-corner[0])/before*cut,corner[1]+(previous[1]-corner[1])/before*cut)
        end=(corner[0]+(following[0]-corner[0])/after*cut,corner[1]+(following[1]-corner[1])/after*cut)
        if math.dist(cursor,start)>1e-6:sampled.extend(line(cursor,start));cursor=start
        arc=max(1,math.ceil((math.dist(start,corner)+math.dist(corner,end))/step))
        for sample in range(1,arc+1):
            t=sample/arc;u=1-t
            point=(u*u*start[0]+2*u*t*corner[0]+t*t*end[0],u*u*start[1]+2*u*t*corner[1]+t*t*end[1])
            if math.dist(cursor,point)>1e-6:sampled.append(point);cursor=point
    if math.dist(cursor,points[-1])>1e-6:sampled.extend(line(cursor,points[-1]))
    output=[]
    for index,point in enumerate(sampled):
        following=sampled[index+1] if index+1<len(sampled) else None
        yaw=math.atan2(following[1]-point[1],following[0]-point[0]) if following else output[-1][2]
        output.append((point[0],point[1],yaw))
    return tuple(output)
