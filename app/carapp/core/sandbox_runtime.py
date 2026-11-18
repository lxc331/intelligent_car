"""Road path validation and blockage inspection in the car owner process."""
import json
import math
import time
import uuid

# Nav2 turns the car to the path heading before driving, so a car parked at an
# angle is normal. Only a genuinely backwards start is refused.
START_HEADING_TOLERANCE = 1.4

# Front-corridor clearance defaults, measured from base_footprint along the
# driven path. stop 0.25 m leaves ~5 cm in front of the 0.20 m body nose.
AVOID_DEFAULTS = dict(stop_distance=.25, slow_distance=.45, clear_distance=.32,
                      half_width=.06, lookahead=.55, hold=.25, release=.5)


class ForwardGuard:
    """Decide whether the corridor ahead is clear, with stop/clear hysteresis.

    The caller measures the corridor distance every tick (``None`` when nothing
    sits inside it), so this stays free of ROS and testable offline. It never
    commands motion: it reports a verdict and a speed scale only.
    """

    def __init__(self, stop_distance=.25, slow_distance=.45, clear_distance=.32,
                 half_width=.06, lookahead=.55, hold=.25, release=.5):
        values = dict(stop_distance=stop_distance, slow_distance=slow_distance,
                      clear_distance=clear_distance, half_width=half_width,
                      lookahead=lookahead, hold=hold, release=release)
        for name, value in values.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('避障参数须为有限数值：%s' % name)
        if not .10 <= stop_distance <= .40:
            raise ValueError('停车距离须在 0.10–0.40 米')
        if not stop_distance + .05 <= slow_distance <= 1.0:
            raise ValueError('减速距离须比停车距离大 0.05 米以上，且不超过 1.0 米')
        if not stop_distance <= clear_distance <= slow_distance:
            raise ValueError('恢复距离须在停车与减速距离之间')
        if not .03 <= half_width <= .15:
            raise ValueError('走廊半宽须在 0.03–0.15 米')
        if not .25 <= lookahead <= 1.2:
            raise ValueError('前视距离须在 0.25–1.2 米')
        if not 0. <= hold <= 2.:
            raise ValueError('阻挡确认时间须在 0–2 秒')
        if not 0. <= release <= 3.:
            raise ValueError('恢复确认时间须在 0–3 秒')
        self.stop_distance, self.slow_distance, self.clear_distance = stop_distance, slow_distance, clear_distance
        self.half_width, self.lookahead = half_width, lookahead
        self.hold, self.release = hold, release
        self.reset()

    def reset(self):
        self.blocked = False
        self.distance = None
        self.since = 0.
        self._hit_at = None
        self._clear_at = None

    def update(self, distance, now):
        """Feed one corridor measurement; returns the latched blocked state."""
        if distance is not None:
            distance = max(0., float(distance))
        self.distance = distance
        if distance is not None and distance <= self.stop_distance:
            self._clear_at = None
            if self._hit_at is None:
                self._hit_at = now
            if not self.blocked and now - self._hit_at >= self.hold:
                self.blocked, self.since = True, now
            return self.blocked
        self._hit_at = None
        if not self.blocked:
            return False
        if distance is None or distance >= self.clear_distance:
            if self._clear_at is None:
                self._clear_at = now
            elif now - self._clear_at >= self.release:
                self.blocked, self.since, self._clear_at = False, 0., None
        else:
            # Lingering inside the hysteresis band: keep the stop latched.
            self._clear_at = None
        return self.blocked

    @property
    def speed_scale(self):
        """Scale for the commanded speed: 0 while blocked, ramp on approach."""
        if self.blocked:
            return 0.
        if self.distance is None or self.distance >= self.slow_distance:
            return 1.
        span = self.slow_distance - self.stop_distance
        return max(0., min(1., (self.distance-self.stop_distance)/span)) if span > 0 else 1.

    def view(self):
        return dict(blocked=self.blocked, distance=self.distance, since=self.since)



def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def _first_block(grid,points,threshold=253):
    """Return (pose, cost) of the first sample the car's footprint cannot pass."""
    try:
        from path_rules import pose_clear
    except ImportError:
        from carapp.core.navigation_path import pose_clear
    footprint=grid['footprint']
    for index,point in enumerate(points):
        if not pose_clear(grid,point,footprint,threshold):
            return point,_cell_cost(grid,point)
        if index:
            previous=points[index-1]
            angle=math.atan2(math.sin(point[2]-previous[2]),math.cos(point[2]-previous[2]))
            count=max(1,math.ceil(math.dist(point[:2],previous[:2])/.02),math.ceil(abs(angle)/.1))
            for sample in range(1,count):
                t=sample/count
                pose=(previous[0]+(point[0]-previous[0])*t,previous[1]+(point[1]-previous[1])*t,
                      previous[2]+angle*t)
                if not pose_clear(grid,pose,footprint,threshold):
                    return pose,_cell_cost(grid,pose)
    return None,None


def _cell_cost(grid,pose):
    """Worst cost among the raster cells the footprint overlaps at this pose."""
    x,y,yaw=pose;ox,oy,oa=grid['origin'];r=grid['resolution']
    c,s=math.cos(oa),math.sin(oa);dx,dy=x-ox,y-oy
    x,y=(c*dx+s*dy)/r,(-s*dx+c*dy)/r;yaw-=oa
    footprint=grid['footprint'];front,rear,half=footprint;a,b=(front+rear)/(2*r),half/r
    c,s=math.cos(yaw),math.sin(yaw)
    x+=(front-rear)/(2*r)*c;y+=(front-rear)/(2*r)*s
    ex,ey=abs(c)*a+abs(s)*b,abs(s)*a+abs(c)*b
    lo_x,hi_x=math.floor(x-ex),math.floor(x+ex)
    lo_y,hi_y=math.floor(y-ey),math.floor(y+ey)
    if lo_x<0 or lo_y<0 or hi_x>=grid['width'] or hi_y>=grid['height']:
        return -1
    worst=0
    for row in range(lo_y,hi_y+1):
        for col in range(lo_x,hi_x+1):
            value=grid['data'][row*grid['width']+col]
            if value<=worst:
                continue
            dc,dr=col+.5-x,row+.5-y
            if abs(c*dc+s*dr)<=a+.5*(abs(c)+abs(s)) and abs(-s*dc+c*dr)<=b+.5*(abs(c)+abs(s)):
                worst=value
    return worst


def _describe_block(grid,points,footprint,threshold=253):
    """Human-readable reason with the offending cell, for the driver's screen."""
    if 'footprint' not in grid:
        grid=dict(grid,footprint=footprint)
    pose,cost=_first_block(grid,points,threshold)
    if pose is None:
        return '当前道路被占用或整车轮廓无法通过，已保持停车'
    return ('当前道路被占用或整车轮廓无法通过，已保持停车（第一处阻挡 x=%.2f y=%.2f 代价=%s；'
            '253=机身内切膨胀，254=实心障碍，255=未知区域）'%(pose[0],pose[1],cost))


class RoadRuntime:
    def _check_road(self,request,points):
        from path_rules import checked_path,swept_clear
        from nav2_msgs.srv import GetCostmap
        points=checked_path(points);epoch=int(request['epoch']);pose=self._pose()
        if not self._ready() or not pose or math.dist(pose[:2],points[0][:2])>.06:
            raise RuntimeError('路径起点与当前定位不一致')
        heading=math.atan2(math.sin(pose[2]-points[0][2]),math.cos(pose[2]-points[0][2]))
        if abs(heading)>START_HEADING_TOLERANCE:
            raise RuntimeError('路径起点朝向与当前定位相反（相差 %.0f°）：请把车头转向路线方向再点开始导航'
                               % math.degrees(abs(heading)))
        if not self.costmap_client.service_is_ready() or not self.follower.server_is_ready():
            raise RuntimeError('道路控制器或代价地图尚未就绪')
        key=request.get('request_id',uuid.uuid4().hex)
        self.pending[key]=dict(request=request,deadline=time.monotonic()+5.,handle=None)
        future=self.costmap_client.call_async(GetCostmap.Request())
        def checked(done):
            if key not in self.pending:return
            self.pending.pop(key,None)
            try:
                if not self._ready() or epoch<=self.revoked_epoch or epoch!=int(self.lease.get('epoch',-1)) or not self.lease.get('allow') or time.monotonic()>self.lease.get('expires',0):
                    raise RuntimeError('路径验证期间控制许可失效')
                cost=done.result().map;meta=cost.metadata;o=meta.origin
                if cost.header.frame_id!='map':raise ValueError('代价地图坐标系错误')
                grid=dict(width=meta.size_x,height=meta.size_y,resolution=meta.resolution,
                    origin=(o.position.x,o.position.y,yaw_of(o.orientation)),data=cost.data)
                if len(cost.data)!=meta.size_x*meta.size_y:
                    raise RuntimeError('代价地图尺寸与元数据不一致，已保持停车')
                threshold=max(253,min(254,int(request.get('pass_cost',253))))
                # 整条路径预检默认关闭：代价图是实时局部观测，容易把可走的
                # 窄道判成阻挡；真正的避障交给 Nav2 的代价图与跟随控制器。
                # 需要临时恢复时把 navigation.sandbox_verify_path 设为真。
                if request.get('verify_path') and not swept_clear(grid,points,self.path_footprint,threshold):
                    raise RuntimeError(_describe_block(grid,points,self.path_footprint,threshold))
                current=self._pose()
                if not current or math.dist(current[:2],points[0][:2])>.06:raise RuntimeError('验证期间位置变化，请重新规划')
                self._navigate(request,points[-1],points)
            except Exception as exc:self._revoke(str(exc));self._reply(request,error=str(exc))
        future.add_done_callback(checked)

    def _inspect_blockage(self,epoch,path):
        """After a stopped FollowPath failure, confirm an occupied remaining road."""
        from nav2_msgs.srv import GetCostmap
        from path_rules import pose_clear
        if not self.costmap_client.service_is_ready():return
        pose=self._pose()
        if not pose:return
        self.goal_state='checking_blockage'
        self._blockage_deadline=time.monotonic()+3.
        index=min(range(len(path)),key=lambda i:math.dist(pose[:2],path[i][:2]))
        future=self.costmap_client.call_async(GetCostmap.Request())
        def inspect(done):
            if self.goal_epoch!=epoch or self.goal_state!='checking_blockage':return
            self.goal_state='aborted'
            try:
                cost=done.result().map;m=cost.metadata;o=m.origin
                if cost.header.frame_id!='map' or len(cost.data)!=m.size_x*m.size_y:return
                grid=dict(width=m.size_x,height=m.size_y,resolution=m.resolution,
                    origin=(o.position.x,o.position.y,yaw_of(o.orientation)),data=cost.data)
                for point in path[index:]:
                    if not pose_clear(grid,point,self.path_footprint,254):
                        self.blocked_at=point[:2];self._pause_blocked();break
            except Exception:pass
        future.add_done_callback(inspect)

    def _pause_blocked(self,detail='前方道路受阻，已停车；净空恢复后自动继续'):
        """Turn Nav2's own "road blocked" abort into a wait the guard resumes.

        Without this the owner stayed in the dead 'blocked' state: the target,
        the remaining path and the motion grant were already dropped, so even
        after the obstacle was removed nothing could move again.
        """
        pose=self._pose()
        previous=getattr(self,'_blocked_pose',None)
        if pose and previous and math.dist(pose[:2],previous)>.10:
            self._blocked_count=0
        self._blocked_pose=pose[:2] if pose else previous
        self._blocked_count=getattr(self,'_blocked_count',0)+1
        if self._blocked_count>3:
            self._revoke('同一位置连续受阻，已停车；请人工处理后再点开始导航')
            return
        self.goal_state='paused'
        self._traffic_paused=True
        self._pause_source='obstacle'
        self.command=None
        self._zero()
        self.detail=detail

