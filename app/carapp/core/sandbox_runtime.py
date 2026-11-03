"""Road path validation and blockage inspection in the car owner process."""
import json
import math
import time
import uuid

# Nav2 turns the car to the path heading before driving, so a car parked at an
# angle is normal. Only a genuinely backwards start is refused.
START_HEADING_TOLERANCE = 1.4


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
                        self.blocked_at=point[:2];self.goal_state='blocked'
                        self.detail='道路受阻，已停车；检查备用道路';break
            except Exception:pass
        future.add_done_callback(inspect)

