"""Road path validation and blockage inspection in the car owner process."""
import json
import math
import time
import uuid

def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class RoadRuntime:
    def _check_road(self,request,points):
        from path_rules import checked_path,swept_clear
        from nav2_msgs.srv import GetCostmap
        points=checked_path(points);epoch=int(request['epoch']);pose=self._pose()
        if not self._ready() or not pose or math.dist(pose[:2],points[0][:2])>.06:
            raise RuntimeError('路径起点与当前定位不一致')
        if abs(math.atan2(math.sin(pose[2]-points[0][2]),math.cos(pose[2]-points[0][2])))>.3:
            raise RuntimeError('路径起点朝向与当前定位不一致')
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
                if len(cost.data)!=meta.size_x*meta.size_y or not swept_clear(grid,points,self.path_footprint,253):
                    raise RuntimeError('当前道路被占用或整车轮廓无法通过，已保持停车')
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
                    if not pose_clear(grid,point,self.path_footprint,253):
                        self.blocked_at=point[:2];self.goal_state='blocked'
                        self.detail='道路受阻，已停车；检查备用道路';break
            except Exception:pass
        future.add_done_callback(inspect)

