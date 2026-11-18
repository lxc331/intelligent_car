"""Sandbox-specific project and route coordination, separate from generic Nav2 UI."""
from pathlib import Path
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog
from carapp.config import APP_DIR
from carapp.core.map_data import MapModel
from carapp.core.sandbox_project import load_project
from carapp.config import avoid_settings, sandbox_footprint
from carapp.core.navigation_avoidance import AvoidancePolicy, AvoidanceSettings
from carapp.core.sandbox_route import plan_route
from carapp.ui.sandbox_registration import SandboxRegistration
from carapp.ui import theme


class SandboxNavigation:
    def __init__(self,feature):
        self.feature=feature;self.project=None;self.route=None;self.dialog=None
        self.events=queue.Queue();self.epoch=0;self.busy=False
        self._launch=False;self._running=False;self._blocked=set();self._retries=0
        self._resend_at=-1e9;self._resend_count=0;self._resend_told=False
        cfg=getattr(getattr(feature,'ctx',None),'cfg',None)
        self.policy=AvoidancePolicy(AvoidanceSettings.from_config(avoid_settings(cfg)))
        self.text='沙盘道路模式：先配准并保存工程；旧路网仅作离线推演。'

    @property
    def motion_active(self):return self._launch or self._running

    def build(self,parent):
        bar=tk.Frame(parent,bg=theme.PANEL);bar.pack(fill='x')
        for text,command in [('导入沙盘工程',self.import_project),('配准 / 新建工程',self.open_registration)]:
            tk.Button(bar,text=text,command=command,bg=theme.PANEL_2,fg=theme.ACCENT,relief='flat',
                font=theme.FONT_SMALL,padx=7,pady=4).pack(side='left',padx=3,pady=2)
        self.label=tk.Label(bar,text=self.text,bg=theme.PANEL,fg=theme.TEXT_DIM,font=theme.FONT_SMALL,anchor='w')
        self.label.pack(side='left',fill='x',expand=True,padx=6)

    def can_change(self):
        f=self.feature;remote=f.session.snapshot() if f.session else {}
        return not f.closed and not f.motion_active and not remote.get('running') and not remote.get('busy')

    def open_registration(self):
        if not self.can_change():return self.feature._notice('请先结束导航服务再配准工程。',True)
        if self.dialog and self.dialog.winfo_exists():self.dialog.lift();return
        source=getattr(self.feature.bundle,'source_path',None)
        self.dialog=SandboxRegistration(self.feature.ctx.root,self.install,source,self.can_change,lambda:self.feature.active)

    def import_project(self,path=None):
        if not self.can_change():return self.feature._notice('请先结束导航服务再导入工程。',True)
        path=path or filedialog.askopenfilename(parent=self.feature.ctx.root,title='选择沙盘工程 sandbox.json',
            initialdir=str(Path(APP_DIR)/'savedMap'),filetypes=[('沙盘工程','sandbox.json')])
        if not path:return
        self.epoch+=1;epoch=self.epoch;self.busy=True
        def work():
            try:
                project=load_project(path);model=MapModel();model.accept_map(project.localization.grid_message,0.)
                self.events.put((epoch,'load',(project,model),None))
            except Exception as exc:self.events.put((epoch,'load',None,str(exc)))
        threading.Thread(target=work,name='sandbox-import',daemon=True).start()

    def install(self,project,model=None):
        if not self.can_change():return self.feature._notice('工程已保存；请结束服务后重新导入。',True)
        # Builds/imports arrive from background workers with the map predecoded.
        if model is None:
            return self.import_project(project.path)
        f=self.feature;f._install_map(project.localization,model)
        self.project=project;self._blocked.clear();self.route=None
        f._install_network(project.network,project.path.parent/'road_network.yaml')
        if getattr(f,'editor',None) is not None and f.editor.project.path!=project.path:
            f.editor=None;f._edit_selected=None
        f._update_edit_bar()
        self.text='已配准工程 | RMS %.1fcm | 25cm窄道%s'%(project.registration.rms*100,'开放' if project.metadata['narrow_enabled'] else '封闭')
        f._notice('工程已绑定：实测定位图 + 道路范围 + 九路口路网。实车从当前确认位置接入路网。')

    def reset(self):
        self.stop();self.project=None;self.route=None;self._blocked.clear();self.policy.reset()
        f=self.feature
        if getattr(f,'editor',None) is not None:f.editor=None;f._edit_selected=None
        self.text='沙盘道路模式：请导入已配准工程。'

    def stop(self):
        self.epoch+=1;self._launch=self._running=False;self.busy=False;self.policy.reset()
        self._resend_count=0;self._resend_told=False

    def plan(self,launch=False,preview_start=None,retry=False):
        f=self.feature
        if not self.project:return f._notice('现有路网仅可推演。请先配准并导入沙盘工程，再沿路网行驶。',True)
        if self.busy or self._running:return False
        if launch and not retry:
            self._retries=0;self._blocked.clear();self.policy.reset()
            self._resend_count=0;self._resend_told=False
        if f.goal_node is None:return f._notice('请先标记一个目标路口。',True)
        snapshot=f.model.snapshot();actual=snapshot.get('pose')
        if launch:
            if not f._ready() or actual is None or snapshot.get('pose_stale'):
                return f._notice('尚无新鲜的已确认位置，请先完成导航定位。',True)
            start=actual
        else:start=preview_start or actual
        if start is None:return f._notice('可先标记路网起点作离线检查；实车执行始终使用当前位置。',True)
        self.epoch+=1;epoch=self.epoch;self.busy=True;self._launch=bool(launch)
        if launch:f._stop_requested=False
        project=self.project;goal=f.goal_node.id;blocked=tuple(self._blocked)
        # 车体尺寸只读配置，提示留在界面线程：记录、状态栏都只有界面线程写，
        # 规划线程只负责算路径并把结果放进队列。
        footprint=sandbox_footprint(f.ctx.cfg)
        f._notice('车体判定 %.2f×%.2f m（前 %.2f / 后 %.2f / 半宽 %.2f）' % (
            footprint[0]+footprint[1], footprint[2]*2, footprint[0], footprint[1], footprint[2]))
        def work():
            try:self.events.put((epoch,'route',(plan_route(project,start,goal,blocked=blocked,footprint=footprint),start),None))
            except Exception as exc:self.events.put((epoch,'route',None,str(exc)))
        threading.Thread(target=work,name='sandbox-road-plan',daemon=True).start()
        f._notice('正在检查道路宽度、整车扫掠区域并计算最短路网路径…')
        return True

    def on_status(self,result):
        phase=result.get('goal_state');f=self.feature
        if phase in ('succeeded','aborted','error','canceled','idle'):
            self._running=False;self.policy.reset()
            return
        if getattr(f,'_stop_requested',False):
            # 人工已经停过一次；在用户重新点开始导航前不再自动介入，
            # 否则车端余下的 paused 状态会把它拖进“重新规划→再取消”的循环。
            return
        if phase in ('paused','blocked'):
            # 车端停住（等净空或已判受阻）：允许期间重新规划，但不算失败。
            self._running=False
        elif phase=='navigating':
            # 车端让开障碍后自己续跑了，跟它保持同步。
            self._running=True
            self._resend_count=0;self._resend_told=False
        if phase=='blocked' and not (result.get('avoid') or {}).get('blocked'):
            # 车端已把目标作废（blocked），但雷达/走廊显示前方畅通：说明障碍已
            # 让开，只是没人再给车一个目标。此时直接重新下发剩余路径。
            self._resend_after_block();return
        decision=self.policy.update(time.monotonic(),phase=phase,avoid=result.get('avoid'))
        if decision.action=='reroute':self._reroute(result)
        elif decision.action=='stop':
            f._notice(decision.reason,True);f.stop_all()
        elif decision.action in ('escalate','recovered') and decision.reason:
            f._notice(decision.reason)
        elif decision.reason and phase=='blocked':
            f._notice(decision.reason,True)

    def _resend_after_block(self):
        """Re-issue the route when the corridor cleared but the car is stopped.

        Nav2 may abort a FollowPath on a real obstacle and drop the goal; once
        that obstacle is gone nothing on the car retries, so the app has to
        hand it a fresh target. Bounded so a genuinely stuck car says so.
        """
        f=self.feature;now=time.monotonic()
        if not self.route or not f._ready() or getattr(f,'_stop_requested',False):return
        if now-self._resend_at<2.5:return
        if self._resend_count>=3:
            if not self._resend_told:
                self._resend_told=True
                f._notice('前方已经畅通，但车端连续 3 次无法起步；请检查是否贴墙，或重新标定位置后再点开始导航。',True)
            return
        self._resend_at=now
        if not self._resend_count:
            # 前方已畅通，之前为绕行封掉的路段是过期判断，先放开再重试。
            self._blocked.clear()
        self._resend_count+=1
        if not self.plan(True,retry=True):
            f._notice('暂时无法重新下发路径，稍后自动重试。',True);return
        f._notice('前方已畅通，正在重新下发路径（第 %d 次）。'%self._resend_count)

    def _nearest_edge(self,point):
        def distance(edge):
            a=self.project.network.node(edge[0]);b=self.project.network.node(edge[1])
            dx,dy=b.x-a.x,b.y-a.y;span=dx*dx+dy*dy
            t=max(0.,min(1.,((point[0]-a.x)*dx+(point[1]-a.y)*dy)/span)) if span else 0.
            return math.hypot(point[0]-a.x-t*dx,point[1]-a.y-t*dy)
        return min(self.route.edges,key=distance)

    def _reroute(self,result):
        """Try a new route from where the car stands; report the outcome back."""
        f=self.feature;now=time.monotonic()
        if not self.route or not f._ready() or f._stop_requested:
            self.policy.rerouted(False,now);return
        point=result.get('blocked_at')
        edge=self._nearest_edge(point) if point else self.route.edges[0]
        self._blocked.update((edge,(edge[1],edge[0])));self._retries+=1
        if not self.plan(True,retry=True):
            self.policy.rerouted(False,now)
            f._notice('当前正在处理其他操作，无法立即重新规划；先等待障碍移开。',True)
            return
        f._notice('已标记受阻路段 %d→%d，正在从当前位置重新规划。'%(edge[0],edge[1]))

    def _requires_turn(self,route,pose):
        """A route whose first leg points backwards would mean a U-turn."""
        if not getattr(route,'points',None) or pose is None:return False
        first=route.points[0]
        angle=math.atan2(math.sin(first[2]-pose[2]),math.cos(first[2]-pose[2]))
        return abs(angle)>self.policy.settings.max_turn

    def on_background_tick(self):
        while True:
            try:epoch,op,result,error=self.events.get_nowait()
            except queue.Empty:return
            if epoch!=self.epoch:continue
            self.busy=False;launch=self._launch;self._launch=False
            f=self.feature
            if error:
                if self.policy.state=='rerouting':self.policy.rerouted(False,time.monotonic())
                f._notice(str(error),True);continue
            if op=='load':self.install(*result);continue
            route,start=result
            if launch and self._requires_turn(route,start):
                if self.policy.state=='rerouting':self.policy.rerouted(False,time.monotonic())
                f._notice('新路线要求掉头（超过 %.0f°），当前走道做不到；已停车等待障碍移开。'
                          %math.degrees(self.policy.settings.max_turn),True)
                continue
            self.route=route
            f.route=route.nodes;f.goal=f._route_goal_pose(route.nodes);f.map_panel.set_route(route.nodes);f.map_panel.set_plan(route.points)
            f._notice('已验证路网路径：%.2f m / %d 个点；窄路和不可通行路段已过滤。'%(route.length,len(route.points)))
            if launch:
                snapshot=f.model.snapshot();current=snapshot.get('pose')
                if f._stop_requested or not f._ready() or not current or snapshot.get('pose_stale') or math.dist(start[:2],current[:2])>.04 or abs(math.atan2(math.sin(start[2]-current[2]),math.cos(start[2]-current[2])))>.2:
                    if self.policy.state=='rerouting':self.policy.rerouted(False,time.monotonic())
                    f._notice('规划期间位置或控制许可变化，请重新开始导航。',True);continue
                cost=max(253,min(254,int(f.ctx.cfg.get('navigation',{}).get('sandbox_pass_cost',254))))
                accepted=f.session.submit('follow_path',route.points,project_id=self.project.metadata['id'],pass_cost=cost,
                    verify_path=bool(f.ctx.cfg.get('navigation',{}).get('sandbox_verify_path',False)))
                if accepted:
                    self._running=True
                    if self.policy.state=='rerouting':self.policy.rerouted(True,time.monotonic())
                    # 下发成功后必须明确回执，否则界面停在“已验证路网路径”，
                    # 现场会以为没开始而反复点击或停止。
                    f._notice('路径已下发（%.2f m / %d 点），车辆开始沿路网行驶；点“■ 停止导航”可随时停车。'%(route.length,len(route.points)))
                else:
                    if self.policy.state=='rerouting':self.policy.rerouted(False,time.monotonic())
                    f._notice('路径未提交，请等待停车/服务操作完成后重试。',True)

    def on_tick(self):self.label.config(text=self.text)

    def close(self):
        self.stop()
        if self.dialog and self.dialog.winfo_exists():self.dialog.close()
