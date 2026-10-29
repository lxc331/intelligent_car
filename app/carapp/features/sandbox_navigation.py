"""Sandbox-specific project and route coordination, separate from generic Nav2 UI."""
from pathlib import Path
import math
import queue
import threading
import tkinter as tk
from tkinter import filedialog
from carapp.config import APP_DIR
from carapp.core.map_data import MapModel
from carapp.core.sandbox_project import load_project
from carapp.core.sandbox_route import plan_route
from carapp.ui.sandbox_registration import SandboxRegistration
from carapp.ui import theme


class SandboxNavigation:
    def __init__(self,feature):
        self.feature=feature;self.project=None;self.route=None;self.dialog=None
        self.events=queue.Queue();self.epoch=0;self.busy=False
        self._launch=False;self._running=False;self._blocked=set();self._handled_block=None;self._retries=0
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
        self.text='已配准工程 | RMS %.1fcm | 25cm窄道%s'%(project.registration.rms*100,'开放' if project.metadata['narrow_enabled'] else '封闭')
        f._notice('工程已绑定：实测定位图 + 道路范围 + 九路口路网。实车从当前确认位置接入路网。')

    def reset(self):
        self.stop();self.project=None;self.route=None;self._blocked.clear()
        self.text='沙盘道路模式：请导入已配准工程。'

    def stop(self):
        self.epoch+=1;self._launch=self._running=False;self.busy=False

    def plan(self,launch=False,preview_start=None,retry=False):
        f=self.feature
        if not self.project:return f._notice('现有路网仅可推演。请先配准并导入沙盘工程，再沿路网行驶。',True)
        if self.busy or self._running:return False
        if launch and not retry:self._retries=0;self._blocked.clear();self._handled_block=None
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
        def work():
            try:self.events.put((epoch,'route',(plan_route(project,start,goal,blocked=blocked),start),None))
            except Exception as exc:self.events.put((epoch,'route',None,str(exc)))
        threading.Thread(target=work,name='sandbox-road-plan',daemon=True).start()
        f._notice('正在检查道路宽度、整车扫掠区域并计算最短路网路径…')
        return True

    def on_status(self,result):
        phase=result.get('goal_state')
        if phase=='blocked' and self._running:
            f=self.feature;key=result.get('route_id')
            if key==self._handled_block:return
            self._handled_block=key;self._running=False
            point=result.get('blocked_at')
            if point and self.route and self._retries<2 and f._ready() and not f._stop_requested:
                def distance(edge):
                    a=self.project.network.node(edge[0]);b=self.project.network.node(edge[1])
                    dx,dy=b.x-a.x,b.y-a.y;t=max(0.,min(1.,((point[0]-a.x)*dx+(point[1]-a.y)*dy)/(dx*dx+dy*dy)))
                    return math.hypot(point[0]-a.x-t*dx,point[1]-a.y-t*dy)
                edge=min(self.route.edges,key=distance)
                self._blocked.update((edge,(edge[1],edge[0])));self._retries+=1
                self.plan(True,retry=True)
            else:f._notice('道路受阻，已停车；请处理障碍后重新点击开始导航。',True)
        elif phase in ('succeeded','aborted','error','canceled','idle'):
            self._running=False

    def on_background_tick(self):
        while True:
            try:epoch,op,result,error=self.events.get_nowait()
            except queue.Empty:return
            if epoch!=self.epoch:continue
            self.busy=False;launch=self._launch;self._launch=False
            f=self.feature
            if error:f._notice(str(error),True);continue
            if op=='load':self.install(*result);continue
            route,start=result;self.route=route
            f.route=route.nodes;f.goal=f._route_goal_pose(route.nodes);f.map_panel.set_route(route.nodes);f.map_panel.set_plan(route.points)
            f._notice('已验证路网路径：%.2f m / %d 个点；窄路和不可通行路段已过滤。'%(route.length,len(route.points)))
            if launch:
                snapshot=f.model.snapshot();current=snapshot.get('pose')
                if f._stop_requested or not f._ready() or not current or snapshot.get('pose_stale') or math.dist(start[:2],current[:2])>.04 or abs(math.atan2(math.sin(start[2]-current[2]),math.cos(start[2]-current[2])))>.2:
                    f._notice('规划期间位置或控制许可变化，请重新开始导航。',True);continue
                if f.session.submit('follow_path',route.points,project_id=self.project.metadata['id']):self._running=True
                else:f._notice('路径未提交，请等待停车/服务操作完成后重试。',True)

    def on_tick(self):self.label.config(text=self.text)

    def close(self):
        self.stop()
        if self.dialog and self.dialog.winfo_exists():self.dialog.close()
