"""Two-map correspondence picker; map loading/building never blocks Tk."""
import math
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import filedialog,ttk
from datetime import datetime
from carapp.config import APP_DIR
from carapp.core.map_data import MapModel
from carapp.core.navigation_map import load_navigation_map
from carapp.core.sandbox_geometry import fit_registration
from carapp.core.sandbox_project import build_project
from carapp.ui.mappanel import MapPanel
from carapp.ui import theme


class PointPicker(MapPanel):
    def __init__(self,parent,on_point):
        self.selected=[];self.on_point=on_point
        super().__init__(parent,MapModel())
        self.canvas.bind('<ButtonPress-1>',lambda e:None)
        self.canvas.bind('<B1-Motion>',lambda e:None)
        self.canvas.bind('<ButtonRelease-1>',self.pick)
        self.canvas.bind('<ButtonPress-3>',self._drag_start)
        self.canvas.bind('<B3-Motion>',self._drag_move)

    def set_model(self,model):
        self.model=model;self._grid=self._source=self._photo=None
        self._source_revision=self._render_key=self._last_state=None
        self.selected=[];self.reset_view()

    def pick(self,event):
        if not self._grid:return
        scale,left,top=self._geometry();grid=self._grid
        x=(event.x-left)/scale;y=grid['height']-(event.y-top)/scale
        if not (0<=x<grid['width'] and 0<=y<grid['height']):return
        x*=grid['resolution'];y*=grid['resolution'];ox,oy,yaw=grid['origin']
        c,s=math.cos(yaw),math.sin(yaw)
        self.on_point((ox+c*x-s*y,oy+s*x+c*y))

    def _draw(self,state):
        super()._draw(state);self.canvas.delete('correspondence')
        if not self._grid:return
        geometry=self._geometry()
        for index,point in enumerate(self.selected):
            x,y=self._point(*point,geometry)
            self.canvas.create_oval(x-5,y-5,x+5,y+5,outline='#ff6161',width=2,tags='correspondence')
            self.canvas.create_text(x+9,y-9,text=str(index+1),fill='#ef3030',tags='correspondence')


class SandboxRegistration(tk.Toplevel):
    def __init__(self,parent,on_saved,localization=None,can_change=lambda:True,visible=lambda:True):
        super().__init__(parent);self.title('沙盘工程：手绘道路与实测雷达图配准')
        self.geometry('1160x850');self.configure(bg=theme.BG);self.transient(parent)
        self.on_saved=on_saved;self.can_change=can_change;self.visible=visible
        self.events=queue.Queue();self.paths={};self.busy=False;self.closed=False
        self.epoch=0;self.load_tokens=[0,0];self.loading=set();self.points=[[],[]];self._timer=None
        bar=tk.Frame(self,bg=theme.PANEL);bar.pack(fill='x')
        for text,command in [('选择手绘道路图',lambda:self.load(0)),('选择实测定位图',lambda:self.load(1)),
            ('撤销一对',self.undo),('检查配准',self.check),('保存沙盘工程',self.save)]:
            tk.Button(bar,text=text,command=command,bg=theme.PANEL_2,fg=theme.TEXT,relief='flat').pack(side='left',padx=4,pady=5)
        self.status=tk.Label(self,text='依次左图点1、右图对应点1；至少3对分散且非共线的建筑转角。右键拖动、滚轮缩放。',
            bg=theme.PANEL,fg=theme.ACCENT,anchor='w',wraplength=1100,justify='left')
        self.status.pack(fill='x')
        split=ttk.Panedwindow(self,orient='horizontal');split.pack(fill='both',expand=True)
        self.pickers=[PointPicker(split,lambda point,i=i:self.add(i,point)) for i in (0,1)]
        for picker in self.pickers:split.add(picker,weight=1)
        self.listing=tk.Label(self,bg=theme.PANEL,fg=theme.TEXT,anchor='w',justify='left')
        self.listing.pack(fill='x')
        self.protocol('WM_DELETE_WINDOW',self.close)
        roads=Path(APP_DIR)/'savedMap/manual_map/rawMap.yaml'
        if roads.is_file():self.load(0,roads)
        if localization:self.load(1,localization)
        self.tick()

    def load(self,index,path=None):
        if self.busy:return
        path=path or filedialog.askopenfilename(parent=self,title='选择地图 YAML',initialdir=str(Path(APP_DIR)/'savedMap'),filetypes=[('ROS地图','*.yaml *.yml')])
        if not path:return
        self.epoch+=1;self.points=[[],[]]
        self.load_tokens[index]+=1;token=self.load_tokens[index];self.loading.add(index)
        def work():
            try:
                bundle=load_navigation_map(path);model=MapModel()
                if not model.accept_map(bundle.grid_message,0.):raise ValueError('地图解析失败')
                self.events.put(('load',(index,token),(str(path),model),None))
            except Exception as exc:self.events.put(('load',(index,token),None,str(exc)))
        threading.Thread(target=work,daemon=True,name='sandbox-registration-load').start()

    def add(self,index,point):
        if self.busy or self.loading or index not in self.paths:return
        expected=0 if len(self.points[0])==len(self.points[1]) else 1
        if index!=expected:
            self.status.config(text='请先在%s图点选第%d个对应点'%('左侧手绘' if expected==0 else '右侧雷达',len(self.points[expected])+1));return
        self.points[index].append(point)
        self.status.config(text='已记录%s图第%d点；两图点号必须对应同一处实体位置'%('手绘' if index==0 else '雷达',len(self.points[index])))

    def undo(self):
        if self.busy:return
        if len(self.points[0])>len(self.points[1]):self.points[0].pop()
        else:
            for values in self.points:
                if values:values.pop()

    def check(self):
        try:
            reg=fit_registration(*self.points)
            self.status.config(text='配准通过：旋转 %.1f°，平移 (%.3f, %.3f)m，RMS %.1fcm，最大 %.1fcm，尺度比 %.4f。请现场复核点号。'%
                (math.degrees(reg.yaw),reg.x,reg.y,reg.rms*100,reg.maximum*100,reg.scale_ratio))
            return True
        except Exception as exc:self.status.config(text=str(exc));return False

    def save(self):
        if self.busy or self.loading or not self.can_change() or not self.check() or set(self.paths)!={0,1}:return
        parent=filedialog.askdirectory(parent=self,title='选择工程保存位置（会新建子目录）',initialdir=str(Path(APP_DIR)/'savedMap'))
        if not parent:return
        destination=Path(parent)/('sandbox_project_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        self.busy=True;source=[list(p) for p in self.points[0]];target=[list(p) for p in self.points[1]]
        paths=dict(self.paths)
        self.status.config(text='正在生成坐标绑定、九路口路网与道路范围，原图保持不变…')
        def work():
            try:result=build_project(paths[1],paths[0],source,target,destination);self.events.put(('save',0,result,None))
            except Exception as exc:self.events.put(('save',0,None,str(exc)))
        threading.Thread(target=work,daemon=True,name='sandbox-project-build').start()

    def tick(self):
        if self.closed:return
        while True:
            try:op,index,result,error=self.events.get_nowait()
            except queue.Empty:break
            if op=='save':self.busy=False
            if op=='load':
                index,token=index
                if token!=self.load_tokens[index]:continue
                self.loading.discard(index);self.points=[[],[]]
            if error:self.status.config(text=error);continue
            if op=='load':
                self.paths[index]=result[0];self.pickers[index].set_model(result[1])
            elif op=='save':
                self.status.config(text='工程已保存：'+str(result.path))
                self.on_saved(result)
        if self.visible() and self.winfo_ismapped():
            for i,picker in enumerate(self.pickers):picker.selected=list(self.points[i]);picker.on_tick()
            self.listing.config(text=' | '.join('%d: 手绘(%.2f,%.2f) ↔ 雷达(%.2f,%.2f)'%(i+1,*a,*b)
                for i,(a,b) in enumerate(zip(*self.points))))
        self._timer=self.after(150,self.tick)

    def close(self):
        self.closed=True
        if self._timer:self.after_cancel(self._timer)
        for picker in self.pickers:picker.on_close()
        self.destroy()
