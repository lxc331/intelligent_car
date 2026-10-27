"""Wi-Fi provisioning dialog and asynchronous flow; no robot movement commands."""
import copy
import datetime
import json
from pathlib import Path
import threading
import tkinter as tk
import weakref

from carapp.config import APP_DIR, save_config
from carapp.core.connection import search_prefixes
from carapp.core.connection_profiles import get_profile, TRANSPORT_LABELS
from carapp.core.credentials import local_password
from carapp.core.discovery import scan
from carapp.core.wifi_setup import configure_wifi, find_wifi, validate_wifi, WifiSetupError
from carapp.ui import theme


def _deliver(dialog_ref,method,args):
    dialog=dialog_ref()
    if dialog is not None and dialog.alive:getattr(dialog,method)(*args)


def _post(app_ref,dialog_ref,method,*args):
    app=app_ref()
    if app is not None:app._post(_deliver,dialog_ref,method,args)


class WifiSetupDialog(tk.Toplevel):
    def __init__(self,parent,app,ssid=''):
        super().__init__(parent)
        self.app=app
        self.identity=self.current_identity()
        self.profile=copy.deepcopy(get_profile(app.cfg))
        self.alive=True
        self.busy=False
        self.cancel=threading.Event()
        self.result=None
        self.directory=None
        self.latest=None
        self.record_camera=None
        self.owns_recording=False
        self.title('配置小车 Wi-Fi')
        self.configure(bg=theme.BG,padx=16,pady=12)
        self.resizable(False,False)
        self.transient(app.root)
        self.protocol('WM_DELETE_WINDOW',self.close)
        self.ssid=tk.StringVar(value=ssid or (self.profile or {}).get('sta_ssid',''))
        self.wifi_password=tk.StringVar()
        self.open_network=tk.BooleanVar(value=False)
        self.username=tk.StringVar(value=app.cfg.get('terminal',{}).get('username','pi'))
        self.login_password=tk.StringVar(value=local_password())
        self.auto_connect=tk.BooleanVar(value=True)
        self.status=tk.StringVar(value='填写目标网络后点击“下发并连接”。')
        self._build()
        app.ctx.add_frame_listener(self._on_frame)
        self.grab_set()

    def current_identity(self):
        car=self.app.cfg['car']
        return car.get('profile_id',''),car['host'],car.get('transport','wired')

    def _build(self):
        profile=self.profile or {}
        title='目标：%s · %s\n当前连接：%s  %s'%(profile.get('name','未选择车辆'),profile.get('id',''),
               TRANSPORT_LABELS.get(self.identity[2],self.identity[2]),self.identity[1])
        tk.Label(self,text=title,bg=theme.BG,fg=theme.ACCENT,font=theme.FONT,
                 anchor='w',justify='left').pack(fill='x',pady=(0,10))
        form=tk.Frame(self,bg=theme.BG);form.pack(fill='x')
        for row,label,var,secret in [(0,'新 Wi-Fi 名称',self.ssid,False),
                                     (1,'Wi-Fi 密码',self.wifi_password,True),
                                     (3,'小车登录用户',self.username,False),
                                     (4,'小车登录密码',self.login_password,True)]:
            tk.Label(form,text=label,bg=theme.BG,fg=theme.TEXT,font=theme.FONT_SMALL).grid(row=row,column=0,sticky='w',pady=5)
            entry=tk.Entry(form,textvariable=var,show='●' if secret else '',width=35,
                           bg=theme.PANEL_2,fg=theme.TEXT,insertbackground=theme.TEXT,font=theme.FONT)
            entry.grid(row=row,column=1,sticky='ew',padx=(12,0),pady=5)
            if row==1:self.password_entry=entry
        tk.Checkbutton(form,text='开放网络（无 Wi-Fi 密码）',variable=self.open_network,
                       command=self._toggle_open,bg=theme.BG,fg=theme.TEXT,selectcolor=theme.PANEL_2,
                       activebackground=theme.BG,font=theme.FONT_SMALL).grid(row=2,column=1,sticky='w',padx=(12,0))
        tk.Label(self,text='新网络会保存为小车的开机首选，下次启动优先连接。\n'
                 '先通过网线、原局域网或小车热点连接车辆。使用网线可保留切换时的管理连接。\n'
                 '电脑也需要接入目标局域网；支持开放网络和个人密码 Wi-Fi。\n'
                 '失败时小车临时回退 AP 并自动重试。Wi-Fi 密码不写入 App 配置。',
                 bg=theme.BG,fg=theme.TEXT_DIM,font=theme.FONT_SMALL,justify='left',
                 wraplength=550).pack(fill='x',pady=10)
        tk.Checkbutton(self,text='确认新地址可达后，将 App 切到 STA 连接',variable=self.auto_connect,
                       bg=theme.BG,fg=theme.TEXT,selectcolor=theme.PANEL_2,
                       activebackground=theme.BG,font=theme.FONT_SMALL).pack(anchor='w')
        tk.Label(self,textvariable=self.status,bg=theme.PANEL,fg=theme.ACCENT,justify='left',anchor='w',
                 wraplength=550,width=65,height=4,font=theme.FONT_SMALL,padx=8,pady=6).pack(fill='x',pady=10)
        buttons=tk.Frame(self,bg=theme.BG);buttons.pack(fill='x')
        self.submit=self._button(buttons,'下发并连接',self.apply)
        self.search=self._button(buttons,'查找新地址',self.search_new)
        self.use=self._button(buttons,'连接新地址',self.connect_new)
        self.stop=self._button(buttons,'关闭',self.close)
        self.search.configure(state='disabled');self.use.configure(state='disabled')

    def _button(self,parent,text,command):
        button=tk.Button(parent,text=text,command=command,bg=theme.PANEL_2,fg=theme.TEXT,
                         activebackground=theme.BORDER,relief='flat',font=theme.FONT_SMALL,padx=9,pady=5)
        button.pack(side='left',padx=(0,6));return button

    def _toggle_open(self):
        self.password_entry.configure(state='disabled' if self.open_network.get() else 'normal')

    def _progress(self,text):
        if self.alive:self.status.set(text)

    def _credentials(self):
        return dict(username=self.username.get().strip(),login_password=self.login_password.get(),
                    port=int(self.app.cfg.get('terminal',{}).get('port',22)),
                    known_hosts=str(Path(APP_DIR)/'config/ssh_known_hosts'))

    def _start(self,work,finish):
        self.busy=True;self.cancel.clear()
        for button in (self.submit,self.search,self.use):button.configure(state='disabled')
        self.stop.configure(text='停止等待')
        app_ref,dialog_ref=weakref.ref(self.app),weakref.ref(self)
        def worker():
            try:result=work()
            except Exception as exc:
                message=str(exc) if isinstance(exc,(ValueError,WifiSetupError)) else '操作未完成，请检查连接和本机记录'
                _post(app_ref,dialog_ref,'_failed',message)
            else:_post(app_ref,dialog_ref,finish,result)
        threading.Thread(target=worker,name='vehicle-wifi-setup',daemon=True).start()

    def apply(self):
        if self.busy:return
        if self.current_identity()!=self.identity or not self.profile:
            self.status.set('当前车辆或连接已变化，请关闭后重新打开配置窗口');return
        if self.app.ctx.motion_in_use():
            self.status.set('请先停止运动和动作序列，再配置 Wi-Fi');return
        ssid=self.ssid.get()
        password='' if self.open_network.get() else self.wifi_password.get()
        if not password and not self.open_network.get():
            self.status.set('请输入 Wi-Fi 密码；无密码网络请选择“开放网络”');return
        try:
            validate_wifi(ssid,password)
            credentials=self._credentials()
            if not credentials['username']:raise ValueError('请输入小车登录用户')
            import paramiko  # noqa: F401
        except (ValueError,ImportError) as exc:
            self.status.set(str(exc) if isinstance(exc,ValueError) else '请安装 App requirements.txt 中的 paramiko');return
        self.target_ssid=ssid
        self.result=None
        stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.directory=Path(APP_DIR).parent/'.work/wifi_setup'/(stamp+'_'+self.profile['id'])
        try:self._start_evidence()
        except OSError:
            self.status.set('无法创建本机备份目录，尚未下发配置');return
        guard=self.app._connection_guard()
        profile,host,transport=copy.deepcopy(self.profile),self.identity[1],self.identity[2]
        directory,cancel=self.directory,self.cancel
        app_ref,dialog_ref=weakref.ref(self.app),weakref.ref(self)
        self._start(lambda:configure_wifi(profile,host,transport,ssid,password,
                    **credentials,directory=directory,guard=guard,cancel=cancel,
                    progress=lambda text:_post(app_ref,dialog_ref,'_progress',text)),'_finished')

    def _on_frame(self,ts,payload,mode):
        self.latest=(ts,payload,mode)

    def _start_evidence(self):
        self.directory.mkdir(parents=True,exist_ok=True)
        if self.latest:
            (self.directory/('before.'+('jpg' if self.latest[2]=='jpeg' else 'png'))).write_bytes(self.latest[1])
        camera=self.app.camera
        if camera and camera.connected:
            self.owns_recording=not camera.recording
            self.record_camera=camera
            path=camera.start_record(str(self.directory))
            self.app.log('Wi-Fi 配置影像保存在电脑：'+str(path))

    def _finish_evidence(self):
        if self.record_camera and self.owns_recording:self.record_camera.stop_record()
        self.record_camera=None;self.owns_recording=False
        if self.directory and self.latest:
            path=self.directory/('latest.'+('jpg' if self.latest[2]=='jpeg' else 'png'))
            try:path.write_bytes(self.latest[1])
            except OSError:pass

    def _idle(self):
        self.busy=False
        self.submit.configure(state='normal')
        self.search.configure(state='normal' if self.result and self.result.get('submitted') else 'disabled')
        self.use.configure(state='normal' if self.result and self.result.get('ip') else 'disabled')
        self.stop.configure(text='关闭')

    def _failed(self,message):
        if not self.alive:return
        self._finish_evidence();self._idle()
        self.status.set(message)
        self.app.log('小车 Wi-Fi 配置：'+message)

    def _finished(self,result,force_connect=False):
        if not self.alive:return
        self._finish_evidence();self.result=result;self._idle()
        if self.current_identity()!=self.identity:
            self.status.set('原车辆的操作已结束，当前连接已变化，未自动应用结果');return
        if self.directory:
            try:(self.directory/'latest_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
            except OSError:pass
        if result.get('state')=='connected':
            profile=get_profile(self.app.cfg,self.profile['id'])
            previous=copy.deepcopy(profile)
            profile.update(sta_host=result['ip'],sta_ssid=result['ssid'])
            try:save_config(self.app.cfg)
            except OSError:
                profile.clear();profile.update(previous)
                self.status.set('小车已连接，但 App 地址保存失败：'+result['ip']);return
            boot_confirmed=bool(result.get('preference_saved') and result.get('boot_autostart')
                                and result.get('service_active'))
            saved_text='已保存为开机首选，下次启动优先连接。' if boot_confirmed else '开机首选尚未确认，请重新下发配置。'
            self.status.set('小车已连接 %s，新 IP：%s。%s\n%s'%(result['ssid'],result['ip'],saved_text,
                '电脑可访问。' if result.get('reachable') else '电脑尚不能访问，请确认局域网互通后再点击“连接新地址”。'))
            if boot_confirmed and result.get('reachable') and (self.auto_connect.get() or force_connect):self._use_verified(result)
        else:
            message=result.get('message','尚未下发配置' if not result.get('submitted') else '等待确认新网络')
            if result.get('preference_saved') and result.get('boot_autostart') and result.get('service_active'):
                message+='\n该网络已设为开机首选，下次启动仍会优先连接。'
            self.status.set(message)
        if self.directory:
            self.app.log('小车 Wi-Fi 配置记录与原配置备份：'+str(self.directory))

    def _use_verified(self,result):
        if self.app.change_host(result['ip'],self.profile['id'],'sta',sta_ssid=result['ssid']):
            self.identity=self.current_identity()
            self.status.set('已切换 App 到 %s · %s\n已保存为开机首选，下次小车启动优先连接。'%(result['ssid'],result['ip']))

    def _find(self,only_ip=False):
        if self.busy or not self.result or not self.result.get('submitted'):return
        if self.current_identity()!=self.identity:
            self.status.set('当前连接已变化，请重新打开窗口');return
        if self.app.ctx.motion_in_use():
            self.status.set('请先停止运动和动作序列');return
        try:credentials=self._credentials()
        except ValueError:self.status.set('SSH 端口设置无效');return
        car=copy.deepcopy(self.app.cfg['car']);car['transport']='sta'
        profiles=copy.deepcopy(self.app.cfg['cars']);ssid=self.target_ssid
        ip=self.result.get('ip')
        self._connect_after_find=only_ip
        profile,cancel=copy.deepcopy(self.profile),self.cancel
        app_ref,dialog_ref=weakref.ref(self.app),weakref.ref(self)
        def work():
            _post(app_ref,dialog_ref,'_progress','正在查找并核对车辆身份，请确保电脑接入目标局域网')
            candidates=[ip] if only_ip and ip else scan(search_prefixes(car,profiles),cancel=cancel.is_set)
            return find_wifi(profile,ssid,candidates,**credentials,cancel=cancel)
        self._start(work,'_found')

    def _found(self,result):
        self._finished(result,force_connect=self._connect_after_find)

    def search_new(self):self._find()
    def connect_new(self):self._find(only_ip=True)

    def close(self):
        if self.busy:
            self.cancel.set()
            self.status.set('正在停止等待；已经下发的配置仍会由小车继续执行')
            return
        self.destroy()

    def destroy(self):
        if not self.alive:return
        self.alive=False;self.cancel.set();self._finish_evidence()
        if self._on_frame in self.app.frame_listeners:self.app.frame_listeners.remove(self._on_frame)
        # A worker may still retain the dialog until SSH returns. Dispose Tcl
        # variables here, on the UI thread, rather than in that worker's GC.
        for name in ('ssid','wifi_password','open_network','username','login_password','auto_connect','status'):
            variable=getattr(self,name)
            try:variable.set(False if name in ('open_network','auto_connect') else '')
            except tk.TclError:pass
            setattr(self,name,None)
        super().destroy()
