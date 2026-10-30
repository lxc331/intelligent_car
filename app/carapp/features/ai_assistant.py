"""API text/voice commands, coordinated through the existing motion controls."""
import os
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import ttk

from carapp.config import APP_DIR
from carapp.core.ai_actions import AIActionRunner, build_system_prompt, parse_plan
from carapp.core.ai_client import AIClient, AISettings, load_preferences, save_preferences
from carapp.core.ai_voice import MicrophoneRecorder, start_system_dictation
from carapp.features.base import Feature
from carapp.ui import theme
from carapp.ui.camerapanel import CameraPanel
from carapp.ui.focus import widget_has_focus

PREFERENCES = Path(APP_DIR) / "config" / "ai_settings.json"
PROVIDERS = {
    "DeepSeek": ("https://api.deepseek.com", "deepseek-flash"),
    "通义千问": ("https://dashscope.aliyuncs.com/compatible-mode/v1", "qwen-plus"),
    "自定义兼容接口": ("", ""),
}
STOP_WORDS = frozenset(("停止", "停车", "停下", "停止全部", "急停", "紧急停止", "取消", "stop", "stop all"))


class AIAssistantFeature(Feature):
    key = "ai"
    title = "AI 大模型接入"

    def __init__(self, ctx):
        super().__init__(ctx)
        self.closed = self.active = False
        self.busy = False
        self._events = queue.Queue()
        self._generation = 0
        self._runner_epoch = 0
        self._cancel = threading.Event()
        self._permit = threading.Event()
        self._identity = self._car_identity()
        self._was_connected = bool(ctx.state.get("ws_connected"))
        self._plan = None
        self._plan_identity = None
        self._recording = self._voice_opening = False
        self._recorder = self._asr_client = None
        self._asr_model = self._asr_mode = None
        self._history = []
        self._dirty = True
        self._status = "填写 API Key 后连接测试；可发送文字，或使用系统听写。"
        self.runner = AIActionRunner(ctx, allowed=self._allowed,
            notify=lambda text: self._events.put((self._runner_epoch, "runner", text, None)))

    def _car_identity(self):
        car = self.ctx.cfg.get("car", {})
        return (car.get("profile_id"), car.get("host"), car.get("transport"),
                id(getattr(self.ctx.app, "camera", None)), id(getattr(self.ctx.app,"bridge",None)),
                getattr(self.ctx.app,"_connection_generation",0))

    def _allowed(self):
        return not self.closed and self._permit.is_set() and not self.ctx.motion_in_use(exclude=self)

    @property
    def motion_active(self):
        return self.runner.motion_active

    @staticmethod
    def _button(parent, text, command, color=None):
        button = tk.Button(parent, text=text, command=command, bg=theme.PANEL_2,
            fg=color or theme.ACCENT, relief="flat", activebackground=theme.BORDER,
            font=theme.FONT_SMALL, padx=8, pady=5)
        button.pack(side="left", padx=3, pady=3)
        return button

    def _entry(self, parent, label, variable, column, width=26, secret=False):
        tk.Label(parent, text=label, bg=theme.PANEL, fg=theme.TEXT_DIM,
                 font=theme.FONT_SMALL).grid(row=0, column=column, sticky="w", padx=4)
        entry = tk.Entry(parent, textvariable=variable, width=width, show="●" if secret else "",
                         bg=theme.PANEL_2, fg=theme.TEXT, insertbackground=theme.TEXT,
                         relief="flat", font=theme.FONT_SMALL)
        entry.grid(row=1, column=column, sticky="ew", padx=4, pady=(2, 5))
        parent.columnconfigure(column, weight=1)
        return entry

    def build(self, parent):
        self.parent = parent
        ttk.Style(parent).map("AI.TCombobox",
            fieldbackground=[("readonly",theme.PANEL_2)], foreground=[("readonly",theme.TEXT)],
            selectbackground=[("readonly",theme.PANEL_2)], selectforeground=[("readonly",theme.TEXT)])
        try:
            prefs = load_preferences(PREFERENCES)
        except (ValueError, OSError):
            prefs = {}
        self.provider = tk.StringVar(value=prefs.get("provider", "DeepSeek"))
        default_base,default_model=PROVIDERS.get(self.provider.get(),("",""))
        self.base_url = tk.StringVar(value=prefs.get("base_url", default_base))
        self.model = tk.StringVar(value=prefs.get("model", default_model))
        self.api_key = tk.StringVar(value=os.environ.get("DEEPSEEK_API_KEY", "") if self.provider.get()=="DeepSeek" else "")
        self.asr_mode = tk.StringVar(value=prefs.get("asr_mode", "dashscope"))
        self.asr_url = tk.StringVar(value=prefs.get("asr_base_url", "https://dashscope.aliyuncs.com/compatible-mode/v1"))
        self.asr_model = tk.StringVar(value=prefs.get("asr_model", "qwen3-asr-flash"))
        self.asr_key = tk.StringVar()
        self.cloud_voice = tk.BooleanVar(value=False)
        panel = tk.Frame(parent, bg=theme.PANEL, padx=7, pady=5)
        panel.pack(fill="x", padx=5, pady=4)
        row = tk.Frame(panel, bg=theme.PANEL)
        row.pack(fill="x")
        self._entry(row, "模型名称", self.model, 1, 22)
        self._entry(row, "API Key（仅本次使用）", self.api_key, 2, 34, True)
        tk.Label(row, text="服务商", bg=theme.PANEL, fg=theme.TEXT_DIM, font=theme.FONT_SMALL).grid(row=0,column=0,sticky="w",padx=4)
        selector = ttk.Combobox(row, textvariable=self.provider, values=list(PROVIDERS), state="readonly", width=20,style="AI.TCombobox")
        selector.grid(row=1,column=0,sticky="ew",padx=4,pady=(2,5))
        selector.bind("<<ComboboxSelected>>", self._provider_changed)
        row = tk.Frame(panel, bg=theme.PANEL)
        row.pack(fill="x")
        tk.Label(row, text="Base URL", bg=theme.PANEL, fg=theme.TEXT_DIM, font=theme.FONT_SMALL).pack(side="left",padx=4)
        tk.Entry(row, textvariable=self.base_url, bg=theme.PANEL_2, fg=theme.TEXT,
                 insertbackground=theme.TEXT, relief="flat",font=theme.FONT_SMALL).pack(side="left",fill="x",expand=True,padx=4,ipady=4)
        self.btn_connect = self._button(row, "连接测试", self.test_connection)
        self._button(row, "保存地址与模型", self.save_settings)
        tk.Checkbutton(panel, text="录音转写 API（独立语音服务，可选）", variable=self.cloud_voice,
            command=self._toggle_voice_config, bg=theme.PANEL, fg=theme.TEXT_DIM, selectcolor=theme.BG,
            activebackground=theme.PANEL, font=theme.FONT_SMALL).pack(anchor="w")
        self.asr_panel = tk.Frame(panel,bg=theme.PANEL)
        self._entry(self.asr_panel,"语音 Base URL",self.asr_url,0,35)
        self._entry(self.asr_panel,"语音模型",self.asr_model,1,22)
        self._entry(self.asr_panel,"语音 API Key（独立填写）",self.asr_key,2,26,True)
        ttk.Combobox(self.asr_panel,textvariable=self.asr_mode,values=("dashscope","openai"),state="readonly",width=12,style="AI.TCombobox").grid(row=2,column=0,sticky="w",padx=4)
        tk.Label(self.asr_panel,text="dashscope：千问语音；openai：兼容音频转写接口。语音 Key 不会自动沿用 DeepSeek Key。",
                 bg=theme.PANEL,fg=theme.TEXT_DIM,font=theme.FONT_SMALL).grid(row=2,column=1,columnspan=2,sticky="w")
        split = ttk.Panedwindow(parent,orient="horizontal")
        split.pack(fill="both",expand=True,padx=5,pady=3)
        left = tk.Frame(split,bg=theme.PANEL,padx=7,pady=5)
        right = tk.Frame(split,bg=theme.BG)
        split.add(left,weight=3);split.add(right,weight=3)
        self.peer = tk.Label(left,text="",bg=theme.PANEL,fg=theme.ACCENT,font=theme.FONT_SMALL,anchor="w")
        self.peer.pack(fill="x")
        tk.Label(left,text="输入指令，例如：前进1秒，然后右转1秒，最后张开夹爪。",
                 bg=theme.PANEL,fg=theme.TEXT_DIM,font=theme.FONT_SMALL,anchor="w",wraplength=500).pack(fill="x",pady=4)
        self.command = tk.Text(left,height=4,bg=theme.BG,fg=theme.TEXT,insertbackground=theme.TEXT,
                               relief="flat",font=theme.FONT,wrap="word")
        self.command.pack(fill="x",pady=3)
        self.command.bind("<Control-Return>",lambda event:self.submit_command(True) or "break")
        row = tk.Frame(left,bg=theme.PANEL);row.pack(fill="x")
        self.btn_send = self._button(row,"发送并执行",lambda:self.submit_command(True),theme.OK)
        self.btn_plan = self._button(row,"仅生成动作",lambda:self.submit_command(False))
        self.btn_execute = self._button(row,"执行已生成动作",self.execute_plan)
        self.btn_stop = self._button(row,"■ 停止全部",self.ctx.stop_all_motion,theme.ERR)
        row = tk.Frame(left,bg=theme.PANEL);row.pack(fill="x")
        self._button(row,"系统语音输入（Win+H）",self.system_dictation)
        self.btn_record = self._button(row,"录音并执行",self.toggle_recording)
        self._button(row,"取消 AI 请求",self.stop_all,theme.WARN)
        tk.Label(left,text="系统听写会填入文本，点击发送后执行。录音按钮需配置独立语音 API；最长15秒。\n"
            "支持前后/平移/转向、云台和夹爪动作；距离与角度按速度时间估算。切页、失焦或断线会取消 AI 任务。",
            bg=theme.PANEL,fg=theme.TEXT_DIM,font=theme.FONT_SMALL,justify="left",wraplength=510).pack(fill="x",pady=4)
        self.status = tk.Label(left,text=self._status,bg=theme.PANEL,fg=theme.ACCENT,font=theme.FONT_SMALL,
                               justify="left",anchor="w",wraplength=510)
        self.status.pack(fill="x",pady=4)
        result_frame = tk.Frame(left,bg=theme.BG);result_frame.pack(fill="both",expand=True)
        self.result = tk.Text(result_frame,height=8,bg=theme.BG,fg=theme.TEXT,relief="flat",
                              font=theme.FONT_SMALL,wrap="word",state="disabled")
        scroll = ttk.Scrollbar(result_frame,command=self.result.yview)
        scroll.pack(side="right",fill="y");self.result.pack(side="left",fill="both",expand=True)
        self.result.config(yscrollcommand=scroll.set)
        self.cam = CameraPanel(right,self.ctx)
        self.cam.pack(fill="both",expand=True)
        for variable in (self.provider,self.base_url,self.model,self.api_key,self.asr_mode,self.asr_url,self.asr_model,self.asr_key):
            variable.trace_add("write",self._configuration_changed)

    def _configuration_changed(self,*_args):
        if not self.closed and (self.busy or self._recording or self.runner.motion_active or self._plan is not None):
            self.stop_all()
            self._set_status("API 配置已更改，旧请求与动作已取消，请重新发送。")

    def _safe(self, text):
        text = str(text)
        for name in ("api_key", "asr_key"):
            variable = getattr(self,name,None)
            key = variable.get() if variable is not None else ""
            if key:
                text = text.replace(key,"[已隐藏]")
        return text[:16000]

    def _append(self, label, text):
        self._history.append(label + "：" + self._safe(text))
        self._history = self._history[-50:]
        self._dirty = True

    def _set_status(self,text):
        self._status = self._safe(text)
        self._dirty = True

    def _provider_changed(self,_event=None):
        self.stop_all()
        base,model = PROVIDERS.get(self.provider.get(),("",""))
        self.base_url.set(base);self.model.set(model);self.api_key.set("")

    def _toggle_voice_config(self):
        if self.cloud_voice.get():
            self.asr_panel.pack(fill="x",pady=3)
        else:
            self.asr_panel.pack_forget()

    def save_settings(self):
        try:
            save_preferences(PREFERENCES,dict(provider=self.provider.get(),base_url=self.base_url.get(),model=self.model.get(),
                asr_base_url=self.asr_url.get(),asr_model=self.asr_model.get(),asr_mode=self.asr_mode.get()))
            self._set_status("已保存地址与模型；API Key 不写入配置或日志。")
        except (OSError,ValueError) as exc:
            self._set_status("保存失败："+str(exc))

    def _settings(self):
        return AISettings(self.base_url.get().strip(),self.model.get().strip(),self.api_key.get().strip())

    def _can_request(self):
        if self.closed or self.busy or self.runner.motion_active or self._recording:
            self._set_status("请先等待当前任务完成，或点击停止/取消。")
            return False
        return True

    def _new_request(self):
        self._cancel.set()
        self._generation += 1
        self._cancel = threading.Event()
        self.busy = True
        self._plan = self._plan_identity = None
        self._dirty = True
        return self._generation,self._cancel

    def _launch(self,kind,work,extra=None):
        generation = self._generation
        def worker():
            try:
                value,error = work(),None
            except Exception as exc:
                value,error = None,str(exc)
            self._events.put((generation,kind,(value,extra),error))
        threading.Thread(target=worker,name="ai-"+kind,daemon=True).start()

    def test_connection(self):
        if not self._can_request():return
        try:client=AIClient(self._settings())
        except ValueError as exc:self._set_status(str(exc));return
        self._new_request();self._set_status("正在测试 API 连接…")
        self._launch("test",client.check_connection)

    def submit_command(self,execute=True,text=None):
        text = (self.command.get("1.0","end").strip() if text is None else text.strip())
        if text.lower().strip("。！! ") in STOP_WORDS:
            self.ctx.stop_all_motion()
            return
        if not text:self._set_status("请输入文字指令或使用语音输入。 ");return
        if not self._can_request():return
        if execute and self.ctx.motion_in_use(exclude=self):
            self._set_status("其他功能正在执行动作，请先停止后再发送 AI 控制。 ");return
        if execute and (not self.ctx.state.get("ws_connected") or not self.ctx.state.get("cam_connected")):
            self._set_status("请先连接小车并等待实时画面；离线时可用“仅生成动作”。");return
        try:
            client=AIClient(self._settings());prompt=build_system_prompt(self.ctx.cfg)
        except (ValueError,TypeError) as exc:self._set_status(str(exc));return
        self._new_request();cancel=self._cancel
        self._append("你",text);self._set_status("正在理解指令并生成动作…")
        self._launch("plan",lambda:client.plan(text,prompt,cancel_event=cancel),execute)

    def execute_plan(self):
        if self.closed or self.busy or self._plan is None:
            self._set_status("请先生成有效动作。 ");return
        if self._plan_identity != self._car_identity():
            self._plan = None;self._set_status("车辆或摄像头已更换，请重新生成动作。");return
        if not self._allowed():
            self._set_status("本页未激活，或其他功能正在使用小车。");return
        self._runner_epoch = self._generation
        try:started=self.runner.start(self._plan)
        except (ValueError,TypeError) as exc:
            self._plan=None;self._set_status("动作未启动，请重新生成："+str(exc));return
        if started:
            self._set_status("AI 动作已启动；可随时点击停止全部。")
        else:self._set_status(self.runner.phase)

    def system_dictation(self):
        if self.closed:return
        self.command.focus_set()
        generation=self._generation
        def open_dictation():
            if self.closed or not self.active or generation!=self._generation or not widget_has_focus(self.command):return
            try:
                start_system_dictation()
                self._set_status("系统听写会输入到文本框；检查文字后点击发送。")
            except RuntimeError as exc:self._set_status(str(exc))
        self.ctx.root.after(150,open_dictation)

    def toggle_recording(self):
        if self._recording:
            if not self._voice_opening:self._end_recording()
            return
        if not self._can_request():return
        if not self.cloud_voice.get():
            self._set_status("请展开并填写录音转写 API，或使用系统语音输入。 ");return
        if self.ctx.motion_in_use(exclude=self):
            self._set_status("其他功能正在执行动作，请先停止后再录音控制。 ");return
        if not self.ctx.state.get("ws_connected") or not self.ctx.state.get("cam_connected"):
            self._set_status("请先连接小车并等待画面；离线时可使用系统听写和仅生成动作。");return
        try:
            self._settings()
            settings=AISettings(self.asr_url.get().strip(),self.asr_model.get().strip(),self.asr_key.get().strip())
            self._asr_client = AIClient(settings)
        except ValueError as exc:self._set_status("语音配置："+str(exc));return
        self._new_request();self._recording=self._voice_opening=True
        self._asr_model,self._asr_mode=self.asr_model.get().strip(),self.asr_mode.get()
        recorder = MicrophoneRecorder();self._recorder=recorder;cancel=self._cancel
        def start():
            recorder.start()
            if cancel.is_set():recorder.cancel()
        self._set_status("正在打开电脑默认麦克风…")
        self._launch("record",start)

    def _end_recording(self):
        recorder,client=self._recorder,self._asr_client
        self._recording=False;self._voice_opening=False;self.busy=True
        cancel,model,mode=self._cancel,self._asr_model,self._asr_mode
        self._set_status("正在结束录音并转写…")
        def transcribe():
            audio=recorder.stop()
            if cancel.is_set():return ""
            return client.transcribe(audio,model,mode=mode,cancel_event=cancel)
        self._launch("transcribe",transcribe)

    def on_background_tick(self, st):
        if self.closed:return
        identity=self._car_identity()
        if identity != self._identity:
            if self.busy or self._recording or self.runner.motion_active or self._plan is not None:self.stop_all()
            self._identity=identity
        connected=bool(st.get("ws_connected"))
        if self._was_connected and not connected:
            self.stop_all()
        self._was_connected=connected
        if self.runner.motion_active and not connected:self.stop_all()
        while True:
            try:generation,kind,data,error=self._events.get_nowait()
            except queue.Empty:break
            if generation != self._generation:continue
            if kind=="runner":self._set_status(data);continue
            self.busy=False
            if error:
                self._set_status("AI 操作失败："+error);self._append("错误",error)
                if kind in ("record","transcribe"):
                    self._recording=self._voice_opening=False
                    self._cancel_recording()
                continue
            value,extra=data
            if kind=="test":self._set_status("API 已连接，可发送指令。");self._append("连接测试",value)
            elif kind=="plan":
                try:self._plan=parse_plan(value,self.ctx.cfg)
                except (ValueError,TypeError) as exc:
                    self._set_status("动作未执行："+str(exc));self._append("动作校验",str(exc));continue
                self._plan_identity=self._car_identity()
                self._append("AI",self._plan.response+"\n"+self._plan.summary)
                self._set_status("动作已生成。" if self._plan.actions else "AI 回复完成，无可执行动作。")
                if extra and self._plan.actions:self.execute_plan()
            elif kind=="record":
                self._voice_opening=False;self._set_status("正在录音（最长15秒）；再次点击结束并执行。")
            elif kind=="transcribe":
                self._recorder=None
                if value:
                    self._append("语音识别",value)
                    self._voice_text=value
                    self.submit_command(True,text=value)
        if self._recording and not self._voice_opening and self._recorder and not self._recorder.recording:
            self._end_recording()

    def on_tick(self,st):
        if self.closed:return
        self.peer.config(text="当前车：%s · %s"%(self.ctx.cfg['car'].get('profile_id',''),self.ctx.cfg['car'].get('host','')))
        if hasattr(self,"_voice_text"):
            self.command.delete("1.0","end");self.command.insert("1.0",self._voice_text);del self._voice_text
        if self._dirty:
            self.status.config(text=self._status)
            self.result.config(state="normal");self.result.delete("1.0","end")
            self.result.insert("1.0","\n\n".join(self._history));self.result.see("end");self.result.config(state="disabled")
            busy=self.busy or self.runner.motion_active or self._recording
            for button in (self.btn_send,self.btn_plan,self.btn_connect):button.config(state="disabled" if busy else "normal")
            self.btn_execute.config(state="normal" if self._plan and not busy else "disabled")
            self.btn_record.config(text="麦克风打开中…" if self._voice_opening else "结束录音并执行" if self._recording else "录音并执行",
                                   state="disabled" if self._voice_opening or self.busy and not self._recording or self.runner.motion_active else "normal")
            self._dirty=False
        self.cam.on_tick(st)

    def _cancel_recording(self):
        recorder,self._recorder=self._recorder,None
        if recorder:threading.Thread(target=recorder.cancel,name="ai-audio-cancel",daemon=True).start()

    def stop_all(self):
        self._cancel.set();self._generation+=1
        self.busy=False;self._recording=self._voice_opening=False
        self._plan=self._plan_identity=None
        self._cancel_recording();self.runner.stop()
        self._set_status("AI 请求与动作已取消；新指令需重新发送。")

    def on_activate(self):
        self.active=True;self._permit.set()

    def on_deactivate(self):
        self.active=False;self._permit.clear();self.stop_all()

    def on_vehicle_change(self):
        self.stop_all();self._identity=self._car_identity()

    def on_close(self):
        self.closed=True;self._permit.clear();self.stop_all();self.runner.close()
        self.api_key.set("");self.asr_key.set("")
        self.cam.on_close()
