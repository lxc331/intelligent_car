"""Small audited SSH RPC payload; ROS lives in one owner runtime, not each call."""

REMOTE_SCRIPT = r'''
import base64,fcntl,hashlib,json,math,os,pathlib,re,signal,stat,subprocess,sys,time,uuid
p=json.loads(sys.argv[1]); op=p['op']; token=p.get('token','')
if token and not re.fullmatch('[a-f0-9]{32}',token): raise ValueError('Invalid owner token')
root=pathlib.Path('/tmp/carapp_nav_'+token)

def output(value): print(json.dumps(value,ensure_ascii=True))

def atomic(path,value):
 tmp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
 with tmp.open('x') as f: json.dump(value,f)
 os.replace(tmp,path)

def owned():
 try:
  value=json.loads((root/'owner.json').read_text()); pid=int(value['pid'])
  if value['token']!=token or value['root']!=str(root) or pid<=1 or os.getpgid(pid)!=pid: return None
  born=pathlib.Path('/proc',str(pid),'stat').read_text().rsplit(')',1)[1].split()[19]
  env=pathlib.Path('/proc',str(pid),'environ').read_bytes().split(b'\0')
  if born!=value['birth'] or ('CARAPP_NAV_OWNER='+token).encode() not in env: return None
  return value
 except (OSError,ValueError,KeyError): return None

def lease(allow=False):
 if not root.is_dir() or root.is_symlink(): raise RuntimeError('Owner session directory is absent')
 with (root/'intent.lock').open('a') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX)
  try: previous=json.loads((root/'lease.json').read_text())
  except (OSError,ValueError): previous={'epoch':-1}
  epoch=int(p.get('epoch',0))
  if epoch>=int(previous.get('epoch',-1)):
   now=time.monotonic(); expires=float(p.get('expires',now+2.))
   if not math.isfinite(expires): raise ValueError('Invalid lease deadline')
   traffic=bool(p.get('traffic_enabled',p.get('paused',False))); speed_limit=p.get('traffic_speed_limit') if traffic else None
   if speed_limit is not None:
    speed_limit=float(speed_limit)
    if not math.isfinite(speed_limit) or not 0.<speed_limit<=.15: raise ValueError('Invalid traffic speed limit')
   atomic(root/'lease.json',dict(epoch=epoch,allow=bool(allow),at=now,expires=min(now+2.,expires),
          paused=bool(p.get('paused')) and traffic,pause_reason=str(p.get('pause_reason',''))[:200],
          traffic_enabled=traffic,traffic_speed_limit=speed_limit))

def source_env():
 found=False
 for entry in pathlib.Path('/proc').iterdir():
  if not entry.name.isdigit(): continue
  try:
   cmd=(entry/'cmdline').read_bytes()
   if b'rosbridge_websocket' not in cmd or b'python3\x00-c\x00' in cmd: continue
   env={x.split(b'=',1)[0].decode():x.split(b'=',1)[1].decode() for x in (entry/'environ').read_bytes().split(b'\0') if b'=' in x}
   for key in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','CYCLONEDDS_URI','FASTRTPS_DEFAULT_PROFILES_FILE'):
    if key in env: os.environ[key]=env[key]
    else: os.environ.pop(key,None)
   found=True; break
  except (OSError,UnicodeError): pass
 return found

def sourced(operation):
 import shlex
 if not source_env(): raise RuntimeError('Cannot confirm the running rosbridge ROS environment')
 if p.get('_sourced'): return operation()
 args=['python3','-c',p['_script'],json.dumps(dict(p,_sourced=True))]
 command='source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && exec '+shlex.join(args)
 result=subprocess.run(['bash','-c',command],stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=35)
 if result.returncode: raise RuntimeError(result.stderr.decode('utf8','replace')[-12000:])
 if len(result.stdout)>1048576: raise RuntimeError('Oversized navigation response')
 sys.stdout.buffer.write(result.stdout)

def probe():
 import rclpy
 from ament_index_python.packages import get_package_share_directory
 rclpy.init(args=['--ros-args','--disable-external-lib-logs'])
 node=rclpy.create_node('carapp_navigation_check_'+uuid.uuid4().hex[:8])
 try:
  until=time.monotonic()+1.5
  while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.1)
  names=[ns.rstrip('/')+'/'+name for name,ns in node.get_node_names_and_namespaces()]
  required=['navigation','nav2_amcl','nav2_map_server','nav2_planner','nav2_navfn_planner',
            'nav2_controller','dwb_core','nav2_smoother','nav2_behaviors','nav2_bt_navigator',
            'nav2_velocity_smoother','nav2_lifecycle_manager','nav2_msgs','tf2_ros']
  packages={}
  for name in required:
   try: packages[name]=get_package_share_directory(name)
   except LookupError: packages[name]=None
  publishers={name:node.count_publishers(name) for name in ('/scan_raw','/odom','/tf','/map')}
  conflicts=[name for name in names if name.rsplit('/',1)[-1] in
             ('slam_toolbox','map_server','amcl','controller_server','planner_server','bt_navigator')]
  vendor='/home/ubuntu/ros2_ws/src/navigation/config'
  if not os.path.isfile(vendor+'/nav2_params.yaml') and packages['navigation']:
   vendor=packages['navigation']+'/config'
  missing=[name for name,path in packages.items() if not path]
  return dict(connected=True,running=False,ready=False,owned=False,packages=packages,
              missing=missing,conflicts=conflicts,publishers=publishers,vendor_dir=vendor,
              config_exists=all(os.path.isfile(vendor+'/'+name) for name in ('nav2_params.yaml','nav2_controller_dwb.yaml')),
              detail='导航检查完成' if not missing else '缺少导航软件包：'+', '.join(missing))
 finally: node.destroy_node(); rclpy.shutdown()

def startup():
 with open('/tmp/carapp_navigation_start.lock','a') as lock:
  fcntl.flock(lock,fcntl.LOCK_EX)
  info=probe()
  if info['missing'] or not info['config_exists']: raise RuntimeError(info['detail']+'; vendor config unavailable')
  if info['conflicts'] or info['publishers']['/map']:
   raise RuntimeError('请先结束正在运行的建图或其他导航服务：'+', '.join(info['conflicts']))
  if not info['publishers']['/scan_raw'] or not info['publishers']['/odom']:
   raise RuntimeError('雷达或里程计尚未运行')
  if owned(): raise RuntimeError('This owner already has a navigation process')
  for name,descriptor in p['files'].items():
   path=root/name
   if path.is_symlink() or not path.is_file(): raise RuntimeError('Missing session file')
   raw=path.read_bytes()
   if len(raw)!=descriptor['size'] or hashlib.sha256(raw).hexdigest()!=descriptor['sha256']:
    raise RuntimeError('Session upload verification failed: '+name)
  env=dict(os.environ,CARAPP_NAV_OWNER=token,ROS_LOG_DIR=str(root/'ros_logs'))
  log=(root/'supervisor.log').open('ab',buffering=0)
  child=subprocess.Popen(['python3',str(root/'runtime.py'),str(root),token,str(p['speed']),info['vendor_dir']],
                          stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                          env=env,start_new_session=True)
  log.close()
  born=pathlib.Path('/proc',str(child.pid),'stat').read_text().rsplit(')',1)[1].split()[19]
  atomic(root/'owner.json',dict(pid=child.pid,birth=born,root=str(root),token=token))
  end=time.monotonic()+25
  while time.monotonic()<end:
   if child.poll() is not None:
    tail=(root/'supervisor.log').read_text(errors='replace')[-6000:]
    raise RuntimeError('Navigation startup failed: '+tail)
   try:
    state=json.loads((root/'state.json').read_text())
    if state.get('startup_error'): raise RuntimeError(state['startup_error'])
    if state.get('running'): return state
   except (FileNotFoundError,json.JSONDecodeError): pass
   time.sleep(.1)
  raise TimeoutError('Navigation startup is still pending; cancel or reconnect explicitly')

def request():
 if not owned(): return dict(connected=True,running=False,owned=False,ready=False,goal_state='idle')
 request_id=uuid.uuid4().hex
 atomic(root/(request_id+'.request.json'),dict(op=op,request_id=request_id,
        epoch=int(p.get('epoch',0)),values=p.get('values',{})))
 end=time.monotonic()+float(p.get('request_timeout',20))
 answer=root/(request_id+'.reply.json')
 while time.monotonic()<end:
  if answer.exists():
   value=json.loads(answer.read_text()); answer.unlink()
   if value.get('error'): raise RuntimeError(value['error'])
   return value.get('result',{})
  if not owned(): raise RuntimeError('Navigation owner exited')
  time.sleep(.03)
 raise TimeoutError('Navigation request timed out; will not replay automatically')

if op=='preflight': sourced(lambda:output(probe()))
elif op=='prepare':
 os.mkdir(root,0o700); lease(False)
 output(dict(root=str(root),server_monotonic=time.monotonic(),temporary_paths=[str(root)]))
elif op=='upload':
 if p['name'] not in ('map.yaml','map.pgm','runtime.py','config_builder.py','navigation.launch.py'):
  raise ValueError('Unexpected session filename')
 path=root/p['name']; data=base64.b64decode(p['chunk'],validate=True)
 if len(data)>32768 or path.is_symlink(): raise ValueError('Invalid upload chunk')
 offset=int(p['offset'])
 with path.open('xb' if offset==0 else 'r+b') as f:
  f.seek(0,2)
  if f.tell()!=offset or offset+len(data)>20000000: raise ValueError('Invalid upload offset')
  f.write(data)
 output(dict(written=len(data)))
elif op in ('heartbeat','revoke'):
 lease(op=='heartbeat' and bool(p.get('allow')))
 output(dict(connected=True,renewed=True))
elif op=='start': sourced(lambda:output(startup()))
elif op=='status':
 if owned():
  try:
   state=root/'state.json'; value=json.loads(state.read_text())
   value['pose_age']=max(0.,time.time()-state.stat().st_mtime); value['pose_frame']='map'
   if value['pose_age']>2.:
    value.update(pose=None,localized=False,ready=False,detail='导航定位状态已过期，请检查车端服务')
  except (OSError,ValueError): value=dict(running=True,owned=True,ready=False,detail='导航服务启动中')
  value['connected']=True; output(value)
 else: output(dict(connected=True,running=False,owned=False,ready=False,goal_state='idle'))
elif op in ('initial_pose','plan','navigate','cancel'): output(request())
elif op=='stop':
 lease(False) if root.exists() else None
 record=owned()
 if record:
  for sig,wait in ((signal.SIGINT,4),(signal.SIGTERM,2),(signal.SIGKILL,1)):
   if not owned(): break
   try: os.killpg(int(record['pid']),sig)
   except ProcessLookupError: break
   until=time.monotonic()+wait
   while owned() and time.monotonic()<until: time.sleep(.1)
 output(dict(connected=True,running=bool(owned()),owned=bool(owned()),ready=False,
             goal_state='canceled',status='stopped' if not owned() else 'stopping',
             temporary_paths=[str(root)] if root.exists() else []))
else: raise ValueError('Unknown navigation operation')
'''
