"""Auditable remote payloads for mapping; no code is installed in the ROS workspace.

The payloads use only Python's standard library until ROS is explicitly sourced.
All user values travel as JSON argv, never as executable shell fragments.
"""

SUPERVISOR = r'''
import json,os,select,signal,subprocess,sys,time
p=json.loads(sys.argv[1]); root=p['root']; token=p['token']
env=dict(os.environ,CARAPP_MAPPING_OWNER=token,ROS_LOG_DIR=root+'/ros_logs')
log=open(root+'/slam.log','w+b',buffering=0)
child=subprocess.Popen(p['argv'],stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                       env=env,start_new_session=True)
def birth(pid):
 return open('/proc/%d/stat'%pid).read().rsplit(')',1)[1].split()[19]
record={'token':token,'pid':child.pid,'birth':birth(child.pid),'root':root}
with open(root+'/owner.json','w') as f: json.dump(record,f)
def terminate():
 for sig,wait in ((signal.SIGINT,4),(signal.SIGTERM,2),(signal.SIGKILL,1)):
  try: os.killpg(child.pid,sig)
  except ProcessLookupError: break
  end=time.monotonic()+wait
  while time.monotonic()<end:
   child.poll()
   try: os.killpg(child.pid,0)
   except ProcessLookupError: return
   time.sleep(.1)
try:
 while child.poll() is None:
  if time.time()-os.stat(root+'/lease').st_mtime>90:
   log.write(b'\nMapping lease expired; stopping this owner only.\n'); terminate(); break
  if select.select([child.stdout],[],[],.5)[0]:
   block=os.read(child.stdout.fileno(),65536)
   if log.tell()+len(block)>2097152: log.seek(0); log.truncate()
   log.write(block)
finally:
 terminate()  # Also collect descendants if ros2 run exits before its node.
 log.close()
'''

REMOTE_SCRIPT = r'''
import base64,hashlib,json,os,pathlib,re,signal,stat,subprocess,sys,time
p=json.loads(sys.argv[1]); op=p['op']
def absolute(value):
 if not isinstance(value,str) or not value.startswith('/') or any(ord(c)<32 for c in value):
  raise ValueError('An absolute Linux path without control characters is required')
 if '..' in pathlib.PurePosixPath(value).parts: raise ValueError('Parent traversal is not allowed')
 return os.path.realpath(value)
def output(value): print(json.dumps(value,ensure_ascii=True))
def bounded_run(argv,timeout=15,max_output=12000,truncate=True):
 # A temporary file bounds resident memory even when a ROS command is noisy.
 import tempfile
 with tempfile.TemporaryFile() as f:
  child=subprocess.Popen(argv,stdin=subprocess.DEVNULL,stdout=f,stderr=f,start_new_session=True)
  try: code=child.wait(timeout=timeout)
  except subprocess.TimeoutExpired:
   try: os.killpg(child.pid,signal.SIGKILL)
   except ProcessLookupError: pass
   child.wait(); raise TimeoutError('Remote ROS command timed out')
  f.seek(0,2); size=f.tell()
  if size>max_output and not truncate: raise RuntimeError('Remote JSON response exceeds the allowed size')
  f.seek(max(0,size-max_output))
  text=f.read(max_output).decode('utf-8','replace')
  if code: raise RuntimeError('Remote command failed (%d): %s'%(code,text))
  return text
def source_command(argv):
 import shlex
 return ['bash','-c','source /opt/ros/humble/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && exec '+shlex.join(argv)]
def source_environment():
 # Reuse the active rosbridge domain/RMW rather than guessing the car's domain.
 candidates=[]
 for entry in pathlib.Path('/proc').iterdir():
  if not entry.name.isdigit(): continue
  try:
   cmd=(entry/'cmdline').read_bytes()
   if b'rosbridge_websocket' in cmd and b'python3\x00-c\x00' not in cmd:
    values={x.split(b'=',1)[0].decode():x.split(b'=',1)[1].decode() for x in (entry/'environ').read_bytes().split(b'\0') if b'=' in x}
    candidates.append(values)
  except (OSError,UnicodeError): pass
 if candidates:
  for key in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','CYCLONEDDS_URI','FASTRTPS_DEFAULT_PROFILES_FILE'):
   if key in candidates[0]: os.environ[key]=candidates[0][key]
   else: os.environ.pop(key,None)
 return bool(candidates)
def owned_record(root,token):
 try:
  record=json.loads(pathlib.Path(root,'owner.json').read_text())
  if record['token']!=token or record['root']!=root: return None
  pid=int(record['pid'])
  if pid<=1 or os.getpgid(pid)!=pid: return None
  born=pathlib.Path('/proc',str(pid),'stat').read_text().rsplit(')',1)[1].split()[19]
  environ=pathlib.Path('/proc',str(pid),'environ').read_bytes().split(b'\0')
  if born!=record['birth'] or ('CARAPP_MAPPING_OWNER='+token).encode() not in environ: return None
  return record
 except (OSError,ValueError,KeyError): return None
def touch(root):
 os.utime(root+'/lease',None)
def graph():
 import rclpy
 from ament_index_python.packages import get_package_share_directory
 rclpy.init(args=['--ros-args','--disable-external-lib-logs']); node=rclpy.create_node('carapp_mapping_probe_'+os.urandom(4).hex())
 try:
  end=time.monotonic()+2
  while time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.1)
  packages={}
  for name in ('slam','slam_toolbox','nav2_map_server'):
   try: packages[name]=get_package_share_directory(name)
   except LookupError: packages[name]=None
  topics=dict(node.get_topic_names_and_types()); nodes=node.get_node_names_and_namespaces()
  full=[ns.rstrip('/')+'/'+name for name,ns in nodes]
  services=dict(node.get_service_names_and_types())
  publishers={name:node.count_publishers(name) for name in ('/scan_raw','/odom','/tf','/tf_static')}
  config='/home/ubuntu/ros2_ws/src/slam/config/slam.yaml'
  if not os.path.isfile(config) and packages['slam']: config=packages['slam']+'/config/slam.yaml'
  return dict(packages=packages,topics=topics,nodes=full,services=services,publishers=publishers,
              config=config,config_exists=os.path.isfile(config),domain=os.environ.get('ROS_DOMAIN_ID','0'),
              environment_confirmed=bool(p.get('_environment_confirmed')),
              warnings=[] if p.get('_environment_confirmed') else ['Active rosbridge environment could not be read; mapping startup is disabled'])
 finally: node.destroy_node(); rclpy.shutdown()
def with_ros(action):
 p['_environment_confirmed']=source_environment()
 if not p.get('_sourced'):
  updated=dict(p,_sourced=True)
  # Re-execute only our payload after sourcing the installed ROS environment.
  raw=bounded_run(source_command(['python3','-c',p['_script'],json.dumps(updated)]),p.get('_timeout',25),1048576,truncate=False)
  print(raw); return
 action()
def ros_action():
 token=p.get('token',''); root='/tmp/carapp_slam_'+token
 if token and not re.fullmatch('[a-f0-9]{32}',token): raise ValueError('Invalid owner token')
 if op=='preflight': output(graph()); return
 if op=='status':
  record=owned_record(root,token)
  if record:
   touch(root); output(dict(running=True,owned=True,connected=True,detail='App-owned SLAM process is running',temporary_paths=[root],pid=record['pid'])); return
  g=graph(); running='/slam_toolbox' in g['nodes']
  output(dict(running=running,owned=False,connected=True,detail='Existing SLAM attached' if running else 'SLAM is stopped')); return
 if op=='start':
  import fcntl
  # Serialize concurrent App launches; never kill or replace an existing node.
  lock=open('/tmp/carapp_mapping_start.lock','a')
  fcntl.flock(lock,fcntl.LOCK_EX)
  g=graph()
  if not g.get('environment_confirmed'): raise RuntimeError('Cannot confirm the active rosbridge ROS environment; refusing to guess its domain')
  if '/slam_toolbox' in g['nodes']:
   record=owned_record(root,token)
   if record: touch(root)
   output(dict(running=True,owned=bool(record),status='attached',detail='Reusing existing SLAM; no hardware restarted',temporary_paths=[root] if record else [])); return
  if not g['packages']['slam_toolbox'] or not g['config_exists']: raise RuntimeError('SLAM Toolbox or vendor slam.yaml is missing')
  missing=[x for x in ('/scan_raw','/tf') if not g['publishers'][x]]
  if missing: raise RuntimeError('Start the existing car bringup first; missing publishers: '+', '.join(missing))
  if not g['publishers']['/odom']: raise RuntimeError('Wheel odometry /odom is not available; refusing to start mapping')
  pathlib.Path(root).mkdir(mode=0o700,exist_ok=True)
  pathlib.Path(root,'lease').touch()
  argv=['ros2','run','slam_toolbox','sync_slam_toolbox_node','--ros-args','--params-file',g['config'],
        '-r','__node:=slam_toolbox','-p','use_sim_time:=false','-p','mode:=mapping','-p','scan_topic:=/scan_raw',
        '-p','map_frame:=map','-p','odom_frame:=odom','-p','base_frame:=base_footprint','--disable-external-lib-logs']
  payload=json.dumps(dict(root=root,token=token,argv=argv))
  with open(root+'/supervisor.log','ab') as log:
   subprocess.Popen(['python3','-c',p['supervisor'],payload],stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
  end=time.monotonic()+5; record=None
  while time.monotonic()<end:
   record=owned_record(root,token)
   if record: break
   time.sleep(.1)
  if not record: raise RuntimeError('SLAM process failed to start; inspect '+root+'/slam.log')
  time.sleep(1)
  if not owned_record(root,token): raise RuntimeError('SLAM exited during startup; inspect '+root+'/slam.log')
  output(dict(running=True,owned=True,status='started',detail='Only SLAM Toolbox started; existing sensors reused',temporary_paths=[root,root+'/slam.log',root+'/supervisor.log',root+'/owner.json',root+'/lease'],pid=record['pid'])); return
 if op=='stop':
  record=owned_record(root,token)
  if not record:
   output(dict(running=False,owned=False,status='detached',detail='No matching owner process; no external service stopped')); return
  for sig,wait in ((signal.SIGINT,4),(signal.SIGTERM,2),(signal.SIGKILL,1)):
   record=owned_record(root,token)
   if not record: break
   os.killpg(record['pid'],sig)
   end=time.monotonic()+wait
   while time.monotonic()<end and owned_record(root,token): time.sleep(.1)
  running=bool(owned_record(root,token))
  output(dict(running=running,owned=running,status='stopped' if not running else 'stop_failed',temporary_paths=[root])); return
 if op=='save':
  remote=absolute(p['directory']); name=p['name']
  if not re.fullmatch(r'[\w\-][\w\- ]{0,63}',name,re.UNICODE) or name in ('.','..'): raise ValueError('Invalid map name')
  if not os.path.isdir(remote): raise ValueError('Choose an existing destination directory')
  # An exclusively created child directory prevents accidental replacement.
  destination=os.path.join(remote,p['folder']); os.mkdir(destination,0o755)
  prefix=destination+'/map'; warnings=[]; serialized=False
  record=owned_record(root,token)
  if record: touch(root)
  try:
   bounded_run(['ros2','run','nav2_map_server','map_saver_cli','-f',prefix,'--fmt','pgm','--ros-args',
                '-p','map_subscribe_transient_local:=true','-p','save_map_timeout:=10.0','--disable-external-lib-logs'],20)
   image=pathlib.Path(prefix+'.pgm'); yaml=pathlib.Path(prefix+'.yaml')
   if not image.is_file() or not yaml.is_file(): raise RuntimeError('Map saver did not create PGM and YAML')
   if image.stat().st_size<8 or yaml.stat().st_size<20: raise RuntimeError('Map saver produced empty files')
   text=yaml.read_text(); text,count=re.subn(r'(?m)^image\s*:.*$', 'image: map.pgm',text)
   if count!=1: raise RuntimeError('Map YAML must contain exactly one image reference')
   yaml.write_text(text)
   if p.get('serialize',True):
    try:
     import rclpy
     from slam_toolbox.srv import SerializePoseGraph
     rclpy.init(args=['--ros-args','--disable-external-lib-logs']); node=rclpy.create_node('carapp_map_serialize_'+os.urandom(4).hex())
     try:
      client=node.create_client(SerializePoseGraph,'/slam_toolbox/serialize_map')
      if not client.wait_for_service(timeout_sec=3): raise RuntimeError('serialize_map service is unavailable')
      request=SerializePoseGraph.Request(); request.filename=prefix
      future=client.call_async(request); rclpy.spin_until_future_complete(node,future,timeout_sec=12)
      if not future.done(): raise TimeoutError('serialize_map timed out')
      response=future.result()
      if int(response.result)!=0: raise RuntimeError('serialize_map result='+str(response.result))
      serialized=all(os.path.isfile(prefix+ext) and os.path.getsize(prefix+ext)>0 for ext in ('.posegraph','.data'))
      if not serialized: raise RuntimeError('Serialized posegraph/data files were not both created')
     finally: node.destroy_node(); rclpy.shutdown()
    except Exception as exc: warnings.append('2D map saved; continuing-map state unavailable: '+str(exc))
   files=[]
   for filename in ('map.yaml','map.pgm','map.posegraph','map.data'):
    path=pathlib.Path(destination,filename)
    if not path.is_file(): continue
    if path.is_symlink(): raise RuntimeError('Unexpected symbolic link in saved map')
    digest=hashlib.sha256()
    with path.open('rb') as f:
     for chunk in iter(lambda:f.read(262144),b''): digest.update(chunk)
    files.append(dict(name=filename,size=path.stat().st_size,sha256=digest.hexdigest()))
   output(dict(remote_dir=destination,files=files,serialized=serialized,warnings=warnings,scope='container'))
  except Exception as exc:
   output(dict(error=str(exc),remote_dir=destination,files=[],serialized=False,warnings=['Remote destination may contain incomplete map files']))
  return
 raise ValueError('Unknown ROS operation')
if op in ('preflight','start','stop','status','save'):
 with_ros(ros_action)
elif op=='heartbeat':
 token=p.get('token','')
 if not re.fullmatch('[a-f0-9]{32}',token): raise ValueError('Invalid owner token')
 root='/tmp/carapp_slam_'+token; record=owned_record(root,token)
 if record: touch(root)
 output(dict(owned=bool(record),running=bool(record)))
elif op=='directories':
 path=absolute(p['path']); entries=[]
 with os.scandir(path) as listing:
  for item in listing:
   if len(entries)>=2000: break
   if item.is_dir(follow_symlinks=True): entries.append(dict(name=item.name,path=os.path.realpath(item.path)))
 entries.sort(key=lambda e:e['name'].casefold())
 output(dict(path=path,entries=entries,scope=p['scope'],truncated=len(entries)>=2000))
elif op=='canonical_directory':
 path=absolute(p['path'])
 if not os.path.isdir(path): raise ValueError('Choose an existing destination directory')
 output(dict(path=path,scope=p['scope']))
elif op=='mkdir_unique':
 root=absolute(p['path'])
 if not os.path.isdir(root): raise ValueError('Choose an existing destination directory')
 path=os.path.join(root,p['folder']); os.mkdir(path,0o755); output(dict(path=path))
elif op=='manifest':
 root=absolute(p['path']); files=[]
 for name in ('map.yaml','map.pgm','map.posegraph','map.data'):
  file=pathlib.Path(root,name)
  if not file.exists(): continue
  if not stat.S_ISREG(file.lstat().st_mode): raise ValueError('Expected a regular map file')
  digest=hashlib.sha256()
  with file.open('rb') as f:
   for block in iter(lambda:f.read(262144),b''): digest.update(block)
  files.append(dict(name=name,size=file.stat().st_size,sha256=digest.hexdigest()))
 output(dict(files=files))
elif op=='cleanup_map':
 root=absolute(p['path'])
 if pathlib.PurePosixPath(root).parent!=pathlib.PurePosixPath('/tmp'): raise ValueError('Only a direct /tmp staging map may be removed')
 allowed={'map.yaml','map.pgm','map.posegraph','map.data'}
 expected={item['name']:item for item in p['files']}
 actual={item.name for item in pathlib.Path(root).iterdir()}
 if not actual<=allowed or actual!=set(expected): raise ValueError('Staging directory contents changed; nothing removed')
 for name,item in expected.items():
  path=pathlib.Path(root,name)
  if not stat.S_ISREG(path.lstat().st_mode): raise ValueError('Unexpected staging file type')
  digest=hashlib.sha256()
  with path.open('rb') as f:
   for chunk in iter(lambda:f.read(262144),b''): digest.update(chunk)
  if path.stat().st_size!=item['size'] or digest.hexdigest()!=item['sha256']: raise ValueError('Staging map changed; nothing removed')
 for name in expected: pathlib.Path(root,name).unlink()
 pathlib.Path(root).rmdir(); output(dict(removed=True,path=root))
elif op=='read':
 path=absolute(p['path']); limit=268435456
 if not stat.S_ISREG(os.lstat(path).st_mode) or os.path.getsize(path)>limit: raise ValueError('Not a regular bounded map file')
 with open(path,'rb') as f:
  for block in iter(lambda:f.read(262144),b''): sys.stdout.buffer.write(block)
else: raise ValueError('Unknown mapping operation')
'''
