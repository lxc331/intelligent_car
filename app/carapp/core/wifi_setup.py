"""Configure the selected vehicle over SSH; credentials never enter App settings."""
import json
import base64
import hashlib
import os
from pathlib import Path
import re
import shlex
import socket
import time


CONFIG_PATH = '/home/pi/hiwonder-toolbox/wifi_conf.py'
TOOLBOX = '/home/pi/hiwonder-toolbox'
ADAPTER_PATH = TOOLBOX+'/wifi_recovery_adapter.py'
BOOT_LINK_PATH = '/etc/systemd/system/multi-user.target.wants/wifi.service'
PRE_OPEN_ADAPTER_SHA256 = '819ceec0d3977de66731671bddb4c3d47cf23b98d404855cbe30ddc2b7e504a8'

IDENTITY_SCRIPT = r'''
import json,pathlib
p=pathlib.Path('/proc/device-tree/serial-number')
serial=p.read_text().strip('\x00\n ') if p.exists() else ''
print(json.dumps({'profile_id':'HW-'+serial[-9:-1].upper() if len(serial)>=9 else ''}))
'''

BACKUP_SCRIPT = r'''
import json,pathlib,hashlib,base64,sys,subprocess
sys.dont_write_bytecode=True
root=pathlib.Path('/home/pi/hiwonder-toolbox')
if not all((root/name).is_file() for name in ['wifi_recovery.py','wifi_recovery_adapter.py','wifi_config_store.py','wifi_conf.py']):
 print(json.dumps({'ok':False,'error':'recovery_missing'}));raise SystemExit
paths=[root/'wifi_conf.py',root/'wifi_recovery_adapter.py']
unit=pathlib.Path('/etc/systemd/system/wifi.service')
if unit.is_file():paths.append(unit)
paths+=list(pathlib.Path('/etc/NetworkManager/system-connections').glob('hiwonder-wlan0-*.nmconnection'))
files={}
for p in paths:
 raw=p.read_bytes();s=p.stat()
 files[str(p)]={'data':base64.b64encode(raw).decode(),'sha256':hashlib.sha256(raw).hexdigest(),
                'mode':s.st_mode&0o777,'uid':s.st_uid,'gid':s.st_gid}
def service_state(action):
 return subprocess.run(['systemctl',action,'wifi.service'],capture_output=True,text=True,timeout=6).stdout.strip()
print(json.dumps({'ok':True,'files':files,'service':{
 'enabled':service_state('is-enabled'),'active':service_state('is-active')}}))
'''

WRITE_SCRIPT = r'''
import sys,json,pathlib,hashlib,base64,subprocess
sys.dont_write_bytecode=True
sys.path.insert(0,'/home/pi/hiwonder-toolbox')
from wifi_config_store import WifiOperationLock,update_config_file,load_config_values
p=json.load(sys.stdin)
serial=pathlib.Path('/proc/device-tree/serial-number').read_text().strip('\x00\n ')
path=pathlib.Path('/home/pi/hiwonder-toolbox/wifi_conf.py')
error=''
modified=[]
preference_saved=False
adapter=pathlib.Path('/home/pi/hiwonder-toolbox/wifi_recovery_adapter.py')
with WifiOperationLock(timeout=30):
 if 'HW-'+serial[-9:-1].upper()!=p['profile_id']:error='identity'
 elif hashlib.sha256(path.read_bytes()).hexdigest()!=p['expected_sha256']:error='changed'
 elif p.get('adapter') and hashlib.sha256(adapter.read_bytes()).hexdigest()!=p['adapter_expected_sha256']:error='changed'
 else:
  if p.get('adapter'):
   from wifi_config_store import atomic_write_text
   source=base64.b64decode(p['adapter']).decode('utf-8')
   compile(source,str(adapter),'exec')
   atomic_write_text(adapter,source);modified.append(str(adapter))
  update_config_file(path,{'WIFI_MODE':2,'WIFI_STA_SSID':p['ssid'],'WIFI_STA_PASSWORD':p['password']})
  modified.append(str(path))
  try:
   saved=load_config_values(path)
   preference_saved=(saved.get('WIFI_MODE')==2 and saved.get('WIFI_STA_SSID')==p['ssid']
                     and saved.get('WIFI_STA_PASSWORD')==p['password'])
  except Exception:pass
restart_ok=True
boot_autostart=False
service_active=False
def state(action):
 return subprocess.run(['systemctl',action,'wifi.service'],capture_output=True,text=True,timeout=6).stdout.strip()
if not error and preference_saved:
 try:
  enabled_before=state('is-enabled')
  if enabled_before!='enabled':
   subprocess.run(['systemctl','enable','wifi.service'],check=True,timeout=15,capture_output=True)
  boot_autostart=state('is-enabled')=='enabled'
  if enabled_before!='enabled' and boot_autostart:
   modified.append('/etc/systemd/system/multi-user.target.wants/wifi.service')
 except Exception:pass
 try:
  # start is idempotent for an already running service; do not disrupt a
  # working STA connection just to ensure that an inactive service is started.
  subprocess.run(['systemctl','restart' if p.get('restart_service') else 'start','wifi.service'],
                 check=True,timeout=20,capture_output=True)
 except Exception:restart_ok=False
 try:service_active=state('is-active')=='active'
 except Exception:pass
print(json.dumps({'ok':not error,'error':error,'modified_files':modified,'restart_ok':restart_ok,
 'preference_saved':preference_saved,'boot_autostart':boot_autostart,'service_active':service_active}),flush=True)
'''

STATUS_SCRIPT = r'''
import sys,json,subprocess,ipaddress
sys.dont_write_bytecode=True
sys.path.insert(0,'/home/pi/hiwonder-toolbox')
from wifi_config_store import load_config_values
def q(a):return subprocess.check_output(a,text=True,timeout=5).strip()
c=load_config_values('/home/pi/hiwonder-toolbox/wifi_conf.py')
name=q(['nmcli','-g','GENERAL.CONNECTION','device','show','wlan0'])
state=q(['nmcli','-g','GENERAL.STATE','device','show','wlan0']).split()[0]
ssid=''
if name=='hiwonder-wlan0-client':
 ssid=q(['nmcli','--escape','no','-g','802-11-wireless.ssid','connection','show',name])
addresses=json.loads(q(['ip','-j','-4','addr','show','wlan0']))
ips=[]
for d in addresses:
 for a in d.get('addr_info',[]):
  if a.get('family')!='inet':continue
  v=ipaddress.IPv4Address(a['local'])
  if not (v.is_loopback or v.is_link_local or v.is_unspecified or v.is_multicast):ips.append(str(v))
print(json.dumps({'mode':c.get('WIFI_MODE'),'preferred_ssid':c.get('WIFI_STA_SSID',''),
 'ssid':ssid,'connected':name=='hiwonder-wlan0-client' and state=='100' and bool(ips),
 'ip':ips[0] if ips else '', 'fallback_ap':name=='hiwonder-wlan0-ap',
 'preference_saved':c.get('WIFI_MODE')==2 and bool(c.get('WIFI_STA_SSID')) and isinstance(c.get('WIFI_STA_PASSWORD'),str),
 'boot_autostart':subprocess.run(['systemctl','is-enabled','wifi.service'],capture_output=True,text=True,timeout=6).stdout.strip()=='enabled',
 'service_active':subprocess.run(['systemctl','is-active','wifi.service'],capture_output=True,text=True,timeout=6).stdout.strip()=='active'}))
'''


class WifiSetupError(RuntimeError):
    pass


def validate_wifi(ssid, password):
    if not isinstance(ssid,str) or not ssid or len(ssid.encode('utf-8'))>32:
        raise ValueError('Wi-Fi 名称不能为空，UTF-8 长度不能超过 32 字节')
    if any(ch in ssid for ch in ('\0','\r','\n')):
        raise ValueError('Wi-Fi 名称不能包含换行或空字符')
    if not isinstance(password,str) or any(ch in password for ch in ('\0','\r','\n')):
        raise ValueError('Wi-Fi 密码不能包含换行或空字符')
    if password and not (8<=len(password.encode('utf-8'))<=63 or re.fullmatch(r'[0-9a-fA-F]{64}',password)):
        raise ValueError('Wi-Fi 密码须为 8–63 字节或 64 位十六进制密钥；开放网络请选择无密码')


def remote_json(client,script,payload=None,sudo=False,timeout=12):
    command=('sudo -n ' if sudo else '')+'python3 -c '+shlex.quote(script)
    stdin,stdout,stderr=client.exec_command(command,timeout=timeout)
    try:
        if payload is not None:
            stdin.write(json.dumps(payload,ensure_ascii=True))
            stdin.flush()
            stdin.channel.shutdown_write()
        raw=stdout.read(2*1024*1024)
        if stdout.channel.recv_exit_status():
            # Remote stderr/commands may contain credentials; never forward it.
            raise WifiSetupError('车端操作未完成，请检查 SSH 用户的 sudo 权限和网络恢复服务')
        return json.loads(raw.decode('utf-8'))
    finally:
        stdout.channel.close()


def open_vehicle(host,profile_id,username,password,known_hosts,port=22,guard=None):
    import paramiko
    if not re.fullmatch(r'HW-[0-9A-Fa-f]{8}',profile_id or ''):
        raise WifiSetupError('请先选择具体的小车档案')
    if guard:
        try:guard()
        except Exception:raise WifiSetupError('电脑尚未连接所选小车热点') from None
    client=paramiko.SSHClient()
    if os.path.isfile(known_hosts):client.load_host_keys(known_hosts)
    alias=profile_id+'@'+host
    lookup=alias if int(port)==22 else '[%s]:%s'%(alias,port)
    keys=client.get_host_keys()
    # Reuse this vehicle's verified key when DHCP assigns a different address.
    if keys.lookup(lookup) is None:
        for saved in list(keys.keys()):
            if saved.startswith(profile_id+'@') or saved.startswith('['+profile_id+'@'):
                for kind,key in keys[saved].items():keys.add(lookup,kind,key)
                break
    class FirstContact(paramiko.MissingHostKeyPolicy):
        def missing_host_key(self,ssh,hostname,key):
            # Remember only in memory until the CPU identity has been checked.
            ssh.get_host_keys().add(hostname,key.get_name(),key)
    client.set_missing_host_key_policy(paramiko.RejectPolicy() if keys.lookup(lookup) else FirstContact())
    sock=None
    try:
        sock=socket.create_connection((host,int(port)),timeout=5)
        client.connect(alias,port=int(port),sock=sock,username=username,password=password or None,
                       look_for_keys=not bool(password),allow_agent=not bool(password),
                       timeout=5,auth_timeout=6,banner_timeout=6)
        identity=remote_json(client,IDENTITY_SCRIPT)
        if identity.get('profile_id','').upper()!=profile_id.upper():
            raise WifiSetupError('连接到的车辆与所选档案不符，未修改配置')
        Path(known_hosts).parent.mkdir(parents=True,exist_ok=True)
        client.save_host_keys(known_hosts)
        return client
    except WifiSetupError:
        client.close()
        if sock:sock.close()
        raise
    except Exception:
        client.close()
        if sock:sock.close()
        raise WifiSetupError('SSH 连接或认证失败，请检查地址、登录密码和已保存的主机指纹') from None


def _record(directory,name,data):
    Path(directory).mkdir(parents=True,exist_ok=True)
    path=Path(directory)/name
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    return str(path)


def configure_wifi(profile,host,transport,ssid,wifi_password,*,username,login_password,
                   known_hosts,directory,port=22,guard=None,cancel=None,progress=lambda text:None,
                   wait_seconds=155,clock=time.monotonic,sleep=time.sleep):
    """Back up, atomically submit once, then observe; never replay a lost write."""
    validate_wifi(ssid,wifi_password)
    stopped=(cancel.is_set if cancel is not None else lambda:False)
    profile_id=profile['id']
    candidates=list(dict.fromkeys(v for v in (host,profile.get('host')) if v))
    client=None
    for candidate in candidates:
        if stopped():return {'state':'cancelled','submitted':False}
        progress('连接小车 '+candidate)
        try:
            client=open_vehicle(candidate,profile_id,username,login_password,known_hosts,port,
                                guard if transport=='wifi' and candidate==host else None)
            break
        except WifiSetupError as exc:
            if '档案不符' in str(exc):raise
    if client is None:raise WifiSetupError('无法连接所选车辆，请先连接网线、原局域网或小车热点，并核对登录密码')
    result={'state':'pending','submitted':False,'ssid':ssid,'profile_id':profile_id,
            'management_host':candidate,'backup_dir':str(directory),'ip':'','reachable':False}
    try:
        progress('备份当前 Wi-Fi 配置')
        backup=remote_json(client,BACKUP_SCRIPT,sudo=True)
        if not backup.get('ok'):raise WifiSetupError('车端尚未安装 STA 自动恢复服务，未修改配置')
        _record(directory,'before.json',backup)
        _record(directory,'operation.json',result)
        if stopped():
            result['state']='cancelled'
            return result
        progress('下发新 Wi-Fi 配置')
        payload={'profile_id':profile_id,'ssid':ssid,'password':wifi_password,
                 'expected_sha256':backup['files'][CONFIG_PATH]['sha256'],'restart_service':not bool(wifi_password)}
        if not wifi_password:
            previous=backup['files'].get(ADAPTER_PATH,{})
            old=base64.b64decode(previous.get('data','')).decode('utf-8')
            if 'OPEN_NETWORK_SUPPORTED = True' not in old:
                if previous.get('sha256')!=PRE_OPEN_ADAPTER_SHA256:
                    raise WifiSetupError('该车恢复模块版本未识别，尚未下发；请先升级车端开放网络支持')
                source_path=Path(__file__).resolve().parents[3]/'tools/vehicle_network/wifi_recovery_adapter.py'
                if not source_path.is_file():raise WifiSetupError('请使用包含 tools/vehicle_network 的完整项目配置开放网络')
                source=source_path.read_text(encoding='utf-8')
                if 'OPEN_NETWORK_SUPPORTED = True' not in source:raise WifiSetupError('本机恢复模块需要更新，尚未下发')
                compile(source,str(source_path),'exec')
                payload.update(adapter=base64.b64encode(source.encode()).decode(),adapter_expected_sha256=previous['sha256'])
                progress('备份已完成，准备更新开放网络兼容组件并连接')
        # A lost acknowledgement is ambiguous: do not retry this mutation.
        result['submitted']=True
        result['possible_modified_files']=[CONFIG_PATH,BOOT_LINK_PATH]+([ADAPTER_PATH] if payload.get('adapter') else [])
        try:
            response=remote_json(client,WRITE_SCRIPT,payload,sudo=True,timeout=90)
        except Exception:
            result.update(state='uncertain',message='下发期间连接断开，配置可能已生效；请接入目标局域网后查找新地址')
            return result
        if not response.get('ok'):
            result['submitted']=False
            raise WifiSetupError('车端配置或车辆身份已变化，本次未写入，请重新读取后再试')
        result['modified_files']=response['modified_files']
        for field in ('preference_saved','boot_autostart','service_active'):
            result[field]=bool(response.get(field))
        if not result['preference_saved']:
            result['message']='配置写入后未通过长期保存校验，尚不能确认开机首选，请检查备份记录'
            return result
        if not result['boot_autostart']:
            result['message']='新网络已保存，但未能确认网络服务开机自启，请检查 wifi.service 的启用状态'
            return result
        if not response.get('restart_ok',True):
            result['message']='配置已保存，但网络服务未能启动或重新加载，请查看备份记录并检查 wifi.service'
            return result
        if not result['service_active']:
            result['message']='开机首选已保存，但网络服务尚未运行，请检查 wifi.service'
            return result
        deadline=clock()+wait_seconds
        progress('已保存为开机首选，等待小车连接目标 Wi-Fi')
        while not stopped() and clock()<deadline:
            try:status=remote_json(client,STATUS_SCRIPT)
            except Exception:
                result['message']='配置已下发，原连接已中断；电脑接入目标局域网后可查找新地址'
                return result
            try:_record(directory,'status.json',status)
            except OSError:result['record_error']=True
            result['preference_saved']=bool(status.get('preference_saved') and
                                            status.get('preferred_ssid')==ssid and status.get('mode')==2)
            result['boot_autostart']=bool(status.get('boot_autostart'))
            result['service_active']=bool(status.get('service_active'))
            if status.get('preferred_ssid')!=ssid or status.get('mode')!=2:
                result['message']='检测到车端配置被其他操作更改，请核对后重新查询'
                return result
            if not result['preference_saved']:
                result['message']='无法确认当前网络仍是开机首选，请检查车端配置'
                return result
            if not result['boot_autostart']:
                result['message']='网络已保存，但网络服务当前未设置开机自启，请检查 wifi.service'
                return result
            if not result['service_active']:
                result['message']='网络已保存，但网络服务已停止，请检查 wifi.service'
                return result
            if status.get('connected') and status.get('ssid')==ssid:
                result.update(state='connected',ip=status['ip'])
                try:
                    peer=open_vehicle(result['ip'],profile_id,username,login_password,known_hosts,port)
                    peer.close();result['reachable']=True
                except WifiSetupError:pass
                return result
            sleep(2)
        result['message']='已停止等待，车端仍会按 STA 优先策略尝试；连接失败时临时回退 AP' if stopped() else \
                          '尚未确认加入目标 Wi-Fi；请核对名称、密码和信号，小车会临时回退 AP 并自动重试'
        return result
    finally:
        client.close()
        try:_record(directory,'operation.json',result)
        except OSError:result['record_error']=True


def find_wifi(profile,ssid,candidates,*,username,login_password,known_hosts,port=22,cancel=None):
    """Port discovery is only a candidate list; verify keys and CPU before use."""
    import paramiko
    if not os.path.isfile(known_hosts) or not any(
        name.startswith(profile['id']+'@') or name.startswith('['+profile['id']+'@')
        for name in paramiko.HostKeys(known_hosts)):
        raise WifiSetupError('请先通过原连接核对车辆，再查找新地址')
    for host in candidates:
        if cancel is not None and cancel.is_set():break
        client=None
        try:
            client=open_vehicle(host,profile['id'],username,login_password,known_hosts,port)
            status=remote_json(client,STATUS_SCRIPT)
            if status.get('connected') and status.get('ssid')==ssid and status.get('ip')==host:
                return {'state':'connected','submitted':True,'profile_id':profile['id'],
                        'ssid':ssid,'ip':host,'reachable':True,
                        'preference_saved':bool(status.get('preference_saved') and status.get('preferred_ssid')==ssid),
                        'boot_autostart':bool(status.get('boot_autostart')),'service_active':bool(status.get('service_active'))}
        except Exception:pass
        finally:
            if client:client.close()
    return {'state':'pending','submitted':True,'profile_id':profile['id'],'ssid':ssid,'ip':'','reachable':False,
            'message':'未找到身份匹配且已连接目标 Wi-Fi 的小车，请确认电脑也已接入目标局域网'}
