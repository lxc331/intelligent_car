"""PC adapter for the car's installed Nav2, with session-scoped temporary files.

SSH transport is shared with mapping. Navigation never edits vendor files or
starts robot hardware again; one car runtime owns actions and the velocity gate.
"""
import base64
import hashlib
import json
import math
from pathlib import Path
import shlex
import threading
import time
import uuid

from carapp.core.mapping_remote import MappingRemote, MappingRemoteError
from carapp.core.navigation_remote_scripts import REMOTE_SCRIPT
from carapp.core.navigation_config import LAUNCH_SOURCE


class NavigationRemoteError(MappingRemoteError):
    pass


def remote_command(operation, **values):
    payload = dict(values, op=operation)
    if operation in ('preflight', 'start'):
        payload['_script'] = REMOTE_SCRIPT
    return shlex.join(['sudo','-n','docker','exec','-u','ubuntu','ClawPi',
                       'python3','-c',REMOTE_SCRIPT,json.dumps(payload,ensure_ascii=True)])


def validate_bundle(bundle):
    import yaml
    yaml_text = bundle['yaml_text']
    image = bundle['image_bytes']
    if not isinstance(yaml_text,str) or len(yaml_text.encode('utf8')) > 65536:
        raise ValueError('地图描述文件过大或格式错误')
    if not isinstance(image,bytes) or not 10 <= len(image) <= 4000256:
        raise ValueError('地图图像为空或超过 400 万格限制')
    description = yaml.safe_load(yaml_text)
    if not isinstance(description,dict) or description.get('image') != 'map.pgm':
        raise ValueError('导航地图须由本地导入器规范化为 map.pgm')
    resolution = float(description.get('resolution',0.))
    origin = description.get('origin')
    if not math.isfinite(resolution) or not .001 <= resolution <= 2.:
        raise ValueError('地图分辨率无效')
    if not isinstance(origin,(list,tuple)) or len(origin) != 3 or not all(math.isfinite(float(x)) for x in origin):
        raise ValueError('地图原点无效')
    yaw = float(origin[2])
    if abs(math.atan2(math.sin(yaw),math.cos(yaw))) > 1e-8:
        raise ValueError('当前车端 Nav2 不支持带旋转原点的地图，请使用轴向地图')
    if description.get('mode','trinary') != 'trinary' or description.get('negate',0) != 0:
        raise ValueError('导航地图须使用规范化的三值灰度格式')
    if not image.startswith(b'P5\n'):
        raise ValueError('导航图像应是规范化的 P5 PGM')
    return {'map.yaml':yaml_text.encode('utf8'),'map.pgm':image}


def target_values(x, y, yaw):
    values = dict(x=float(x),y=float(y),yaw=float(yaw))
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError('导航坐标必须是有限数值')
    return values


class NavigationRemote(MappingRemote):
    """Synchronous foreground methods plus independent leased motion intent.

    request_cancel() is intentionally nonblocking and bypasses the foreground
    SSH lock. A failed revoke channel is backed by the 2-second motion lease.
    Once a lease expires the car latches the old goal off; reconnection cannot
    resume it. Callers must explicitly submit a new navigation goal.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._intent_lock = threading.RLock()
        self._intent_epoch = 0
        self._motion_allowed = False
        self._lease_exists = False
        self._heartbeat_done = threading.Event()
        self._heartbeat_thread = None
        self._prepared = False
        self._clock_offset = None

    def _remote(self, operation, scope='container', timeout=12, **values):
        if scope != 'container':
            raise ValueError('导航操作只在 ClawPi 容器中执行')
        values.setdefault('token',self.token)
        raw = self._execute(remote_command(operation,**values),timeout=timeout,limit=1048576)
        try:
            value = json.loads(raw.decode('utf8'))
            if not isinstance(value,dict):
                raise ValueError('not an object')
            return value
        except (ValueError,UnicodeError) as exc:
            raise NavigationRemoteError('车端返回无法解析的导航结果') from exc

    def connect(self):
        # MappingRemote.connect calls this overridden _remote('preflight'), so
        # the common host key/authentication code never starts or probes SLAM.
        return super().connect()

    def capture_intent(self):
        with self._intent_lock:
            return self._intent_epoch

    def _intent(self):
        with self._intent_lock:
            result = dict(epoch=self._intent_epoch,allow=self._motion_allowed)
            if self._clock_offset is not None:
                result['expires'] = time.monotonic()+self._clock_offset+2.
            return result

    def _disallow(self):
        with self._intent_lock:
            self._intent_epoch += 1
            self._motion_allowed = False
            return self._intent_epoch

    def _start_heartbeat(self):
        self._heartbeat_done.clear()
        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            return
        def run():
            while not self._heartbeat_done.wait(.4):
                if not self._lease_exists:
                    continue
                try:
                    self._remote('heartbeat',timeout=3,**self._intent())
                except Exception:
                    # Do not silently grant a previous goal after a successful
                    # later heartbeat. The next explicit goal needs a new epoch.
                    self._disallow()
        self._heartbeat_thread = threading.Thread(target=run,name='navigation-lease',daemon=True)
        self._heartbeat_thread.start()

    def start(self, map_bundle, speed=.06):
        speed = float(speed)
        if not math.isfinite(speed) or not .02 <= speed <= .15:
            raise ValueError('导航速度应在 0.02–0.15 m/s')
        files = validate_bundle(map_bundle)
        folder = Path(__file__).parent
        files['runtime.py'] = (folder/'navigation_runtime.py').read_bytes()
        files['config_builder.py'] = (folder/'navigation_config.py').read_bytes()
        files['navigation.launch.py'] = LAUNCH_SOURCE.encode('utf8')
        with self._lock:
            if self._lease_exists:
                raise NavigationRemoteError('导航服务已启动，请先结束再重新载入地图')
            if self._prepared:
                self.token = uuid.uuid4().hex
            self._disallow()
            # A remote instance represents one stack lifetime. Reconnect/create
            # a new instance after stop instead of reusing an uploaded map.
            prepared = self._remote('prepare',**self._intent())
            # The response timestamp is earlier than local receipt. This is a
            # conservative lower bound on the server clock offset: delayed SSH
            # heartbeats expire at issue time, not two seconds after arrival.
            if 'server_monotonic' not in prepared:
                raise NavigationRemoteError('车端未返回导航租约时钟')
            self._clock_offset = float(prepared['server_monotonic'])-time.monotonic()
            self._prepared = True
            self._lease_exists = True
            self._start_heartbeat()
            descriptors = {}
            try:
                for name,raw in files.items():
                    for offset in range(0,len(raw),32768):
                        chunk = raw[offset:offset+32768]
                        answer = self._remote('upload',name=name,offset=offset,
                                              chunk=base64.b64encode(chunk).decode('ascii'))
                        if answer.get('written') != len(chunk):
                            raise NavigationRemoteError('导航文件上传未完成：'+name)
                    descriptors[name] = dict(size=len(raw),sha256=hashlib.sha256(raw).hexdigest())
                return self._remote('start',timeout=38,files=descriptors,speed=speed)
            except Exception:
                self._disallow()
                try:
                    self._remote('stop',timeout=12,**self._intent())
                except Exception:
                    pass
                self._lease_exists = False
                raise

    def status(self):
        with self._lock:
            if not self.connected:
                self._disallow()
                return dict(connected=False,running=False,owned=False,ready=False,goal_state='canceled',
                            motion_stopped=False,motion_lease_seconds=2.,
                            detail='SSH 已断开；车端控制租约到期后停车')
            began = time.monotonic()
            result = self._remote('status')
            if 'pose' in result:
                result['pose_age'] = float(result.get('pose_age', 0.)) + time.monotonic() - began
            if result.get('goal_state') in ('succeeded','canceled','aborted','error'):
                with self._intent_lock:
                    self._motion_allowed = False
            return result

    def initial_pose(self, x, y, yaw):
        with self._lock:
            self._disallow()
            return self._remote('initial_pose',values=target_values(x,y,yaw),
                                epoch=self.capture_intent(),request_timeout=5,timeout=8)

    def plan(self, x, y, yaw):
        with self._lock:
            return self._remote('plan',values=target_values(x,y,yaw),epoch=self.capture_intent(),
                                request_timeout=18,timeout=21)

    def navigate(self, x, y, yaw, *, intent_epoch=None):
        values = target_values(x,y,yaw)
        with self._lock:
            if not self._lease_exists or self._clock_offset is None:
                raise NavigationRemoteError('请先启动导航服务并确认定位')
            with self._intent_lock:
                if intent_epoch is not None and intent_epoch != self._intent_epoch:
                    raise NavigationRemoteError('该导航请求已被停止操作作废')
                self._intent_epoch += 1
                epoch = self._intent_epoch
                self._motion_allowed = True
            try:
                # Commit the new grant before submitting the action; the car
                # must not observe the preceding idle lease as a cancellation.
                self._remote('heartbeat',timeout=3,**self._intent())
                with self._intent_lock:
                    if epoch != self._intent_epoch or not self._motion_allowed:
                        raise NavigationRemoteError('导航授权提交期间已取消')
                result = self._remote('navigate',values=values,epoch=epoch,request_timeout=8,timeout=11)
                with self._intent_lock:
                    if epoch != self._intent_epoch:
                        raise NavigationRemoteError('导航请求提交期间已取消')
                return result
            except Exception:
                with self._intent_lock:
                    if self._intent_epoch == epoch:
                        self._intent_epoch += 1
                        self._motion_allowed = False
                self.request_cancel()
                raise

    def request_cancel(self):
        epoch = self._disallow()
        token = self.token
        if not self._lease_exists or not self.connected:
            return False
        def revoke():
            try:
                self._remote('revoke',timeout=3,token=token,epoch=epoch,allow=False)
            except Exception:
                pass
        threading.Thread(target=revoke,name='navigation-emergency-revoke',daemon=True).start()
        return True

    def cancel(self):
        epoch = self._disallow()
        with self._lock:
            if not self.connected:
                return dict(connected=False,running=False,goal_state='canceled',canceled=False,
                            motion_stopped=False,motion_lease_seconds=2.)
            if self._lease_exists:
                self._remote('revoke',timeout=3,epoch=epoch,allow=False)
            return self._remote('cancel',epoch=epoch,request_timeout=8,timeout=11)

    def stop(self):
        self.request_cancel()
        with self._lock:
            if not self.connected:
                self._lease_exists = False
                time.sleep(2.2)
                return dict(connected=False,running=False,owned=False,ready=False,
                            motion_stopped=True,motion_lease_seconds=0.,
                            detail='SSH 未连接；车端运动租约最多 2 秒后失效，进程将在会话租约到期后结束')
            try:
                return self._remote('stop',timeout=12,**self._intent())
            finally:
                self._lease_exists = False

    def close(self):
        self.request_cancel()
        self._heartbeat_done.set()
        self._lease_exists = False
        super().close()
