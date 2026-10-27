"""Synchronous, UI-independent SSH adapter for the car's existing SLAM Toolbox.

Call from a worker thread. This adapter never launches the full robot bringup,
never publishes velocity, and never writes to the car's source workspace.
"""
import base64
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shlex
import socket
import tempfile
import threading
import time
import uuid

from carapp.core.mapping_remote_scripts import REMOTE_SCRIPT, SUPERVISOR


class MappingRemoteError(RuntimeError):
    """A bounded remote operation failed; its text contains no login password."""


class MappingSaveError(MappingRemoteError):
    """A save failed or only one destination was completed."""

    def __init__(self, message, partial=None):
        super().__init__(message)
        self.partial = partial or {}


def validate_remote_path(path):
    path = str(path)
    if (not path.startswith('/') or '\\' in path or
            any(ord(char) < 32 for char in path) or '..' in PurePosixPath(path).parts):
        raise ValueError('请选择绝对 Linux 目录；不允许控制字符、反斜杠或 ..')
    if len(path) > 2048:
        raise ValueError('保存路径过长')
    return str(PurePosixPath(path))


def validate_map_name(name):
    name = str(name).strip()
    # Windows reserved names are rejected because maps must also live on this PC.
    reserved = {'CON', 'PRN', 'AUX', 'NUL'} | {'COM%d' % i for i in range(1, 10)} | {'LPT%d' % i for i in range(1, 10)}
    if not re.fullmatch(r'[\w\-][\w\- ]{0,63}', name, re.UNICODE) or name.upper() in reserved:
        raise ValueError('地图名限 1–64 个中英文、数字、下划线、短横线或空格，不允许路径及保留名称')
    return name


def remote_command(operation, scope='container', **values):
    if scope not in ('container', 'host'):
        raise ValueError('未知车端文件系统')
    payload = dict(values, op=operation, scope=scope)
    if operation in ('preflight', 'start', 'stop', 'status', 'save'):
        if scope != 'container':
            raise ValueError('ROS 建图操作必须在 ClawPi 容器中执行')
        payload['_script'] = REMOTE_SCRIPT
    args = ['python3', '-c', REMOTE_SCRIPT, json.dumps(payload, ensure_ascii=True)]
    if scope == 'container':
        args = ['sudo', '-n', 'docker', 'exec', '-u', 'ubuntu', 'ClawPi'] + args
    return shlex.join(args)


class MappingRemote:
    """An owner token limits stop() to exactly this session's SLAM process group.

    A remote supervisor expires after 90 seconds without status/save activity.
    close() only disconnects SSH; callers should stop() explicitly before close.
    """

    def __init__(self, host, username, password='', port=22, profile_id='',
                 known_hosts=None, connection_guard=None):
        self.host, self.username, self.port = host, username, int(port)
        self._password = password
        self.profile_id = profile_id
        if known_hosts is None:
            from carapp.config import APP_DIR
            known_hosts = os.path.join(APP_DIR, 'config', 'ssh_known_hosts')
        self.known_hosts = str(known_hosts)
        self.connection_guard = connection_guard
        self.token = uuid.uuid4().hex
        self.client = None
        self.preflight = None
        self._lock = threading.RLock()
        self._closed = threading.Event()
        self.fingerprint = None

    @property
    def connected(self):
        client = self.client
        transport = client.get_transport() if client else None
        return bool(transport and transport.is_active() and not self._closed.is_set())

    def _guard(self, check_network=True):
        if self._closed.is_set():
            raise MappingRemoteError('建图 SSH 已关闭')
        if check_network and self.connection_guard:
            self.connection_guard()

    def connect(self):
        """Read-only preflight; no files, services, sensors or motors are started."""
        with self._lock:
            import paramiko
            self._closed.clear()
            self._guard()
            if self.client:
                self.client.close()
                self.client = None
            client = paramiko.SSHClient()
            client.load_system_host_keys()
            if os.path.isfile(self.known_hosts):
                client.load_host_keys(self.known_hosts)
            owner = self

            class RememberNewKey(paramiko.MissingHostKeyPolicy):
                def missing_host_key(self, ssh, hostname, key):
                    owner.fingerprint = 'SHA256:' + base64.b64encode(hashlib.sha256(key.asbytes()).digest()).decode().rstrip('=')
                    ssh.get_host_keys().add(hostname, key.get_name(), key)
                    Path(owner.known_hosts).parent.mkdir(parents=True, exist_ok=True)
                    ssh.save_host_keys(owner.known_hosts)

            client.set_missing_host_key_policy(RememberNewKey())
            sock = socket.create_connection((self.host, self.port), timeout=6)
            alias = (self.profile_id + '@' if self.profile_id else '') + self.host
            try:
                client.connect(alias, port=self.port, sock=sock, username=self.username,
                               password=self._password or None, look_for_keys=not bool(self._password),
                               allow_agent=not bool(self._password), timeout=6, auth_timeout=8, banner_timeout=8)
                client.get_transport().set_keepalive(10)
                self.client = client
                info = self._remote('preflight', _timeout=20, timeout=25)
                info['connected'] = True
                info['fingerprint'] = self.fingerprint
                self.preflight = info
                return info
            except Exception:
                self.client = None
                client.close()
                sock.close()
                raise

    def _execute(self, command, timeout=25, limit=1024 * 1024, sink=None):
        """Drain stdout and stderr together, with a wall deadline and byte caps."""
        self._guard()
        if not self.connected:
            raise MappingRemoteError('建图 SSH 未连接')
        channel = self.client.get_transport().open_session(timeout=6)
        output, error, count = bytearray(), bytearray(), 0
        deadline = time.monotonic() + timeout
        guard_at = time.monotonic() + 3
        eof = set()
        def receive(block, stderr=False):
            nonlocal count
            if stderr:
                error.extend(block)
                if len(error) > 12000:
                    del error[:-12000]
            else:
                count += len(block)
                if count > limit:
                    raise MappingRemoteError('远端响应超过允许大小')
                if sink is None:
                    output.extend(block)
                else:
                    sink.write(block)
        try:
            channel.exec_command(command)
            # Exit status and drained streams are different SSH events. A
            # large Docker stdout can still have tail packets in transit.
            channel.settimeout(0.)
            while True:
                now = time.monotonic()
                self._guard(check_network=now >= guard_at)
                if now >= guard_at:
                    guard_at = now + 3
                if now > deadline:
                    raise TimeoutError('建图远端操作超时；不会自动重启')
                progressed = False
                while channel.recv_ready():
                    self._guard(check_network=False)
                    block = channel.recv(65536)
                    if not block:
                        break
                    receive(block)
                    progressed = True
                while channel.recv_stderr_ready():
                    self._guard(check_network=False)
                    block = channel.recv_stderr(65536)
                    if not block:
                        break
                    receive(block, stderr=True)
                    progressed = True
                finished = channel.exit_status_ready()
                if finished:
                    # recv_ready=False means no buffered bytes, not EOF. Read
                    # nonblocking until both public recv APIs return b''.
                    for stream, reader in (('stdout', channel.recv), ('stderr', channel.recv_stderr)):
                        if stream in eof:
                            continue
                        try:
                            block = reader(65536)
                        except socket.timeout:
                            continue
                        if block:
                            receive(block, stderr=stream == 'stderr')
                            progressed = True
                        else:
                            eof.add(stream)
                if finished and len(eof) == 2 and not channel.recv_ready() and not channel.recv_stderr_ready():
                    status = channel.recv_exit_status()
                    if status:
                        message = error.decode('utf-8', 'replace').strip() or '退出码 %d' % status
                        raise MappingRemoteError('车端操作失败：' + message)
                    return bytes(output)
                if not progressed:
                    time.sleep(.015)
        finally:
            channel.close()

    def _remote(self, operation, scope='container', timeout=25, **values):
        raw = self._execute(remote_command(operation, scope, **values), timeout)
        try:
            return json.loads(raw.decode('utf-8'))
        except (ValueError, UnicodeError) as exc:
            raise MappingRemoteError('车端返回了无法解析的建图结果') from exc

    def list_directories(self, path, scope='container'):
        with self._lock:
            return self._remote('directories', scope, path=validate_remote_path(path))

    def start(self):
        with self._lock:
            return self._remote('start', token=self.token, supervisor=SUPERVISOR, _timeout=25, timeout=30)

    def status(self):
        with self._lock:
            if not self.connected:
                return dict(connected=False, running=False, owned=False, detail='建图 SSH 未连接')
            return self._remote('status', token=self.token, _timeout=12, timeout=17)

    def stop(self):
        with self._lock:
            if not self.connected:
                return dict(connected=False, running=False, owned=False, status='disconnected',
                            detail='SSH 未连接；本次进程将在 90 秒租约到期后退出')
            return self._remote('stop', token=self.token, _timeout=15, timeout=20)

    def _download(self, remote_path, scope, target, descriptor):
        expected = int(descriptor['size'])
        if expected <= 0 or expected > 256 * 1024 * 1024:
            raise MappingRemoteError('地图文件为空或超过 256 MiB 限制')
        with open(target, 'xb') as stream:
            self._execute(remote_command('read', scope, path=remote_path), timeout=60,
                          limit=expected, sink=stream)
        digest = hashlib.sha256()
        with open(target, 'rb') as stream:
            for block in iter(lambda: stream.read(262144), b''):
                digest.update(block)
        if os.path.getsize(target) != expected or digest.hexdigest() != descriptor['sha256']:
            raise MappingRemoteError('地图下载大小或 SHA256 校验失败：' + descriptor['name'])

    def _heartbeat(self):
        """Lightweight lease renewal; never starts ROS or touches another owner."""
        return self._remote('heartbeat', token=self.token, timeout=10)

    def _bridge_host(self, container_dir, remote_dir, folder, files):
        target = self._remote('mkdir_unique', 'host', path=remote_dir, folder=folder)['path']
        for descriptor in files:
            self._heartbeat()
            name = descriptor['name']
            args = ['sudo', '-n', 'docker', 'cp', 'ClawPi:' + container_dir + '/' + name, target + '/' + name]
            self._execute(shlex.join(args), timeout=30, limit=65536)
            self._heartbeat()
        copied = self._remote('manifest', 'host', path=target)['files']
        if copied != files:
            raise MappingSaveError('容器到宿主机复制校验失败', dict(remote_dir=target, container_dir=container_dir))
        return target

    def save_map(self, remote_dir, name, local_root, scope='container', serialize=True):
        """Save once remotely, verify byte-identical copies, then publish locally.

        Every save gets a new child directory. Errors carry .partial with the
        remote result, so the UI must not report both destinations as successful.
        """
        remote_dir, name = validate_remote_path(remote_dir), validate_map_name(name)
        if scope not in ('host', 'container'):
            raise ValueError('未知车端文件系统')
        with self._lock:
            self._guard()
            root = Path(local_root).resolve()
            root.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime('%Y%m%d_%H%M%S')
            folder = name + '_' + stamp + '_' + uuid.uuid4().hex[:8]
            final = root / folder
            partial = dict(local_dir=None, remote_dir=None, files=[], serialized=False, warnings=[], scope=scope)
            stage = None
            try:
                canonical = self._remote('canonical_directory', scope, path=remote_dir)['path']
                canonical = validate_remote_path(canonical)
                partial['requested_remote_dir'] = remote_dir
                remote_dir = canonical
                # Host paths are deliberately bridged, never interpreted inside Docker.
                container_parent = remote_dir if scope == 'container' else '/tmp'
                self._heartbeat()
                result = self._remote('save', token=self.token, directory=container_parent, name=name,
                                      folder=folder, serialize=bool(serialize), _timeout=60, timeout=65)
                partial.update(result)
                partial['scope'] = scope
                if result.get('error'):
                    raise MappingSaveError('车端地图保存失败：' + result['error'], partial)
                self._heartbeat()
                files = result['files']
                names = [item['name'] for item in files]
                allowed = {'map.pgm', 'map.yaml', 'map.posegraph', 'map.data'}
                if not {'map.pgm', 'map.yaml'}.issubset(names) or len(set(names)) != len(names) or not set(names) <= allowed:
                    raise MappingSaveError('车端地图文件清单不完整或异常', partial)
                if result.get('serialized') and not {'map.posegraph', 'map.data'}.issubset(names):
                    raise MappingSaveError('继续建图状态缺少 posegraph/data 文件，未声明完整成功', partial)
                for item in files:
                    if not re.fullmatch('[a-f0-9]{64}', item.get('sha256', '')):
                        raise MappingSaveError('车端地图校验清单异常', partial)
                destination = validate_remote_path(result['remote_dir'])
                if destination != str(PurePosixPath(container_parent) / folder):
                    raise MappingSaveError('车端保存结果目录与请求不一致', partial)
                if scope == 'host':
                    partial['container_dir'] = destination
                    partial['temporary_paths'] = [destination]
                    partial['remote_dir'] = self._bridge_host(destination, remote_dir, folder, files)
                    try:
                        self._remote('cleanup_map', 'container', path=destination, files=files)
                        partial['temporary_paths'] = []
                    except Exception as exc:
                        partial['warnings'].append('宿主机地图已验证；容器临时副本未能清理：' + str(exc))
                stage = Path(tempfile.mkdtemp(prefix='.mapping_', dir=root))
                for descriptor in files:
                    self._heartbeat()
                    path = str(PurePosixPath(partial['remote_dir']) / descriptor['name'])
                    self._download(path, scope, stage / descriptor['name'], descriptor)
                    self._heartbeat()
                yaml_text = (stage / 'map.yaml').read_text(encoding='utf-8')
                if not re.search(r'(?m)^image:\s*map\.pgm\s*$', yaml_text):
                    raise MappingSaveError('地图 YAML 图像引用不便携，未发布到本地地图目录', partial)
                partial['local_dir'] = str(final)
                # The manifest is PC metadata, not part of the identical remote map files.
                (stage / 'manifest.json').write_text(json.dumps(partial, ensure_ascii=False, indent=2), encoding='utf-8')
                if final.exists():
                    raise MappingSaveError('本地地图目录已存在，未覆盖', partial)
                stage.rename(final)
                stage = None
                return partial
            except MappingSaveError as exc:
                partial.update(exc.partial)
                partial['local_dir'] = None
                exc.partial = partial
                raise
            except Exception as exc:
                partial['local_dir'] = None
                raise MappingSaveError('双端保存未完成：' + str(exc), partial) from exc
            finally:
                if stage is not None:
                    # Only exact files created in this unique local staging directory.
                    for item in stage.iterdir():
                        if item.is_file() and not item.is_symlink():
                            item.unlink()
                    try:
                        stage.rmdir()
                    except OSError:
                        pass

    def close(self):
        """Disconnect only. Never kill someone else's SLAM on window teardown."""
        self._closed.set()
        client, self.client = self.client, None
        if client:
            client.close()
