"""SSH PTY plus read-only filesystem queries; no SFTP/server installation required."""
import base64
import codecs
import concurrent.futures
import hashlib
import json
import os
import queue
import shlex
import threading
import time

LIST_SCRIPT = r'''
import os,sys,json,stat,pwd
home=pwd.getpwuid(os.getuid()).pw_dir
requested=sys.argv[1]
path=os.path.realpath(home if requested in ('','~') else os.path.expanduser(requested))
items=[]
with os.scandir(path) as entries:
 for index,e in enumerate(entries):
  if index>=5000: break
  try:
   s=e.stat(follow_symlinks=False)
   items.append(dict(name=e.name,is_dir=e.is_dir(follow_symlinks=True),is_link=e.is_symlink(),
                     size=s.st_size,mtime=s.st_mtime,permissions=stat.filemode(s.st_mode)))
  except OSError as exc:
   items.append(dict(name=e.name,is_dir=False,size=0,mtime=0,permissions='?',error=str(exc)))
items.sort(key=lambda e:(not e['is_dir'],e['name'].casefold()))
print(json.dumps(dict(path=path,home=home,entries=items,truncated=len(items)>=5000),ensure_ascii=True))
'''

READ_SCRIPT = r'''
import os,sys,json,stat,base64
path=sys.argv[1]
s=os.stat(path)
if not stat.S_ISREG(s.st_mode): raise ValueError('Only regular files can be previewed')
with open(path,'rb') as f: data=f.read(262145)
print(json.dumps(dict(path=path,size=s.st_size,truncated=len(data)>262144,
                     data=base64.b64encode(data[:262144]).decode('ascii'))))
'''


def query_command(operation, path, scope="host"):
    script = LIST_SCRIPT if operation == "directory" else READ_SCRIPT
    args = ["python3", "-c", script, path]
    if scope == "container":
        args = ["sudo", "-n", "docker", "exec", "-u", "ubuntu", "ClawPi"] + args
    elif scope != "host":
        raise ValueError("未知文件系统视图")
    return " ".join(shlex.quote(arg) for arg in args)


class SSHSession:
    """All I/O runs off the Tk thread. Every event carries its connection generation."""
    def __init__(self, known_hosts, events=None, connection_guard=None):
        self.known_hosts = known_hosts
        self.events = events if events is not None else queue.Queue(maxsize=1024)
        self.connection_guard = connection_guard
        self.generation = 0
        self.client = self.channel = None
        self.connected = False
        self._state_lock = threading.RLock()
        self.outbound = queue.Queue(maxsize=128)
        self.dimensions = (100, 30)
        self._pool = concurrent.futures.ThreadPoolExecutor(max_workers=2, thread_name_prefix="ssh-files")

    def _event(self, generation, kind, payload):
        while generation == self.generation:
            try:
                self.events.put((generation, kind, payload), timeout=.1)
                return
            except queue.Full:
                pass

    def connect(self, host, username, password="", port=22, profile_id=""):
        self.disconnect()
        generation = self.generation
        guard = self.connection_guard
        outbound = self.outbound

        def worker():
            client = channel = None
            try:
                import paramiko
                if guard:
                    guard()
                client = paramiko.SSHClient()
                client.load_system_host_keys()
                if os.path.isfile(self.known_hosts):
                    client.load_host_keys(self.known_hosts)
                owner = self
                class RememberNewKey(paramiko.MissingHostKeyPolicy):
                    def missing_host_key(self, ssh, hostname, key):
                        fingerprint = base64.b64encode(hashlib.sha256(key.asbytes()).digest()).decode().rstrip('=')
                        ssh.get_host_keys().add(hostname,key.get_name(),key)
                        os.makedirs(os.path.dirname(owner.known_hosts),exist_ok=True)
                        ssh.save_host_keys(owner.known_hosts)
                        owner._event(generation,"notice","首次连接指纹 SHA256:"+fingerprint)
                client.set_missing_host_key_policy(RememberNewKey())
                # Distinct aliases for the two cars which share the AP address.
                import socket
                sock = socket.create_connection((host,int(port)),timeout=6)
                alias = (profile_id+"@" if profile_id else "")+host
                try:
                    client.connect(alias,port=int(port),sock=sock,username=username,
                                   password=password or None,look_for_keys=not bool(password),
                                   allow_agent=not bool(password),timeout=6,auth_timeout=8,banner_timeout=8)
                except Exception:
                    sock.close()
                    raise
                if generation != self.generation:
                    return
                client.get_transport().set_keepalive(10)
                channel = client.invoke_shell(term="xterm-256color",width=self.dimensions[0],height=self.dimensions[1])
                channel.settimeout(.25)
                if generation != self.generation:
                    return
                with self._state_lock:
                    if generation != self.generation:
                        return
                    self.client,self.channel,self.connected = client,channel,True
                self._event(generation,"connected",{"host":host,"username":username})
                decoder = codecs.getincrementaldecoder("utf-8")("replace")
                applied,guard_at = self.dimensions,time.monotonic()
                while generation == self.generation and client.get_transport().is_active():
                    if guard and time.monotonic()-guard_at > 3:
                        guard()
                        guard_at = time.monotonic()
                    if self.dimensions != applied:
                        applied=self.dimensions
                        channel.resize_pty(width=applied[0],height=applied[1])
                    try:
                        outgoing = outbound.get_nowait()
                        if generation == self.generation:
                            channel.sendall(outgoing)
                    except queue.Empty:
                        pass
                    if channel.recv_ready():
                        data=channel.recv(32768)
                        if not data:
                            break
                        text=decoder.decode(data)
                        if text:
                            self._event(generation,"data",text)
                    elif channel.closed or channel.exit_status_ready():
                        break
                    else:
                        time.sleep(.02)
            except Exception as exc:
                # No command replay and no password included in events.
                self._event(generation,"error",type(exc).__name__+": "+str(exc))
            finally:
                if channel:
                    channel.close()
                if client:
                    client.close()
                with self._state_lock:
                    if generation == self.generation:
                        self.connected=False
                        self.client=self.channel=None
                        self._event(generation,"disconnected","SSH 会话已断开")
        threading.Thread(target=worker,daemon=True,name="ssh-terminal").start()

    def send(self, text):
        with self._state_lock:
            if not self.connected:
                return False
            payload=text.encode("utf-8") if isinstance(text,str) else text
            if not payload:
                return True
            try:
                self.outbound.put_nowait(payload)
                return True
            except queue.Full:
                return False

    def resize(self, columns, rows):
        self.dimensions=(max(30,min(240,columns)),max(8,min(100,rows)))

    def _query(self, operation, path, scope, request_id):
        client,generation,guard=self.client,self.generation,self.connection_guard
        if not self.connected or client is None:
            self._event(generation,"operation_error",{"request_id":request_id,"error":"SSH 未连接"})
            return
        def worker():
            try:
                if generation != self.generation:
                    return
                if guard:
                    guard()
                _,stdout,stderr=client.exec_command(query_command(operation,path,scope),timeout=10)
                raw=stdout.read(4*1024*1024)
                error=stderr.read(65536).decode("utf-8","replace").strip()
                status=stdout.channel.recv_exit_status()
                if status:
                    raise OSError(error or "远端读取失败，退出码 %d"%status)
                data=json.loads(raw.decode("utf-8"))
                data.update(request_id=request_id,scope=scope)
                self._event(generation,operation,data)
            except Exception as exc:
                self._event(generation,"operation_error",{"request_id":request_id,"error":str(exc)})
        self._pool.submit(worker)

    def list_directory(self,path,scope="host",request_id=0):
        self._query("directory",path,scope,request_id)

    def read_file(self,path,scope="host",request_id=0):
        self._query("file",path,scope,request_id)

    def disconnect(self):
        with self._state_lock:
            self.generation+=1
            self.connected=False
            channel,client=self.channel,self.client
            self.channel=self.client=None
            # A previous worker never consumes the next vehicle's keystrokes.
            self.outbound=queue.Queue(maxsize=128)
        if channel:
            channel.close()
        if client:
            client.close()

    def close(self):
        self.disconnect()
        self._pool.shutdown(wait=False,cancel_futures=True)
