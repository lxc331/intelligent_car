# -*- coding: utf-8 -*-
"""路侧红绿灯广播接收（UDP + JSON），不依赖 Tk，可离线测试。

发送端按 JSON 数据报广播 ``{"type": "traffic_signal", "seq": ..., "h": ..., "v": ...,
"mask_hex": ..., "phase": ...}``，默认 UDP 8765。本模块只做三件事：收包、校验、
保留最近一次合法报文；控制与界面侧用 ``snapshot()`` 取快照，或在构造时用
``on_update`` 注册纯数据回调（回调在接收线程执行，不得访问 Tk 控件）。

断线、切车、退出时先 ``stop()`` 再 ``reset()``（或直接丢弃实例），避免上一会话的
报文继续被当成新会话的状态。
"""
import json
import socket
import threading
import time

DEFAULT_PORT = 8765
RECV_TIMEOUT = .4


class SignalReceiver:
    """后台线程收 UDP 包并维护最新灯态；调用方用 ``snapshot()`` 取快照。"""

    def __init__(self, port=DEFAULT_PORT, bind_addr='', timeout=3.0,
                 clock=time.monotonic, socket_factory=socket.socket, on_update=None):
        self.port = port
        self.bind_addr = bind_addr
        self.timeout = timeout
        self.clock = clock
        self.socket_factory = socket_factory
        self.on_update = on_update
        self._lock = threading.Lock()
        self._sock = None
        self._thread = None
        self._stop = threading.Event()

        self.state = None          # 最近一次合法报文（dict）
        self.peer = None           # 发送方 IP
        self.last_rx = 0.0         # 最后收包时间（clock 时基）
        self.seq = 0
        self.packets = 0
        self.bad_packets = 0
        self.last_error = None

    # ---------- 生命周期 ----------
    def start(self):
        """绑定端口并启动接收线程；绑定失败返回 False，原因见 ``last_error``。"""
        thread = self._thread
        if thread is not None and thread.is_alive():
            return True
        sock = None
        try:
            sock = self.socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.settimeout(RECV_TIMEOUT)
            sock.bind((self.bind_addr, self.port))
        except (OSError, OverflowError) as exc:
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
            self.last_error = "绑定端口 %s 失败：%s" % (self.port, exc)
            return False
        self._sock = sock
        self.last_error = None
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, args=(sock,),
                                        name="signal-receiver", daemon=True)
        self._thread.start()
        return True

    def stop(self):
        """停止接收线程并关闭套接字；可重复调用。"""
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=1.2)
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def reset(self):
        """清空会话数据（灯态、对端、计数）；不改变绑定状态和 ``last_error``。"""
        with self._lock:
            self.state = None
            self.peer = None
            self.last_rx = 0.0
            self.seq = 0
            self.packets = 0
            self.bad_packets = 0

    # ---------- 内部 ----------
    @staticmethod
    def _parse(data):
        """合法报文返回 dict，其余（非法 JSON、非 dict、非本类型）返回 None。"""
        try:
            message = json.loads(data.decode('utf-8'))
        except (UnicodeDecodeError, ValueError):
            return None
        if not isinstance(message, dict) or message.get('type') != 'traffic_signal':
            return None
        return message

    def _loop(self, sock):
        while not self._stop.is_set():
            try:
                data, peer = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            message = self._parse(data)
            if message is None:
                with self._lock:
                    self.bad_packets += 1
                continue
            with self._lock:
                self.state = message
                self.peer = peer[0]
                self.last_rx = self.clock()
                self.seq = message.get('seq', 0)
                self.packets += 1
                snapshot = self._snapshot_locked()
            if self.on_update is not None:
                try:
                    # 纯数据回调：异常不能打断收包线程。
                    self.on_update(snapshot)
                except Exception:
                    pass

    # ---------- 对外 ----------
    @property
    def running(self):
        """接收线程是否仍在运行；绑定失败或已停止时为 False。"""
        thread = self._thread
        return thread is not None and thread.is_alive()

    def snapshot(self):
        with self._lock:
            return self._snapshot_locked()

    def _snapshot_locked(self):
        age = self.clock() - self.last_rx if self.last_rx else None
        return {
            'state': dict(self.state) if self.state else None,
            'peer': self.peer,
            'age': age,
            'seq': self.seq,
            'packets': self.packets,
            'bad': self.bad_packets,
            'stale': (age is None) or (age > self.timeout),
            'error': self.last_error,
        }
