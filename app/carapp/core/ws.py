# -*- coding: utf-8 -*-
"""极简 WebSocket 客户端（纯标准库，RFC6455 子集）。

为什么自己写：目标是"复制到任何一台电脑双击就能跑"，所以不想依赖
websocket-client / paramiko 这类第三方库。我们只需要 text 帧收发，
足够驱动 rosbridge。
"""

import base64
import os
import socket
import struct
import threading
import time


class WebSocket:
    def __init__(self, host, port=9090, timeout=5.0):
        self.host = host
        self.port = port
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)
        self._buf = b""
        self._send_lock = threading.Lock()
        self.last_received = time.monotonic()
        try:
            self._handshake()
        except Exception:
            self.close()
            raise

    def _handshake(self):
        key = base64.b64encode(os.urandom(16)).decode()
        req = (
            "GET / HTTP/1.1\r\n"
            "Host: %s:%d\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Key: %s\r\n"
            "Sec-WebSocket-Version: 13\r\n"
            "\r\n" % (self.host, self.port, key)
        )
        self.sock.sendall(req.encode())
        buf = b""
        while b"\r\n\r\n" not in buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise RuntimeError("WebSocket 握手被关闭")
            buf += chunk
        head, _, rest = buf.partition(b"\r\n\r\n")
        first = head.split(b"\r\n")[0].decode("latin1")
        if "101" not in first:
            raise RuntimeError("WebSocket 握手失败: " + first)
        self._buf = rest

    def send_text(self, text):
        payload = text.encode("utf-8")
        header = bytearray([0x81])  # FIN + text
        n = len(payload)
        if n < 126:
            header.append(0x80 | n)
        elif n < 65536:
            header.append(0x80 | 126)
            header += struct.pack(">H", n)
        else:
            header.append(0x80 | 127)
            header += struct.pack(">Q", n)
        mask = os.urandom(4)
        header += mask
        body = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        with self._send_lock:
            self.sock.sendall(bytes(header) + body)

    def _read_exact(self, n):
        out = b""
        while len(out) < n:
            if self._buf:
                take = self._buf[: n - len(out)]
                self._buf = self._buf[len(take):]
                out += take
                continue
            chunk = self.sock.recv(65536)
            if not chunk:
                raise RuntimeError("连接已关闭")
            self.last_received = time.monotonic()
            self._buf += chunk
        return out

    def recv_text(self):
        """返回一条文本消息；控制帧返回 None。"""
        b1, b2 = self._read_exact(2)
        opcode = b1 & 0x0F
        masked = b2 & 0x80
        n = b2 & 0x7F
        if n == 126:
            n = struct.unpack(">H", self._read_exact(2))[0]
        elif n == 127:
            n = struct.unpack(">Q", self._read_exact(8))[0]
        mask = self._read_exact(4) if masked else None
        payload = self._read_exact(n)
        if mask:
            payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        if opcode == 0x8:
            raise RuntimeError("服务端关闭连接")
        if opcode == 0x9:  # ping -> pong
            self._send_control(0x0A, payload)
            return None
        if opcode == 0xA:
            return None
        if opcode in (0x1, 0x2):
            return payload.decode("utf-8", "replace")
        return None

    def _send_control(self, opcode, data):
        header = bytearray([0x80 | opcode])
        n = len(data)
        header.append(0x80 | n)
        mask = os.urandom(4)
        header += mask
        body = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        with self._send_lock:
            self.sock.sendall(bytes(header) + body)

    def ping(self):
        self._send_control(0x09, b"carapp")

    def close(self):
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            self.sock.close()
        except Exception:
            pass
