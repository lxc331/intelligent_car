# -*- coding: utf-8 -*-
"""rosbridge 客户端：把 WebSocket 上的 JSON 协议封装成好用的对象。

对外提供三件事：
    publish(topic, msg)          发布话题（控制舵机就走这条）
    subscribe(topic, throttle)   订阅话题（里程计、电池走这条）
    call_service(srv, args)      调服务（查节点列表走这条）

内部单线程 while True，断线自动重连，状态变化通过 on_status 回调告诉 UI。
"""

import json
import threading
import time

from carapp.core.ws import WebSocket


class RosBridge:
    def __init__(self, host, port=9090, on_message=None, on_status=None, log=None):
        self.host = host
        self.port = port
        self.on_message = on_message or (lambda topic, msg: None)
        self.on_status = on_status or (lambda connected, detail="": None)
        self.log = log or (lambda text: None)

        self._ws = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._connected = False
        self._thread = None
        self._subs = {}          # topic -> throttle_ms
        self._svc_waiters = {}   # id -> callback
        self._svc_seq = 0

    # ---------- 对外接口 ----------
    @property
    def connected(self):
        return self._connected

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="rosbridge", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._close()

    def publish(self, topic, msg):
        return self._send({"op": "publish", "topic": topic, "msg": msg})

    def subscribe(self, topic, throttle_ms=0):
        self._subs[topic] = throttle_ms
        payload = {"op": "subscribe", "topic": topic}
        if throttle_ms:
            payload["throttle_rate"] = throttle_ms
        return self._send(payload)

    def call_service(self, service, args=None, callback=None):
        self._svc_seq += 1
        sid = "srv-%d" % self._svc_seq
        if callback:
            self._svc_waiters[sid] = callback
        return self._send({"op": "call_service", "id": sid, "service": service,
                           "args": args or {}})

    # ---------- 内部 ----------
    def _send(self, obj):
        with self._lock:
            if not self._ws:
                return False
            try:
                self._ws.send_text(json.dumps(obj))
                return True
            except Exception:
                return False

    def _close(self):
        with self._lock:
            if self._ws:
                self._ws.close()
            self._ws = None
        if self._connected:
            self._connected = False
            self.on_status(False, "连接已断开")

    def _set_connected(self, value, detail=""):
        if self._connected != value:
            self._connected = value
            self.on_status(value, detail)

    def _run(self):
        while not self._stop.is_set():
            try:
                ws = WebSocket(self.host, self.port, timeout=6)
                with self._lock:
                    self._ws = ws
                self._set_connected(True, "ws://%s:%d" % (self.host, self.port))
                self.log("已连接 rosbridge ws://%s:%d" % (self.host, self.port))
                # 重连后把之前的订阅补回来
                for topic, throttle in self._subs.items():
                    payload = {"op": "subscribe", "topic": topic}
                    if throttle:
                        payload["throttle_rate"] = throttle
                    ws.send_text(json.dumps(payload))
                ws.sock.settimeout(None)
                while not self._stop.is_set():
                    raw = ws.recv_text()
                    if raw:
                        self._dispatch(raw)
            except Exception as exc:
                self._set_connected(False, str(exc))
                self.log("rosbridge 断开：%s，3 秒后重连" % exc)
                time.sleep(3)
            finally:
                self._close()

    def _dispatch(self, raw):
        try:
            data = json.loads(raw)
        except Exception:
            return
        op = data.get("op")
        if op == "publish":
            topic = data.get("topic", "")
            self.on_message(topic, data.get("msg"))
        elif op == "service_response":
            cb = self._svc_waiters.pop(data.get("id"), None)
            if cb:
                try:
                    cb(data.get("values", {}), data.get("result"))
                except Exception:
                    pass
        elif op == "status":
            level = data.get("level")
            if level in ("error", "warning"):
                self.log("rosbridge 提示：%s" % data.get("msg"))
